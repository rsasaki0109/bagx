"""Single bag quality evaluation engine.

Evaluates GNSS, IMU, and inter-topic sync quality,
producing a structured quality report with a composite score.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import asdict, dataclass, field
from typing import TextIO

import numpy as np
from rich.console import Console
from rich.table import Table

from bagx.reader import BagReader, Message

logger = logging.getLogger(__name__)


@dataclass
class EvalConfig:
    """Configurable scoring thresholds for evaluation.

    Users can tune these for their sensor suite via the Python API.
    """

    gnss_fix_weight: float = 0.6
    gnss_hdop_weight: float = 0.4
    gnss_hdop_scale: float = 10.0  # HDOP multiplier for score
    imu_accel_noise_excellent: float = 0.05  # m/s²
    imu_accel_noise_scale: float = 200.0
    imu_gyro_noise_excellent: float = 0.005  # rad/s
    imu_gyro_noise_scale: float = 2000.0
    sync_delay_excellent_ms: float = 5.0
    sync_delay_scale: float = 1.5


@dataclass
class GnssMetrics:
    total_messages: int = 0
    fix_count: int = 0
    no_fix_count: int = 0
    fix_rate: float = 0.0
    hdop_mean: float = float("nan")
    hdop_std: float = float("nan")
    hdop_max: float = float("nan")
    latitude_range: tuple[float, float] = (0.0, 0.0)
    longitude_range: tuple[float, float] = (0.0, 0.0)
    altitude_mean: float = float("nan")
    altitude_std: float = float("nan")
    score: float = 0.0


@dataclass
class ImuMetrics:
    total_messages: int = 0
    accel_noise_x: float = float("nan")
    accel_noise_y: float = float("nan")
    accel_noise_z: float = float("nan")
    gyro_noise_x: float = float("nan")
    gyro_noise_y: float = float("nan")
    gyro_noise_z: float = float("nan")
    accel_bias_stability: float = float("nan")
    gyro_bias_stability: float = float("nan")
    frequency_hz: float = float("nan")
    score: float = 0.0


@dataclass
class SyncMetrics:
    topic_pairs: list[tuple[str, str]] = field(default_factory=list)
    mean_delay_ms: list[float] = field(default_factory=list)
    max_delay_ms: list[float] = field(default_factory=list)
    std_delay_ms: list[float] = field(default_factory=list)
    score: float = 0.0


@dataclass
class EvalReport:
    bag_path: str
    duration_sec: float
    total_messages: int
    topic_count: int
    gnss: GnssMetrics | None = None
    imu: ImuMetrics | None = None
    sync: SyncMetrics | None = None
    overall_score: float = 0.0

    def to_dict(self) -> dict:
        d = asdict(self)
        # Clean up NaN for JSON
        return _clean_nan(d)


def _clean_nan(obj):
    if isinstance(obj, dict):
        return {k: _clean_nan(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_clean_nan(v) for v in obj]
    if isinstance(obj, tuple):
        return tuple(_clean_nan(v) for v in obj)
    if isinstance(obj, float) and math.isnan(obj):
        return None
    return obj


def evaluate_bag(
    bag_path: str,
    output_json: TextIO | None = None,
    config: EvalConfig | None = None,
) -> EvalReport:
    """Run full quality evaluation on a bag file."""
    if config is None:
        config = EvalConfig()

    reader = BagReader(bag_path)
    summary = reader.summary()

    report = EvalReport(
        bag_path=str(bag_path),
        duration_sec=summary.duration_sec,
        total_messages=summary.message_count,
        topic_count=len(summary.topics),
    )

    # Classify topics
    gnss_topics = []
    imu_topics = []
    all_topics = []

    for name, info in summary.topics.items():
        all_topics.append(name)
        if "NavSatFix" in info.type or "navsatfix" in info.type.lower():
            gnss_topics.append(name)
        elif "Imu" in info.type or "imu" in info.type.lower():
            imu_topics.append(name)

    # Collect messages
    gnss_messages: list[Message] = []
    imu_messages: list[Message] = []
    topic_timestamps: dict[str, list[int]] = {t: [] for t in all_topics}

    # Read all messages if we need sync analysis
    for msg in reader.read_messages(topics=None):
        if msg.topic in topic_timestamps:
            topic_timestamps[msg.topic].append(msg.timestamp_ns)
        if msg.topic in gnss_topics:
            gnss_messages.append(msg)
        elif msg.topic in imu_topics:
            imu_messages.append(msg)

    # Evaluate GNSS
    if gnss_messages:
        report.gnss = _evaluate_gnss(gnss_messages, config)
    else:
        logger.info("No GNSS topics found in %s", bag_path)

    # Evaluate IMU
    if imu_messages:
        report.imu = _evaluate_imu(imu_messages, config)
    else:
        logger.info("No IMU topics found in %s", bag_path)

    # Evaluate sync
    sync_topics = [t for t in all_topics if len(topic_timestamps.get(t, [])) > 10]
    if len(sync_topics) >= 2:
        report.sync = _evaluate_sync(topic_timestamps, sync_topics, config)

    # Overall score
    scores = []
    if report.gnss:
        scores.append(report.gnss.score)
    if report.imu:
        scores.append(report.imu.score)
    if report.sync:
        scores.append(report.sync.score)

    report.overall_score = float(np.mean(scores)) if scores else 0.0

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def _evaluate_gnss(messages: list[Message], config: EvalConfig) -> GnssMetrics:
    metrics = GnssMetrics(total_messages=len(messages))

    statuses = []
    latitudes = []
    longitudes = []
    altitudes = []
    hdops = []

    for msg in messages:
        d = msg.data
        status = d.get("status", -1)
        statuses.append(status)

        if status >= 0:  # STATUS_FIX or better
            metrics.fix_count += 1
        else:
            metrics.no_fix_count += 1

        lat = d.get("latitude")
        lon = d.get("longitude")
        alt = d.get("altitude")

        if lat is not None and not math.isnan(lat):
            latitudes.append(lat)
        if lon is not None and not math.isnan(lon):
            longitudes.append(lon)
        if alt is not None and not math.isnan(alt):
            altitudes.append(alt)

        # HDOP from covariance (horizontal = sqrt(cov[0]))
        cov = d.get("position_covariance", [])
        if cov and len(cov) >= 1 and cov[0] > 0:
            hdops.append(math.sqrt(cov[0]))

    metrics.fix_rate = metrics.fix_count / max(metrics.total_messages, 1)

    if hdops:
        metrics.hdop_mean = float(np.mean(hdops))
        metrics.hdop_std = float(np.std(hdops))
        metrics.hdop_max = float(np.max(hdops))

    if latitudes:
        metrics.latitude_range = (min(latitudes), max(latitudes))
    if longitudes:
        metrics.longitude_range = (min(longitudes), max(longitudes))
    if altitudes:
        metrics.altitude_mean = float(np.mean(altitudes))
        metrics.altitude_std = float(np.std(altitudes))

    # Score: weighted combination
    fix_score = min(metrics.fix_rate * 100, 100)
    hdop_score = (
        max(0, 100 - metrics.hdop_mean * config.gnss_hdop_scale)
        if not math.isnan(metrics.hdop_mean)
        else 50
    )
    metrics.score = fix_score * config.gnss_fix_weight + hdop_score * config.gnss_hdop_weight

    return metrics


def _evaluate_imu(messages: list[Message], config: EvalConfig) -> ImuMetrics:
    metrics = ImuMetrics(total_messages=len(messages))

    accel_x, accel_y, accel_z = [], [], []
    gyro_x, gyro_y, gyro_z = [], [], []
    timestamps = []

    for msg in messages:
        d = msg.data
        timestamps.append(msg.timestamp_ns)

        la = d.get("linear_acceleration", {})
        if la:
            accel_x.append(la.get("x", 0.0))
            accel_y.append(la.get("y", 0.0))
            accel_z.append(la.get("z", 0.0))

        av = d.get("angular_velocity", {})
        if av:
            gyro_x.append(av.get("x", 0.0))
            gyro_y.append(av.get("y", 0.0))
            gyro_z.append(av.get("z", 0.0))

    if accel_x:
        metrics.accel_noise_x = float(np.std(accel_x))
        metrics.accel_noise_y = float(np.std(accel_y))
        metrics.accel_noise_z = float(np.std(accel_z))

    if gyro_x:
        metrics.gyro_noise_x = float(np.std(gyro_x))
        metrics.gyro_noise_y = float(np.std(gyro_y))
        metrics.gyro_noise_z = float(np.std(gyro_z))

    # Bias stability via Allan variance approximation (windowed std)
    if len(accel_x) > 100:
        window = len(accel_x) // 10
        windowed = [
            np.std(accel_x[i : i + window]) for i in range(0, len(accel_x) - window, window)
        ]
        metrics.accel_bias_stability = float(np.min(windowed)) if windowed else float("nan")

    if len(gyro_x) > 100:
        window = len(gyro_x) // 10
        windowed = [
            np.std(gyro_x[i : i + window]) for i in range(0, len(gyro_x) - window, window)
        ]
        metrics.gyro_bias_stability = float(np.min(windowed)) if windowed else float("nan")

    # Frequency
    if len(timestamps) >= 2:
        ts = np.array(timestamps, dtype=np.float64)
        diffs = np.diff(ts) / 1e9  # to seconds
        diffs = diffs[diffs > 0]
        if len(diffs) > 0:
            metrics.frequency_hz = float(1.0 / np.median(diffs))

    # Score: low noise is good
    accel_noise = (
        np.mean(
            [
                n
                for n in [metrics.accel_noise_x, metrics.accel_noise_y, metrics.accel_noise_z]
                if not math.isnan(n)
            ]
        )
        if accel_x
        else 1.0
    )
    gyro_noise = (
        np.mean(
            [
                n
                for n in [metrics.gyro_noise_x, metrics.gyro_noise_y, metrics.gyro_noise_z]
                if not math.isnan(n)
            ]
        )
        if gyro_x
        else 1.0
    )

    # Normalize using config thresholds
    accel_score = max(
        0, min(100, 100 - (accel_noise - config.imu_accel_noise_excellent) * config.imu_accel_noise_scale)
    )
    gyro_score = max(
        0, min(100, 100 - (gyro_noise - config.imu_gyro_noise_excellent) * config.imu_gyro_noise_scale)
    )

    metrics.score = accel_score * 0.5 + gyro_score * 0.5

    return metrics


def _evaluate_sync(
    topic_timestamps: dict[str, list[int]],
    topics: list[str],
    config: EvalConfig,
) -> SyncMetrics:
    metrics = SyncMetrics()

    # Evaluate pairs of adjacent topics (by name order)
    sorted_topics = sorted(topics)
    pairs = list(zip(sorted_topics[:-1], sorted_topics[1:]))
    if not pairs:
        return metrics

    all_delays = []

    for t1, t2 in pairs[:5]:  # Limit to 5 pairs
        ts1 = np.array(sorted(topic_timestamps[t1]), dtype=np.int64)
        ts2 = np.array(sorted(topic_timestamps[t2]), dtype=np.int64)

        # For each message in t1, find nearest in t2
        delays = []
        j = 0
        for t in ts1:
            while j < len(ts2) - 1 and abs(ts2[j + 1] - t) < abs(ts2[j] - t):
                j += 1
            if j < len(ts2):
                delays.append(abs(int(ts2[j] - t)))

        if delays:
            delay_ms = np.array(delays) / 1e6  # ns to ms
            metrics.topic_pairs.append((t1, t2))
            metrics.mean_delay_ms.append(float(np.mean(delay_ms)))
            metrics.max_delay_ms.append(float(np.max(delay_ms)))
            metrics.std_delay_ms.append(float(np.std(delay_ms)))
            all_delays.extend(delay_ms.tolist())

    if all_delays:
        mean_delay = np.mean(all_delays)
        # Score: configurable thresholds
        metrics.score = max(
            0, min(100, 100 - (mean_delay - config.sync_delay_excellent_ms) * config.sync_delay_scale)
        )

    return metrics


def print_eval_report(report: EvalReport, console: Console | None = None) -> None:
    """Pretty-print an evaluation report."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Bag Evaluation: [cyan]{report.bag_path}[/cyan][/bold]")
    console.print(
        f"Duration: {report.duration_sec:.1f}s | Messages: {report.total_messages:,} | Topics: {report.topic_count}"
    )

    if report.gnss:
        g = report.gnss
        table = Table(title="GNSS Quality", show_header=True)
        table.add_column("Metric", style="bold")
        table.add_column("Value", justify="right")
        table.add_row("Messages", str(g.total_messages))
        table.add_row("Fix Rate", f"{g.fix_rate:.1%}")
        table.add_row(
            "HDOP (mean)", f"{g.hdop_mean:.2f}" if not math.isnan(g.hdop_mean) else "N/A"
        )
        table.add_row(
            "HDOP (max)", f"{g.hdop_max:.2f}" if not math.isnan(g.hdop_max) else "N/A"
        )
        table.add_row(
            "Altitude (mean+/-std)",
            f"{g.altitude_mean:.1f}+/-{g.altitude_std:.1f}"
            if not math.isnan(g.altitude_mean)
            else "N/A",
        )
        table.add_row(
            "Score",
            f"[{'green' if g.score >= 70 else 'yellow' if g.score >= 40 else 'red'}]{g.score:.1f}/100[/]",
        )
        console.print(table)

    if report.imu:
        m = report.imu
        table = Table(title="IMU Quality", show_header=True)
        table.add_column("Metric", style="bold")
        table.add_column("Value", justify="right")
        table.add_row("Messages", str(m.total_messages))
        table.add_row(
            "Frequency",
            f"{m.frequency_hz:.1f} Hz" if not math.isnan(m.frequency_hz) else "N/A",
        )
        table.add_row(
            "Accel Noise (xyz)",
            f"{m.accel_noise_x:.4f}, {m.accel_noise_y:.4f}, {m.accel_noise_z:.4f}"
            if not math.isnan(m.accel_noise_x)
            else "N/A",
        )
        table.add_row(
            "Gyro Noise (xyz)",
            f"{m.gyro_noise_x:.6f}, {m.gyro_noise_y:.6f}, {m.gyro_noise_z:.6f}"
            if not math.isnan(m.gyro_noise_x)
            else "N/A",
        )
        table.add_row(
            "Accel Bias Stability",
            f"{m.accel_bias_stability:.4f}" if not math.isnan(m.accel_bias_stability) else "N/A",
        )
        table.add_row(
            "Score",
            f"[{'green' if m.score >= 70 else 'yellow' if m.score >= 40 else 'red'}]{m.score:.1f}/100[/]",
        )
        console.print(table)

    if report.sync:
        s = report.sync
        table = Table(title="Topic Sync Quality", show_header=True)
        table.add_column("Topic Pair", style="bold")
        table.add_column("Mean (ms)", justify="right")
        table.add_column("Max (ms)", justify="right")
        table.add_column("Std (ms)", justify="right")
        for i, (t1, t2) in enumerate(s.topic_pairs):
            table.add_row(
                f"{t1} <-> {t2}",
                f"{s.mean_delay_ms[i]:.1f}",
                f"{s.max_delay_ms[i]:.1f}",
                f"{s.std_delay_ms[i]:.1f}",
            )
        table.add_row(
            "Score",
            f"[{'green' if s.score >= 70 else 'yellow' if s.score >= 40 else 'red'}]{s.score:.1f}/100[/]",
            "",
            "",
        )
        console.print(table)

    color = (
        "green" if report.overall_score >= 70 else "yellow" if report.overall_score >= 40 else "red"
    )
    console.print(
        f"\n[bold]Overall Score: [{color}]{report.overall_score:.1f}/100[/{color}][/bold]\n"
    )
