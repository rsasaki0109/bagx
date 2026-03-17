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
    is_static: bool = False
    allan_accel_vrw: float = float("nan")  # m/s/sqrt(s), from Allan @ tau=1s
    allan_gyro_arw: float = float("nan")   # rad/sqrt(s), from Allan @ tau=1s
    allan_accel_bias_instab: float = float("nan")  # m/s², Allan dev minimum
    allan_gyro_bias_instab: float = float("nan")   # rad/s, Allan dev minimum
    noise_note: str = ""
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

    # Collect messages per topic
    gnss_messages: list[Message] = []
    imu_messages_by_topic: dict[str, list[Message]] = {t: [] for t in imu_topics}
    topic_timestamps: dict[str, list[int]] = {t: [] for t in all_topics}

    # Read all messages if we need sync analysis
    for msg in reader.read_messages(topics=None):
        if msg.topic in topic_timestamps:
            topic_timestamps[msg.topic].append(msg.timestamp_ns)
        if msg.topic in gnss_topics:
            gnss_messages.append(msg)
        elif msg.topic in imu_messages_by_topic:
            imu_messages_by_topic[msg.topic].append(msg)

    # Evaluate GNSS
    if gnss_messages:
        report.gnss = _evaluate_gnss(gnss_messages, config)
    else:
        logger.info("No GNSS topics found in %s", bag_path)

    # Evaluate IMU — per-topic, take best score
    if imu_messages_by_topic:
        best_imu: ImuMetrics | None = None
        for topic_name, msgs in imu_messages_by_topic.items():
            if not msgs:
                continue
            imu_result = _evaluate_imu(msgs, config)
            logger.debug(
                "IMU topic %s: score=%.1f, accel_noise=(%.4f,%.4f,%.4f)",
                topic_name, imu_result.score,
                imu_result.accel_noise_x, imu_result.accel_noise_y, imu_result.accel_noise_z,
            )
            if best_imu is None or imu_result.score > best_imu.score:
                best_imu = imu_result
        report.imu = best_imu
    if report.imu is None:
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

    # Noise estimation via first-order differencing.
    # Mathematically equivalent to Allan Deviation at tau=dt (one sample interval).
    # Removes low-frequency motion components (vehicle dynamics), isolating sensor noise.
    # Divide by sqrt(2) because diff doubles white noise variance: Var(x[n]-x[n-1]) = 2*sigma^2.
    if len(accel_x) > 2:
        metrics.accel_noise_x = float(np.std(np.diff(accel_x)) / math.sqrt(2))
        metrics.accel_noise_y = float(np.std(np.diff(accel_y)) / math.sqrt(2))
        metrics.accel_noise_z = float(np.std(np.diff(accel_z)) / math.sqrt(2))

    if len(gyro_x) > 2:
        metrics.gyro_noise_x = float(np.std(np.diff(gyro_x)) / math.sqrt(2))
        metrics.gyro_noise_y = float(np.std(np.diff(gyro_y)) / math.sqrt(2))
        metrics.gyro_noise_z = float(np.std(np.diff(gyro_z)) / math.sqrt(2))

    # Frequency
    dt = float("nan")
    if len(timestamps) >= 2:
        ts = np.array(timestamps, dtype=np.float64)
        diffs = np.diff(ts) / 1e9  # to seconds
        diffs = diffs[diffs > 0]
        if len(diffs) > 0:
            dt = float(np.median(diffs))
            metrics.frequency_hz = float(1.0 / dt)

    # Detect if data is static: accel magnitude should be ~9.81 with low variance
    if len(accel_x) > 100:
        accel_mag = np.sqrt(np.array(accel_x)**2 + np.array(accel_y)**2 + np.array(accel_z)**2)
        accel_mag_std = float(np.std(accel_mag))
        accel_mag_mean = float(np.mean(accel_mag))
        # Static if: mean magnitude ~9.81 (±2) and low std (<0.5 m/s²)
        metrics.is_static = (abs(accel_mag_mean - 9.81) < 2.0 and accel_mag_std < 0.5)

    if metrics.is_static and not math.isnan(dt) and len(accel_x) > 500:
        # Allan Variance on static data — gives reliable VRW/ARW and bias instability
        allan_a = _compute_allan_dev(np.array(accel_x), dt)
        allan_g = _compute_allan_dev(np.array(gyro_x), dt)

        if allan_a is not None:
            metrics.allan_accel_vrw = allan_a["adev_1s"]
            metrics.allan_accel_bias_instab = allan_a["bias_instab"]
            metrics.accel_bias_stability = allan_a["bias_instab"]
        if allan_g is not None:
            metrics.allan_gyro_arw = allan_g["adev_1s"]
            metrics.allan_gyro_bias_instab = allan_g["bias_instab"]
            metrics.gyro_bias_stability = allan_g["bias_instab"]

        metrics.noise_note = "Static data detected — Allan Variance computed for VRW/ARW and bias instability."
    else:
        # Dynamic data: use windowed mean drift as bias stability approximation
        if len(accel_x) > 100:
            window = len(accel_x) // 10
            windowed_means = [
                np.mean(accel_x[i : i + window]) for i in range(0, len(accel_x) - window, window)
            ]
            metrics.accel_bias_stability = float(np.std(windowed_means)) if len(windowed_means) > 1 else float("nan")

        if len(gyro_x) > 100:
            window = len(gyro_x) // 10
            windowed_means = [
                np.mean(gyro_x[i : i + window]) for i in range(0, len(gyro_x) - window, window)
            ]
            metrics.gyro_bias_stability = float(np.std(windowed_means)) if len(windowed_means) > 1 else float("nan")

        duration_sec = len(accel_x) * dt if not math.isnan(dt) else 0
        if duration_sec < 60:
            metrics.noise_note = "Short recording — for bias instability, record >60s of static data."
        else:
            metrics.noise_note = "Dynamic data — noise via diff (=Allan τ=dt). For bias instability, record static data."

    logger.debug(
        "IMU static=%s, note=%s", metrics.is_static, metrics.noise_note
    )

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


def _compute_allan_dev(data: np.ndarray, dt: float) -> dict | None:
    """Compute Allan Deviation at multiple tau values.

    Returns dict with adev_1s (at tau~1s) and bias_instab (minimum adev).
    Returns None if data is too short.
    """
    N = len(data)
    if N < 100:
        return None

    # Logarithmically-spaced cluster sizes
    cluster_sizes = np.unique(np.logspace(0, np.log10(N // 2), 50).astype(int))
    cluster_sizes = cluster_sizes[(cluster_sizes >= 1) & (cluster_sizes * 2 <= N)]

    if len(cluster_sizes) == 0:
        return None

    theta = np.cumsum(data) * dt  # integrated signal
    taus = []
    adevs = []

    for m in cluster_sizes:
        tau = m * dt
        # Non-overlapping Allan variance for speed
        n_chunks = (N - 1) // m
        if n_chunks < 2:
            continue
        chunks = np.array([
            theta[(i + 1) * m] - theta[i * m]
            for i in range(n_chunks)
        ])
        avar = float(np.mean(np.diff(chunks) ** 2) / (2 * tau ** 2))
        taus.append(tau)
        adevs.append(math.sqrt(avar))

    if not adevs:
        return None

    taus = np.array(taus)
    adevs = np.array(adevs)

    # adev at tau ~= 1s
    idx_1s = int(np.argmin(np.abs(taus - 1.0)))
    adev_1s = float(adevs[idx_1s])

    # Bias instability = minimum adev
    bias_instab = float(np.min(adevs))

    return {"adev_1s": adev_1s, "bias_instab": bias_instab}


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
        if not math.isnan(m.allan_accel_vrw):
            table.add_row("Allan VRW (accel)", f"{m.allan_accel_vrw:.6f} m/s/√s")
            table.add_row("Allan ARW (gyro)", f"{m.allan_gyro_arw:.6f} rad/√s")
            table.add_row("Allan Bias (accel)", f"{m.allan_accel_bias_instab:.6f} m/s²")
            table.add_row("Allan Bias (gyro)", f"{m.allan_gyro_bias_instab:.6f} rad/s")
        table.add_row("Data Type", "[green]Static[/]" if m.is_static else "[yellow]Dynamic[/]")
        table.add_row(
            "Score",
            f"[{'green' if m.score >= 70 else 'yellow' if m.score >= 40 else 'red'}]{m.score:.1f}/100[/]",
        )
        console.print(table)
        if m.noise_note:
            console.print(f"  [dim]{m.noise_note}[/dim]")

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
