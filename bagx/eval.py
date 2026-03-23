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

from bagx.contracts import report_metadata
from bagx.reader import BagReader, Message
from bagx.topic_filters import is_sync_candidate

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
    imu_topic: str = ""
    sync: SyncMetrics | None = None
    domain_score: float | None = None
    overall_score: float = 0.0
    topic_info: dict = field(default_factory=dict)  # {name: {"type": str, "count": int, "rate_hz": float}}
    _topic_timestamps: dict = field(default_factory=dict, repr=False)  # internal, not serialized

    def to_dict(self) -> dict:
        d = asdict(self)
        # Remove internal fields
        d.pop("_topic_timestamps", None)
        # Clean up NaN for JSON
        d = _clean_nan(d)
        # Add recommendations
        d["recommendations"] = [
            _strip_rich_markup(r) for r in generate_recommendations(self)
        ]
        d.update(report_metadata("eval"))
        return d


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
                report.imu_topic = topic_name
        report.imu = best_imu
    if report.imu is None:
        logger.info("No IMU topics found in %s", bag_path)

    # Store timestamps for domain-specific pipeline latency analysis
    report._topic_timestamps = topic_timestamps

    # Build topic info with rates for domain detection
    for name, info in summary.topics.items():
        ts_list = topic_timestamps.get(name, [])
        rate_hz = 0.0
        if len(ts_list) >= 2:
            ts_sorted = sorted(ts_list)
            dur = (ts_sorted[-1] - ts_sorted[0]) / 1e9
            if dur > 0:
                rate_hz = (len(ts_list) - 1) / dur
        report.topic_info[name] = {
            "type": info.type,
            "count": info.count,
            "rate_hz": round(rate_hz, 1),
        }

    domains = _detect_domain_names(report.topic_info)

    # Evaluate sync
    sync_topics = [
        t
        for t in all_topics
        if len(topic_timestamps.get(t, [])) > 10
        and is_sync_candidate(t, summary.topics[t].type)
    ]
    if len(sync_topics) >= 2 and not ({"Nav2", "MoveIt"} & domains):
        report.sync = _evaluate_sync(topic_timestamps, sync_topics, config)

    # Overall score
    report.domain_score = _compute_domain_score(report)
    scores = []
    if report.gnss:
        scores.append(report.gnss.score)
    if report.imu:
        scores.append(report.imu.score)
    if report.sync and not ({"Nav2", "MoveIt"} & domains):
        scores.append(report.sync.score)
    if report.domain_score is not None:
        scores.append(report.domain_score)

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

    # Frequency: use duration/count to handle bursty timestamps correctly
    # (median of intervals fails when messages arrive in bursts)
    dt = float("nan")
    if len(timestamps) >= 2:
        ts_sorted = sorted(timestamps)
        duration_s = (ts_sorted[-1] - ts_sorted[0]) / 1e9
        if duration_s > 0:
            metrics.frequency_hz = float((len(timestamps) - 1) / duration_s)
            dt = float(duration_s / (len(timestamps) - 1))

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
    valid_accel_noise = [
        n
        for n in [metrics.accel_noise_x, metrics.accel_noise_y, metrics.accel_noise_z]
        if not math.isnan(n)
    ]
    accel_noise = float(np.mean(valid_accel_noise)) if valid_accel_noise else 1.0

    valid_gyro_noise = [
        n
        for n in [metrics.gyro_noise_x, metrics.gyro_noise_y, metrics.gyro_noise_z]
        if not math.isnan(n)
    ]
    gyro_noise = float(np.mean(valid_gyro_noise)) if valid_gyro_noise else 1.0

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
        overlap_start = max(int(ts1[0]), int(ts2[0]))
        overlap_end = min(int(ts1[-1]), int(ts2[-1]))
        if overlap_end <= overlap_start:
            continue

        ts1 = ts1[(ts1 >= overlap_start) & (ts1 <= overlap_end)]
        ts2 = ts2[(ts2 >= overlap_start) & (ts2 <= overlap_end)]
        if len(ts1) < 5 or len(ts2) < 5:
            continue

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

    from pathlib import Path as _Path

    bag_name = _Path(report.bag_path).name or _Path(report.bag_path).stem
    console.print(f"\n[bold]Bag Evaluation: [cyan]{bag_name}[/cyan][/bold]")
    console.print(f"[dim]{report.bag_path}[/dim]")
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
        imu_title = "IMU Quality"
        if report.imu_topic:
            imu_title += f" ({report.imu_topic})"
        table = Table(title=imu_title, show_header=True)
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

    if report.domain_score is not None:
        domain_color = (
            "green" if report.domain_score >= 70 else "yellow" if report.domain_score >= 40 else "red"
        )
        console.print(
            f"[bold]Domain Score: [{domain_color}]{report.domain_score:.1f}/100[/{domain_color}][/bold]"
        )

    color = (
        "green" if report.overall_score >= 70 else "yellow" if report.overall_score >= 40 else "red"
    )
    console.print(
        f"\n[bold]Overall Score: [{color}]{report.overall_score:.1f}/100[/{color}][/bold]"
    )

    # Actionable recommendations
    recs = generate_recommendations(report)
    if recs:
        console.print("\n[bold]Recommendations:[/bold]")
        for rec in recs:
            console.print(f"  {rec}")
    console.print()


def _strip_rich_markup(text: str) -> str:
    """Remove rich markup tags for plain text output (JSON, etc.)."""
    import re
    return re.sub(r"\[/?[^\]]*\]", "", text).replace(":heavy_check_mark:", "OK").replace(":warning:", "WARNING").replace(":x:", "ERROR").replace(":information_source:", "INFO")


def generate_recommendations(report: EvalReport) -> list[str]:
    """Generate actionable recommendations based on eval results."""
    recs: list[str] = []

    # Detect non-SLAM domains first to suppress irrelevant SLAM advice
    domain_names = _detect_domain_names(report.topic_info)
    is_non_slam_domain = bool(domain_names)

    # --- GNSS ---
    if report.gnss:
        g = report.gnss
        if g.fix_rate >= 0.95:
            recs.append(
                f"[green]:heavy_check_mark:[/green] GNSS fix rate {g.fix_rate:.0%} — suitable as ground truth reference"
            )
        elif g.fix_rate >= 0.5:
            recs.append(
                f"[yellow]:warning:[/yellow] GNSS fix rate {g.fix_rate:.0%} — usable but expect gaps in GNSS-aided SLAM"
            )
        else:
            recs.append(
                f"[red]:x:[/red] GNSS fix rate {g.fix_rate:.0%} — unreliable, do not use as ground truth"
            )

        if not math.isnan(g.hdop_mean) and g.hdop_mean > 5.0:
            recs.append(
                f"[yellow]:warning:[/yellow] HDOP {g.hdop_mean:.1f} is high — GNSS position accuracy is degraded"
            )
    elif not is_non_slam_domain:
        recs.append(
            "[dim]:information_source:[/dim] No GNSS data — ground truth will need an external source (motion capture, total station, etc.)"
        )

    # --- IMU ---
    if report.imu:
        m = report.imu
        accel_noise = np.mean([
            n for n in [m.accel_noise_x, m.accel_noise_y, m.accel_noise_z]
            if not math.isnan(n)
        ]) if not math.isnan(m.accel_noise_x) else float("nan")

        gyro_noise = np.mean([
            n for n in [m.gyro_noise_x, m.gyro_noise_y, m.gyro_noise_z]
            if not math.isnan(n)
        ]) if not math.isnan(m.gyro_noise_x) else float("nan")

        if not math.isnan(accel_noise):
            if accel_noise < 0.05:
                recs.append(
                    f"[green]:heavy_check_mark:[/green] IMU accel noise {accel_noise:.4f} m/s\u00b2 — excellent, set imu_acc_noise_density to {accel_noise:.4f}"
                )
            elif accel_noise < 0.2:
                recs.append(
                    f"[green]:heavy_check_mark:[/green] IMU accel noise {accel_noise:.4f} m/s\u00b2 — good for LIO, set imu_acc_noise_density to {accel_noise:.4f}"
                )
            else:
                if is_non_slam_domain:
                    recs.append(
                        f"[yellow]:warning:[/yellow] IMU accel noise {accel_noise:.4f} m/s\u00b2 — noisy, consider checking IMU calibration"
                    )
                else:
                    recs.append(
                        f"[yellow]:warning:[/yellow] IMU accel noise {accel_noise:.4f} m/s\u00b2 — noisy, LiDAR-only odometry may outperform LIO"
                    )

        if not math.isnan(gyro_noise):
            if gyro_noise < 0.01:
                recs.append(
                    f"[green]:heavy_check_mark:[/green] IMU gyro noise {gyro_noise:.6f} rad/s — set imu_gyro_noise_density to {gyro_noise:.6f}"
                )
            else:
                recs.append(
                    f"[yellow]:warning:[/yellow] IMU gyro noise {gyro_noise:.6f} rad/s — consider lowering IMU integration weight in SLAM"
                )

        if not math.isnan(m.frequency_hz):
            if m.frequency_hz < 100:
                recs.append(
                    f"[yellow]:warning:[/yellow] IMU rate {m.frequency_hz:.0f}Hz is low — 200Hz+ recommended for tightly-coupled LIO"
                )

        if m.noise_note and "static" in m.noise_note.lower() and "record" in m.noise_note.lower():
            recs.append(
                f"[dim]:information_source:[/dim] {m.noise_note}"
            )
    elif not is_non_slam_domain:
        recs.append(
            "[dim]:information_source:[/dim] No IMU data — LiDAR-only odometry (KISS-ICP) recommended"
        )

    # --- Sync ---
    if report.sync and report.sync.mean_delay_ms:
        worst_delay = max(report.sync.mean_delay_ms)
        worst_idx = report.sync.mean_delay_ms.index(worst_delay)
        t1, t2 = report.sync.topic_pairs[worst_idx]

        if worst_delay < 5:
            recs.append(
                f"[green]:heavy_check_mark:[/green] Sensor sync excellent ({worst_delay:.1f}ms) — suitable for tightly-coupled fusion"
            )
        elif worst_delay < 20:
            recs.append(
                f"[green]:heavy_check_mark:[/green] Sensor sync good ({worst_delay:.1f}ms)"
            )
        elif not is_non_slam_domain:
            recs.append(
                f"[yellow]:warning:[/yellow] {t1} \u2194 {t2} sync delay {worst_delay:.0f}ms — enable per-point deskew / timestamp compensation"
            )

    # --- Domain-specific recommendations ---
    recs.extend(_detect_domain_recommendations(report))

    return recs


def _detect_domain_recommendations(report: EvalReport) -> list[str]:
    """Detect ROS2 framework and generate domain-specific recommendations."""
    recs: list[str] = []
    topics = report.topic_info
    if not topics:
        return recs

    domains = _detect_domain_names(topics)
    topic_types = {t: info["type"] for t, info in topics.items()}
    image_topics = _select_topics(
        topics,
        type_markers=("image", "compressedimage"),
    )
    color_image_topics = [
        name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
    ]
    depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
    infra_image_topics = [name for name in image_topics if "infra" in name.lower()]
    camera_info_topics = _select_topics(
        topics,
        type_markers=("camerainfo",),
        contains=("camera_info",),
    )
    color_info_topics = [
        name for name in camera_info_topics if "depth" not in name.lower() and "infra" not in name.lower()
    ]
    depth_info_topics = [name for name in camera_info_topics if "depth" in name.lower()]
    infra_info_topics = [name for name in camera_info_topics if "infra" in name.lower()]

    odom_topics = _select_topics(
        topics,
        type_markers=("odometry",),
        suffixes=("/odom",),
        contains=("odometry/filtered", "/odometry/"),
    )
    scan_topics = _select_topics(
        topics,
        type_markers=("laserscan",),
        suffixes=("/scan",),
        contains=("/scan_",),
    )
    cmd_vel_topics = _select_topics(
        topics,
        type_markers=("twist", "twiststamped"),
        suffixes=("/cmd_vel", "/cmd_vel_smoothed"),
        contains=("cmd_vel",),
    )
    local_costmap_topics = _select_topics(
        topics,
        suffixes=("/local_costmap", "/local_costmap/costmap"),
        contains=("local_costmap",),
    )
    global_costmap_topics = _select_topics(
        topics,
        suffixes=("/global_costmap", "/global_costmap/costmap"),
        contains=("global_costmap",),
    )
    plan_topics = _select_topics(
        topics,
        suffixes=("/plan", "/global_plan"),
        contains=("planner_server/plan", "/plan", "trajectory"),
    )
    amcl_topics = _select_topics(
        topics,
        type_markers=("posewithcovariancestamped",),
        suffixes=("/amcl_pose",),
        contains=("amcl_pose",),
    )
    map_topics = _select_topics(
        topics,
        type_markers=("occupancygrid",),
        suffixes=("/map",),
        contains=("/map",),
    )
    behavior_tree_topics = _select_topics(
        topics,
        suffixes=("/behavior_tree_log",),
        contains=("behavior_tree_log",),
    )
    navigate_status_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/status",),
    )
    navigate_feedback_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/feedback",),
    )

    nav2_evidence = sum(
        bool(matches)
        for matches in [
            odom_topics,
            scan_topics,
            cmd_vel_topics,
            local_costmap_topics,
            global_costmap_topics,
            plan_topics,
            amcl_topics,
            map_topics,
            behavior_tree_topics,
            navigate_status_topics,
            navigate_feedback_topics,
        ]
    )

    # --- Nav2 detection ---
    if nav2_evidence >= 2 or (scan_topics and odom_topics):
        recs.append("[bold cyan]Nav2 topics detected[/bold cyan]")

        # Odom check
        for name in odom_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 20:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Odometry ({name}) at {rate:.0f}Hz — good for Nav2")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Odometry ({name}) at {rate:.0f}Hz — Nav2 works best at 20Hz+")

        # LaserScan check
        for name in scan_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 10:
                    recs.append(f"  [green]:heavy_check_mark:[/green] LaserScan ({name}) at {rate:.0f}Hz — good for costmap")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] LaserScan ({name}) at {rate:.0f}Hz — 10Hz+ recommended for costmap updates")

        # Costmap publication checks
        for name in local_costmap_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 1:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Local costmap ({name}) at {rate:.1f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Local costmap ({name}) at {rate:.1f}Hz — low refresh for local obstacle updates")
        for name in global_costmap_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 0.2:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Global costmap ({name}) at {rate:.1f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Global costmap ({name}) at {rate:.1f}Hz — planner map refresh is very sparse")

        # cmd_vel check
        for name in cmd_vel_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 10:
                recs.append(f"  [yellow]:warning:[/yellow] cmd_vel at {rate:.0f}Hz — control loop may be too slow")

        if plan_topics:
            for name in plan_topics:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Global plan ({name}) recorded {topics[name]['count']} times — planner output is visible"
                )
        else:
            recs.append(
                "  [yellow]:warning:[/yellow] No global plan topic recorded — planner activity is hard to inspect from this bag"
            )

        if navigate_status_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] NavigateToPose status ({navigate_status_topics[0]}) recorded — goal lifecycle is observable"
            )
        elif navigate_feedback_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] NavigateToPose feedback ({navigate_feedback_topics[0]}) recorded"
            )
        else:
            recs.append(
                "  [yellow]:warning:[/yellow] No NavigateToPose action topic recorded — goal-level debugging is limited"
            )

        # Nav2 pipeline latency
        ts = report._topic_timestamps
        nav2_pipeline = []
        if scan_topics and local_costmap_topics:
            nav2_pipeline.append((scan_topics[0], local_costmap_topics[0], "scan → costmap"))
        if odom_topics and amcl_topics:
            nav2_pipeline.append((odom_topics[0], amcl_topics[0], "odom → localization"))
        if scan_topics and cmd_vel_topics:
            nav2_pipeline.append((scan_topics[0], cmd_vel_topics[0], "scan → cmd_vel (full loop)"))
        _add_pipeline_latency_recs(recs, ts, nav2_pipeline)
        if plan_topics and cmd_vel_topics:
            _add_event_response_latency_recs(
                recs,
                ts,
                [(plan_topics[0], cmd_vel_topics[0], "plan → cmd_vel onset")],
                min_input_samples=1,
                max_response_ms=1000.0,
            )

    # --- Autoware detection ---
    autoware_prefixes = ("/sensing/", "/perception/", "/planning/", "/control/", "/localization/", "/vehicle/")
    autoware_topics = [
        name for name in topics if any(name.startswith(prefix) for prefix in autoware_prefixes)
    ]

    if len(autoware_topics) >= 1:
        recs.append("[bold cyan]Autoware topics detected[/bold cyan]")

        # Camera check
        camera_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
            prefixes=("/sensing/",),
            contains=("/camera/", "image"),
        )
        for name in camera_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 15:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Camera ({name}) at {rate:.0f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Camera ({name}) at {rate:.0f}Hz — 15Hz+ recommended for perception")

        # LiDAR check under /sensing
        lidar_topics = _select_topics(
            topics,
            type_markers=("pointcloud2", "velodynescan"),
            prefixes=("/sensing/",),
            contains=("lidar", "pointcloud", "velodyne_packets"),
        )
        for name in lidar_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                recs.append(f"  [green]:heavy_check_mark:[/green] LiDAR ({name}) at {rate:.0f}Hz")

        # GNSS under /sensing
        gnss_topics = _select_topics(
            topics,
            type_markers=("navsatfix",),
            prefixes=("/sensing/",),
            contains=("gnss",),
        )
        for name in gnss_topics:
            if "navsatfix" in topic_types[name].lower():
                recs.append(f"  [green]:heavy_check_mark:[/green] GNSS ({name})")

        vehicle_status_topics = _select_topics(
            topics,
            prefixes=("/vehicle/status/",),
            contains=("velocity_status", "steering_status", "gear_status"),
        ) or _select_topics(topics, prefixes=("/vehicle/status/",))
        for name in vehicle_status_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Vehicle status ({name}) at {rate:.0f}Hz — control/vehicle telemetry recorded"
                )

        # Autoware pipeline latency
        ts = report._topic_timestamps
        # Find actual topic names matching patterns
        sensing_lidar = lidar_topics
        perception = _select_topics(topics, prefixes=("/perception/",))
        planning = _select_topics(
            topics,
            prefixes=("/planning/",),
            contains=("trajectory", "path"),
        ) or _select_topics(topics, prefixes=("/planning/",))
        control = _select_topics(
            topics,
            prefixes=("/control/",),
            contains=("control_cmd", "command", "trajectory_follower"),
        ) or _select_topics(topics, prefixes=("/control/",))

        for name in planning:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                recs.append(f"  [green]:heavy_check_mark:[/green] Planning output ({name}) at {rate:.0f}Hz")

        for name in control:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 10:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Control command ({name}) at {rate:.0f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Control command ({name}) at {rate:.0f}Hz — low for closed-loop vehicle control")

        if not planning and not control:
            recs.append(
                "  [dim]:information_source:[/dim] Sensing/localization-only Autoware bag — skipping planning/control checks"
            )
        elif not planning:
            recs.append(
                "  [yellow]:warning:[/yellow] Control topics are present but no planning topic was recorded"
            )
        elif not control:
            recs.append(
                "  [yellow]:warning:[/yellow] Planning topics are present but no control topic was recorded"
            )

        aw_pipeline = []
        if sensing_lidar and perception:
            aw_pipeline.append((sensing_lidar[0], perception[0], "sensing → perception"))
        if perception and planning:
            aw_pipeline.append((perception[0], planning[0], "perception → planning"))
        if planning and control:
            aw_pipeline.append((planning[0], control[0], "planning → control"))
        if sensing_lidar and control:
            aw_pipeline.append((sensing_lidar[0], control[0], "end-to-end (sensing → control)"))
        _add_pipeline_latency_recs(recs, ts, aw_pipeline)

    # --- MoveIt detection ---
    joint_state_topics = _select_topics(
        topics,
        type_markers=("jointstate",),
        suffixes=("/joint_states",),
        contains=("joint_states",),
    )
    planned_path_topics = _select_topics(
        topics,
        type_markers=("displaytrajectory",),
        suffixes=("/display_planned_path",),
        contains=("display_planned_path", "planned_path"),
    )
    planning_scene_topics = _select_topics(
        topics,
        type_markers=("planningscene",),
        suffixes=("/planning_scene", "/monitored_planning_scene"),
        contains=("planning_scene", "monitored_planning_scene"),
    )
    move_group_result_topics = _select_topics(
        topics,
        suffixes=("/move_group/result",),
        contains=("move_group/result",),
    )
    move_action_status_topics = _select_topics(
        topics,
        contains=("move_action/_action/status",),
    )
    move_action_feedback_topics = _select_topics(
        topics,
        contains=("move_action/_action/feedback",),
    )
    execute_action_status_topics = _select_topics(
        topics,
        contains=("execute_trajectory/_action/status",),
    )
    execute_action_feedback_topics = _select_topics(
        topics,
        contains=("execute_trajectory/_action/feedback",),
    )
    controller_action_status_topics = _select_topics(
        topics,
        contains=("follow_joint_trajectory/_action/status",),
    )
    controller_action_feedback_topics = _select_topics(
        topics,
        contains=("follow_joint_trajectory/_action/feedback",),
    )

    moveit_evidence = sum(
        bool(matches)
        for matches in [
            joint_state_topics,
            planned_path_topics,
            planning_scene_topics,
            move_group_result_topics,
            move_action_status_topics,
            move_action_feedback_topics,
            execute_action_status_topics,
            execute_action_feedback_topics,
            controller_action_status_topics,
            controller_action_feedback_topics,
        ]
    )

    if moveit_evidence >= 2 or (
        joint_state_topics and (
            planned_path_topics
            or planning_scene_topics
            or move_group_result_topics
            or move_action_status_topics
            or execute_action_status_topics
            or controller_action_status_topics
        )
    ):
        recs.append("[bold cyan]MoveIt topics detected[/bold cyan]")

        for name in joint_state_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 100_000:  # sanity check for broken timestamps
                if rate >= 100:
                    recs.append(f"  [green]:heavy_check_mark:[/green] JointState ({name}) at {rate:.0f}Hz — good for motion planning")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] JointState ({name}) at {rate:.0f}Hz — 100Hz+ recommended for smooth trajectories")

        if move_action_status_topics or move_action_feedback_topics:
            name = (move_action_status_topics or move_action_feedback_topics)[0]
            recs.append(
                f"  [green]:heavy_check_mark:[/green] MoveGroup action activity recorded on {name}"
            )

        if execute_action_status_topics or execute_action_feedback_topics:
            name = (execute_action_status_topics or execute_action_feedback_topics)[0]
            recs.append(
                f"  [green]:heavy_check_mark:[/green] execute_trajectory action activity recorded on {name}"
            )

        if controller_action_status_topics or controller_action_feedback_topics:
            name = (controller_action_status_topics or controller_action_feedback_topics)[0]
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Joint trajectory controller activity recorded on {name}"
            )
        elif planned_path_topics:
            recs.append(
                "  [yellow]:warning:[/yellow] Planned path is present but trajectory execution was not recorded"
            )

        # MoveIt pipeline latency
        ts = report._topic_timestamps
        moveit_pipeline = []
        if joint_state_topics and planned_path_topics:
            moveit_pipeline.append((joint_state_topics[0], planned_path_topics[0], "joint_states → planned_path"))
        _add_pipeline_latency_recs(recs, ts, moveit_pipeline, min_output_samples=1)
        if planned_path_topics and controller_action_status_topics:
            _add_event_response_latency_recs(
                recs,
                ts,
                [(planned_path_topics[0], controller_action_status_topics[0], "planned_path → arm execution")],
                min_input_samples=1,
                max_response_ms=5000.0,
            )
        elif planned_path_topics and execute_action_status_topics:
            _add_event_response_latency_recs(
                recs,
                ts,
                [(planned_path_topics[0], execute_action_status_topics[0], "planned_path → execute_trajectory")],
                min_input_samples=1,
                max_response_ms=5000.0,
            )

    robot_arm_evidence = bool(joint_state_topics and (color_image_topics or depth_image_topics))
    if robot_arm_evidence and "MoveIt" not in domains:
        recs.append("[bold cyan]Robot arm perception/manipulation topics detected[/bold cyan]")

        for name in joint_state_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 100_000:
                if rate >= 100:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] JointState ({name}) at {rate:.0f}Hz — good for manipulation state tracking"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] JointState ({name}) at {rate:.0f}Hz — 100Hz+ recommended for responsive arm control"
                    )

        for name in color_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] RGB image ({name}) at {rate_label}Hz — good for manipulation perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] RGB image ({name}) at {rate_label}Hz — 15Hz+ recommended for manipulation perception"
                    )

        for name in depth_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Depth image ({name}) at {rate_label}Hz — good for RGB-D grasp perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Depth image ({name}) at {rate_label}Hz — 15Hz+ recommended for RGB-D grasp perception"
                    )

        if color_image_topics and depth_image_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] RGB and depth streams are both recorded — suitable for open-loop manipulation perception benchmarking"
            )
        if color_info_topics or depth_info_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] Camera calibration topics are recorded — exported perception data is reusable"
            )

    perception_evidence = bool(camera_info_topics and (color_image_topics or depth_image_topics or infra_image_topics))
    if perception_evidence and not ({"Autoware", "MoveIt", "RobotArm"} & domains):
        recs.append("[bold cyan]Perception topics detected[/bold cyan]")

        for name in color_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] RGB image ({name}) at {rate_label}Hz — good for camera-based perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] RGB image ({name}) at {rate_label}Hz — 15Hz+ recommended for camera-based perception"
                    )

        for name in depth_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Depth image ({name}) at {rate_label}Hz — good for RGB-D perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Depth image ({name}) at {rate_label}Hz — 15Hz+ recommended for RGB-D perception"
                    )

        for name in infra_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Infra image ({name}) at {rate_label}Hz — good for depth/perception debugging"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Infra image ({name}) at {rate_label}Hz — 15Hz+ recommended for stereo/depth debugging"
                    )

        if color_image_topics and depth_image_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] RGB and depth streams are both recorded — suitable for RGB-D perception benchmarking"
            )
        if len(infra_image_topics) >= 2:
            recs.append(
                "  [green]:heavy_check_mark:[/green] Infra stereo streams are both recorded — depth debugging is possible"
            )
        if color_info_topics or depth_info_topics or infra_info_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] Camera calibration topics are recorded — exported perception data is reusable"
            )

    return recs


def _detect_domain_names(topics: dict[str, dict]) -> set[str]:
    """Return high-level framework names inferred from topic names/types."""
    if not topics:
        return set()

    names: set[str] = set()

    odom_topics = _select_topics(
        topics,
        type_markers=("odometry",),
        suffixes=("/odom",),
        contains=("odometry/filtered", "/odometry/"),
    )
    scan_topics = _select_topics(
        topics,
        type_markers=("laserscan",),
        suffixes=("/scan",),
        contains=("/scan_",),
    )
    cmd_vel_topics = _select_topics(
        topics,
        type_markers=("twist", "twiststamped"),
        suffixes=("/cmd_vel", "/cmd_vel_smoothed"),
        contains=("cmd_vel",),
    )
    local_costmap_topics = _select_topics(
        topics,
        suffixes=("/local_costmap", "/local_costmap/costmap"),
        contains=("local_costmap",),
    )
    global_costmap_topics = _select_topics(
        topics,
        suffixes=("/global_costmap", "/global_costmap/costmap"),
        contains=("global_costmap",),
    )
    plan_topics = _select_topics(
        topics,
        suffixes=("/plan", "/global_plan"),
        contains=("planner_server/plan", "/plan", "trajectory"),
    )
    amcl_topics = _select_topics(
        topics,
        type_markers=("posewithcovariancestamped",),
        suffixes=("/amcl_pose",),
        contains=("amcl_pose",),
    )
    navigate_status_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/status",),
    )
    navigate_feedback_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/feedback",),
    )
    nav2_evidence = sum(
        bool(matches)
        for matches in [
            odom_topics,
            scan_topics,
            cmd_vel_topics,
            local_costmap_topics,
            global_costmap_topics,
            plan_topics,
            amcl_topics,
            navigate_status_topics,
            navigate_feedback_topics,
        ]
    )
    if nav2_evidence >= 2 or (scan_topics and odom_topics):
        names.add("Nav2")

    autoware_prefixes = ("/sensing/", "/perception/", "/planning/", "/control/", "/localization/", "/vehicle/")
    autoware_topics = [
        name for name in topics if any(name.startswith(prefix) for prefix in autoware_prefixes)
    ]
    if autoware_topics:
        names.add("Autoware")

    joint_state_topics = _select_topics(
        topics,
        type_markers=("jointstate",),
        suffixes=("/joint_states",),
        contains=("joint_states",),
    )
    planned_path_topics = _select_topics(
        topics,
        type_markers=("displaytrajectory",),
        suffixes=("/display_planned_path",),
        contains=("display_planned_path", "planned_path"),
    )
    planning_scene_topics = _select_topics(
        topics,
        type_markers=("planningscene",),
        suffixes=("/planning_scene", "/monitored_planning_scene"),
        contains=("planning_scene", "monitored_planning_scene"),
    )
    move_action_status_topics = _select_topics(
        topics,
        contains=("move_action/_action/status",),
    )
    execute_action_status_topics = _select_topics(
        topics,
        contains=("execute_trajectory/_action/status",),
    )
    controller_action_status_topics = _select_topics(
        topics,
        contains=("follow_joint_trajectory/_action/status",),
    )
    moveit_evidence = sum(
        bool(matches)
        for matches in [
            joint_state_topics,
            planned_path_topics,
            planning_scene_topics,
            move_action_status_topics,
            execute_action_status_topics,
            controller_action_status_topics,
        ]
    )
    if moveit_evidence >= 2 or (joint_state_topics and planned_path_topics):
        names.add("MoveIt")

    image_topics = _select_topics(
        topics,
        type_markers=("image", "compressedimage"),
    )
    color_image_topics = [
        name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
    ]
    depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
    infra_image_topics = [name for name in image_topics if "infra" in name.lower()]
    if joint_state_topics and (color_image_topics or depth_image_topics):
        names.add("RobotArm")

    camera_info_topics = _select_topics(
        topics,
        type_markers=("camerainfo",),
        contains=("camera_info",),
    )
    if (
        camera_info_topics
        and (color_image_topics or depth_image_topics or infra_image_topics)
        and "RobotArm" not in names
        and "Autoware" not in names
    ):
        names.add("Perception")

    return names


def _compute_domain_score(report: EvalReport) -> float | None:
    """Compute framework-aware score when generic SLAM metrics are not representative."""
    topics = report.topic_info
    if not topics:
        return None

    scores: list[float] = []
    domains = _detect_domain_names(topics)

    if "Nav2" in domains:
        odom_topics = _select_topics(
            topics,
            type_markers=("odometry",),
            suffixes=("/odom",),
            contains=("odometry/filtered", "/odometry/"),
        )
        scan_topics = _select_topics(
            topics,
            type_markers=("laserscan",),
            suffixes=("/scan",),
            contains=("/scan_",),
        )
        local_costmap_topics = _select_topics(
            topics,
            suffixes=("/local_costmap", "/local_costmap/costmap"),
            contains=("local_costmap",),
        )
        global_costmap_topics = _select_topics(
            topics,
            suffixes=("/global_costmap", "/global_costmap/costmap"),
            contains=("global_costmap",),
        )
        plan_topics = _select_topics(
            topics,
            suffixes=("/plan", "/global_plan"),
            contains=("planner_server/plan", "/plan", "trajectory"),
        )
        navigate_topics = _select_topics(
            topics,
            contains=("navigate_to_pose/_action/status", "navigate_to_pose/_action/feedback"),
        )

        nav2_scores = []
        if odom_topics:
            nav2_scores.append(_rate_goal_score(topics[odom_topics[0]]["rate_hz"], 20.0))
        if scan_topics:
            nav2_scores.append(_rate_goal_score(topics[scan_topics[0]]["rate_hz"], 10.0))
        if local_costmap_topics:
            nav2_scores.append(_rate_goal_score(topics[local_costmap_topics[0]]["rate_hz"], 1.0))
        if global_costmap_topics:
            nav2_scores.append(_rate_goal_score(topics[global_costmap_topics[0]]["rate_hz"], 0.2))
        nav2_scores.append(100.0 if plan_topics else 0.0)
        nav2_scores.append(100.0 if navigate_topics else 0.0)
        scores.append(float(np.mean(nav2_scores)))

    if "Autoware" in domains:
        camera_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
            prefixes=("/sensing/",),
            contains=("/camera/", "image"),
        )
        lidar_topics = _select_topics(
            topics,
            type_markers=("pointcloud2", "velodynescan"),
            prefixes=("/sensing/",),
            contains=("lidar", "pointcloud", "velodyne_packets"),
        )
        gnss_topics = _select_topics(
            topics,
            type_markers=("navsatfix",),
            prefixes=("/sensing/",),
            contains=("gnss",),
        )
        vehicle_status_topics = _select_topics(
            topics,
            prefixes=("/vehicle/status/",),
            contains=("velocity_status", "steering_status", "gear_status"),
        ) or _select_topics(topics, prefixes=("/vehicle/status/",))
        planning_topics = _select_topics(
            topics,
            prefixes=("/planning/",),
            contains=("trajectory", "path"),
        ) or _select_topics(topics, prefixes=("/planning/",))
        control_topics = _select_topics(
            topics,
            prefixes=("/control/",),
            contains=("control_cmd", "command", "trajectory_follower"),
        ) or _select_topics(topics, prefixes=("/control/",))

        autoware_scores = []
        autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in camera_topics)
        autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 10.0) for name in lidar_topics)
        autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 20.0) for name in vehicle_status_topics)
        if gnss_topics:
            autoware_scores.append(100.0)
        if planning_topics or control_topics:
            autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 5.0) for name in planning_topics)
            autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 10.0) for name in control_topics)
            if planning_topics and not control_topics:
                autoware_scores.append(0.0)
            if control_topics and not planning_topics:
                autoware_scores.append(0.0)
        if autoware_scores:
            scores.append(float(np.mean(autoware_scores)))

    if "RobotArm" in domains:
        joint_state_topics = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        image_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
        )
        color_image_topics = [
            name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
        ]
        depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
        camera_info_topics = _select_topics(
            topics,
            type_markers=("camerainfo",),
            contains=("camera_info",),
        )

        robot_arm_scores = []
        if joint_state_topics:
            robot_arm_scores.append(_rate_goal_score(topics[joint_state_topics[0]]["rate_hz"], 100.0))
        robot_arm_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in color_image_topics)
        robot_arm_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in depth_image_topics)
        if color_image_topics and depth_image_topics:
            robot_arm_scores.append(100.0)
        if camera_info_topics:
            robot_arm_scores.append(100.0)
        if robot_arm_scores:
            scores.append(float(np.mean(robot_arm_scores)))

    if "Perception" in domains:
        image_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
        )
        color_image_topics = [
            name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
        ]
        depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
        infra_image_topics = [name for name in image_topics if "infra" in name.lower()]
        camera_info_topics = _select_topics(
            topics,
            type_markers=("camerainfo",),
            contains=("camera_info",),
        )

        perception_scores = []
        perception_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in color_image_topics)
        perception_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in depth_image_topics)
        perception_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in infra_image_topics)
        if color_image_topics and depth_image_topics:
            perception_scores.append(100.0)
        if len(infra_image_topics) >= 2:
            perception_scores.append(100.0)
        if camera_info_topics:
            perception_scores.append(100.0)
        if perception_scores:
            scores.append(float(np.mean(perception_scores)))

    if "MoveIt" in domains:
        joint_state_topics = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        planned_path_topics = _select_topics(
            topics,
            type_markers=("displaytrajectory",),
            suffixes=("/display_planned_path",),
            contains=("display_planned_path", "planned_path"),
        )
        move_action_topics = _select_topics(
            topics,
            contains=("move_action/_action/status", "move_action/_action/feedback"),
        )
        execute_action_topics = _select_topics(
            topics,
            contains=("execute_trajectory/_action/status", "execute_trajectory/_action/feedback"),
        )
        controller_action_topics = _select_topics(
            topics,
            contains=("follow_joint_trajectory/_action/status", "follow_joint_trajectory/_action/feedback"),
        )

        moveit_scores = []
        if joint_state_topics:
            moveit_scores.append(_rate_goal_score(topics[joint_state_topics[0]]["rate_hz"], 100.0))
        moveit_scores.append(100.0 if planned_path_topics else 0.0)
        execution_observable = bool(controller_action_topics or execute_action_topics)
        if planned_path_topics:
            moveit_scores.append(100.0 if execution_observable else 0.0)
        if move_action_topics:
            moveit_scores.append(100.0)
        if controller_action_topics:
            moveit_scores.append(100.0)
        elif execute_action_topics:
            moveit_scores.append(85.0)
        scores.append(float(np.mean(moveit_scores)))

    return max(scores) if scores else None


def _rate_goal_score(rate_hz: float, target_hz: float) -> float:
    """Normalize a topic rate against a practical target."""
    if rate_hz <= 0 or target_hz <= 0:
        return 0.0
    return float(max(0.0, min(100.0, (rate_hz / target_hz) * 100.0)))


def _format_rate_hz(rate_hz: float) -> str:
    """Format a rate without hiding threshold-relevant decimals."""
    if rate_hz < 20 and abs(rate_hz - round(rate_hz)) >= 0.05:
        return f"{rate_hz:.1f}"
    return f"{rate_hz:.0f}"


def _select_topics(
    topics: dict[str, dict],
    *,
    type_markers: tuple[str, ...] = (),
    prefixes: tuple[str, ...] = (),
    suffixes: tuple[str, ...] = (),
    contains: tuple[str, ...] = (),
) -> list[str]:
    """Return topic names matching type/name heuristics, sorted by rate desc."""
    matches = []
    normalized_type_markers = tuple(marker.lower() for marker in type_markers)
    normalized_prefixes = tuple(prefix.lower() for prefix in prefixes)
    normalized_suffixes = tuple(suffix.lower() for suffix in suffixes)
    normalized_contains = tuple(fragment.lower() for fragment in contains)

    for name, info in topics.items():
        if int(info.get("count", 0) or 0) <= 0:
            continue
        lower_name = name.lower()
        lower_type = str(info.get("type", "")).lower()

        if normalized_type_markers and not any(marker in lower_type for marker in normalized_type_markers):
            continue

        has_name_filters = bool(normalized_prefixes or normalized_suffixes or normalized_contains)
        if has_name_filters:
            name_match = (
                any(lower_name.startswith(prefix) for prefix in normalized_prefixes)
                or any(lower_name == suffix or lower_name.endswith(suffix) for suffix in normalized_suffixes)
                or any(fragment in lower_name for fragment in normalized_contains)
            )
            if not name_match:
                continue

        matches.append(name)

    return sorted(matches, key=lambda name: (-topics[name]["rate_hz"], name))


def _add_pipeline_latency_recs(
    recs: list[str],
    topic_timestamps: dict[str, list[int]],
    pipeline: list[tuple[str, str, str]],
    *,
    min_output_samples: int = 5,
) -> None:
    """Measure and report pipeline latency between topic pairs.

    For each (input_topic, output_topic, label), computes the median delay
    from input to the next output message, which estimates processing latency.
    """
    for input_topic, output_topic, label in pipeline:
        ts_in = topic_timestamps.get(input_topic, [])
        ts_out = topic_timestamps.get(output_topic, [])
        if len(ts_in) < max(min_output_samples, 1) or len(ts_out) < min_output_samples:
            continue

        arr_in = np.array(sorted(ts_in), dtype=np.int64)
        arr_out = np.array(sorted(ts_out), dtype=np.int64)

        # For each output message, find the most recent input BEFORE it
        # using binary search (j must NOT carry over between iterations)
        latencies_ms = []
        for t_out in arr_out:
            # Binary search: find the rightmost input <= t_out
            idx = int(np.searchsorted(arr_in, t_out, side="right")) - 1
            if 0 <= idx < len(arr_in):
                latency = (t_out - arr_in[idx]) / 1e6  # ns to ms
                # Only count if the input is reasonably recent (< 2x output interval)
                if latency >= 0:
                    latencies_ms.append(latency)

        if not latencies_ms:
            continue

        # If the stages run at a similar period, also evaluate index-aligned
        # pairing. This avoids under-reporting when true latency exceeds one
        # period and "latest input before output" starts matching the next frame.
        if len(arr_in) >= 6 and len(arr_out) >= 6:
            input_period_ms = float(np.median(np.diff(arr_in)) / 1e6)
            output_period_ms = float(np.median(np.diff(arr_out)) / 1e6)
            if input_period_ms > 0 and output_period_ms > 0:
                period_ratio = max(input_period_ms, output_period_ms) / min(input_period_ms, output_period_ms)
                if period_ratio <= 1.25:
                    pair_count = min(len(arr_in), len(arr_out))
                    aligned = ((arr_out[:pair_count] - arr_in[:pair_count]) / 1e6).astype(float)
                    aligned = aligned[aligned >= 0]
                    if len(aligned) >= 5:
                        aligned_median = float(np.median(aligned))
                        predecessor_median = float(np.median(latencies_ms))
                        if aligned_median > predecessor_median + max(input_period_ms * 0.25, 1.0):
                            latencies_ms = aligned.tolist()

        median_lat = float(np.median(latencies_ms))
        p95_lat = float(np.percentile(latencies_ms, 95))
        sample_note = ""
        if len(latencies_ms) < 5:
            plural = "s" if len(latencies_ms) != 1 else ""
            sample_note = f" ({len(latencies_ms)} sample{plural})"

        if median_lat < 50:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note}"
            )
        elif median_lat < 200:
            recs.append(
                f"  [yellow]:warning:[/yellow] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — may affect real-time performance"
            )
        else:
            recs.append(
                f"  [red]:x:[/red] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — too slow for real-time"
            )


def _add_event_response_latency_recs(
    recs: list[str],
    topic_timestamps: dict[str, list[int]],
    pairs: list[tuple[str, str, str]],
    *,
    min_input_samples: int = 1,
    max_response_ms: float = 2000.0,
) -> None:
    """Measure sparse event-to-next-response latency."""
    for input_topic, output_topic, label in pairs:
        ts_in = topic_timestamps.get(input_topic, [])
        ts_out = topic_timestamps.get(output_topic, [])
        if len(ts_in) < min_input_samples or not ts_out:
            continue

        arr_in = np.array(sorted(ts_in), dtype=np.int64)
        arr_out = np.array(sorted(ts_out), dtype=np.int64)
        latencies_ms: list[float] = []
        for t_in in arr_in:
            idx = int(np.searchsorted(arr_out, t_in, side="right"))
            if idx >= len(arr_out):
                continue
            latency_ms = float((arr_out[idx] - t_in) / 1e6)
            if 0 <= latency_ms <= max_response_ms:
                latencies_ms.append(latency_ms)

        if len(latencies_ms) < min_input_samples:
            continue

        median_lat = float(np.median(latencies_ms))
        p95_lat = float(np.percentile(latencies_ms, 95))
        sample_note = ""
        if len(latencies_ms) < 5:
            plural = "s" if len(latencies_ms) != 1 else ""
            sample_note = f" ({len(latencies_ms)} sample{plural})"

        if median_lat < 50:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note}"
            )
        elif median_lat < 200:
            recs.append(
                f"  [yellow]:warning:[/yellow] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — may affect responsiveness"
            )
        else:
            recs.append(
                f"  [red]:x:[/red] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — too slow for interactive use"
            )
