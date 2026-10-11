"""Single bag quality evaluation engine.

Evaluates GNSS, IMU, and inter-topic sync quality,
producing a structured quality report with a composite score.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import asdict, dataclass, field
from typing import Any, TextIO

import numpy as np
from rich.console import Console
from rich.markup import escape
from rich.table import Table

from bagx.custom_rules import CustomDomainResult, evaluate_custom_rule_set, load_custom_rule_set
from bagx.contracts import report_metadata
from bagx.eval_domains import (
    _compute_domain_score,
    _detect_domain_recommendations,
    detect_domain_names,
)
from bagx.eval_findings import generate_findings
# Re-exported for plugins and callers that import these from bagx.eval.
from bagx.eval_topics import _has_control_signature as _has_control_signature
from bagx.eval_topics import _has_nav2_signature as _has_nav2_signature
from bagx.eval_topics import (
    _select_action_result_topics,
    _select_action_status_topics,
    _select_service_event_topics,
)
from bagx.eval_topics import _select_topics as _select_topics
from bagx.eval_workflow import (
    WorkflowMetrics,
    _generate_workflow_recommendations,
    _summarize_workflow_metrics,
)
from bagx.findings import Finding, severity_at_least
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
    custom_domains: list[CustomDomainResult] = field(default_factory=list)
    workflow_metrics: list[WorkflowMetrics] = field(default_factory=list)
    findings: list[Finding] = field(default_factory=list)
    tf: dict | None = None  # TF tree summary (frames, roots, edges); None without /tf
    _topic_timestamps: dict = field(default_factory=dict, repr=False)  # internal, not serialized

    def to_dict(self) -> dict:
        if not self.findings:
            self.findings = generate_findings(
                self,
                detect_domain_names(self.topic_info, self.custom_domains),
            )
        d = asdict(self)
        # Remove internal fields
        d.pop("_topic_timestamps", None)
        # Clean up NaN for JSON
        d = _clean_nan(d)
        for custom_domain in d.get("custom_domains", []):
            custom_domain["recommendations"] = [
                _strip_rich_markup(item) for item in custom_domain.get("recommendations", [])
            ]
        # Add recommendations
        d["recommendations"] = [
            _strip_rich_markup(r) for r in generate_recommendations(self)
        ]
        d["findings"] = [finding.to_dict() for finding in self.findings]
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
    custom_rules_path: str | None = None,
    include_anomaly: bool = False,
) -> EvalReport:
    """Run full quality evaluation on a bag file.

    When ``include_anomaly`` is True, anomaly detection runs alongside the
    quality eval and its temporal findings (fix-lost segments, IMU spike
    clusters, rate gaps) are merged into ``report.findings``. This is the
    surface CI gates with ``bagx diff`` and ``bagx benchmark`` use to reason
    about segment-level readiness.
    """
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
    summary_topic_info = {
        name: {"type": info.type, "count": info.count, "rate_hz": 0.0}
        for name, info in summary.topics.items()
    }
    workflow_candidate_topics = set(_select_action_status_topics(summary_topic_info))
    workflow_candidate_topics.update(_select_action_result_topics(summary_topic_info))
    workflow_candidate_topics.update(_select_service_event_topics(summary_topic_info))

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
    workflow_messages: dict[str, list[dict[str, Any]]] = {
        topic_name: [] for topic_name in workflow_candidate_topics
    }

    from bagx.msg_checks import MessageCheckAccumulator
    from bagx.tf_tree import TfAccumulator

    topic_types = {name: info.type for name, info in summary.topics.items()}
    tf_accumulator = TfAccumulator(topic_types)
    msg_accumulator = MessageCheckAccumulator(topic_types)

    # Read all messages if we need sync analysis
    for msg in reader.read_messages(topics=None):
        tf_accumulator.add(msg)
        msg_accumulator.add(msg)
        if msg.topic in topic_timestamps:
            topic_timestamps[msg.topic].append(msg.timestamp_ns)
        if msg.topic in workflow_messages:
            workflow_messages[msg.topic].append(msg.data)
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

    custom_rule_set = load_custom_rule_set(custom_rules_path) if custom_rules_path else None
    report.custom_domains = evaluate_custom_rule_set(
        report.topic_info,
        report._topic_timestamps,
        custom_rule_set,
    )
    report.workflow_metrics = _summarize_workflow_metrics(workflow_messages, report.topic_info)

    domains = detect_domain_names(report.topic_info, report.custom_domains)

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
    report.findings = generate_findings(report, domains)

    tf_report = tf_accumulator.build(str(bag_path), bag_end_ns=summary.end_time_ns or None)
    report.findings.extend(tf_report.findings)
    report.findings.extend(msg_accumulator.build())
    if tf_report.edges:
        tf_summary = tf_report.to_dict()
        tf_summary.pop("findings", None)
        tf_summary.pop("bag_path", None)
        report.tf = tf_summary

    if include_anomaly:
        # Local import avoids a circular dependency: anomaly imports from eval
        # contracts that aren't ready at module load time.
        from bagx.anomaly import detect_anomalies
        try:
            anomaly_report = detect_anomalies(bag_path)
            report.findings.extend(anomaly_report.findings)
        except Exception as exc:  # pragma: no cover - defensive
            logger.warning("Anomaly detection failed during eval: %s", exc)

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


def print_eval_report(
    report: EvalReport,
    console: Console | None = None,
    *,
    findings_only: bool = False,
    severity_min: str | None = None,
) -> None:
    """Pretty-print an evaluation report.

    findings_only: when True, suppress sensor tables and recommendations and
    render the structured findings list instead.
    severity_min: when set, filter findings to this severity or higher (only
    affects the findings_only rendering).
    """
    if console is None:
        console = Console()

    if findings_only:
        _print_findings_only(report, console, severity_min=severity_min)
        return

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

    workflow_rows = [metric for metric in report.workflow_metrics if metric.observed_events > 0]
    if workflow_rows:
        table = Table(title="Workflow Outcomes", show_header=True)
        table.add_column("Topic", style="bold")
        table.add_column("Kind")
        table.add_column("Observed", justify="right")
        table.add_column("Success", justify="right")
        table.add_column("Failure", justify="right")
        table.add_column("Notes")
        for metric in workflow_rows:
            note_parts: list[str] = []
            if metric.kind == "service_event" and metric.request_events > 0:
                note_parts.append(f"responses {metric.response_events}/{metric.request_events}")
            elif metric.completed_events > 0:
                note_parts.append(f"completed {metric.completed_events}")
            if metric.failure_reasons:
                note_parts.append(f"failures: {', '.join(metric.failure_reasons)}")
            table.add_row(
                metric.topic,
                metric.kind.replace("_", " "),
                str(metric.observed_events),
                str(metric.success_events),
                str(metric.failure_events),
                "; ".join(note_parts) or "-",
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


_SEVERITY_STYLE = {
    "info": "green",
    "warning": "yellow",
    "error": "red",
    "critical": "bold red",
}


def _print_findings_only(
    report: EvalReport,
    console: Console,
    *,
    severity_min: str | None = None,
) -> None:
    """Render the structured findings list as text, optionally severity-filtered."""
    from pathlib import Path as _Path

    findings = report.findings or generate_findings(
        report, detect_domain_names(report.topic_info, report.custom_domains)
    )
    if severity_min:
        findings = [f for f in findings if severity_at_least(f.severity, severity_min)]

    bag_name = _Path(report.bag_path).name or _Path(report.bag_path).stem
    console.print(f"\n[bold]Findings: [cyan]{bag_name}[/cyan][/bold]")

    if not findings:
        suffix = f" (severity >= {severity_min})" if severity_min else ""
        console.print(f"[dim]No findings{suffix}.[/dim]\n")
        return

    findings_sorted = sorted(
        findings,
        key=lambda f: (-_severity_rank(f.severity), f.category, f.id),
    )
    for finding in findings_sorted:
        style = _SEVERITY_STYLE.get(finding.severity, "white")
        console.print(
            f"  [{style}]{finding.severity.upper():<8}[/{style}] "
            f"[bold]{finding.id}[/bold] — {escape(finding.title)}"
        )
        if finding.affected_topics:
            console.print(f"    [dim]topics:[/dim] {', '.join(finding.affected_topics)}")
        for ev in finding.evidence:
            unit = f" {ev.unit}" if ev.unit else ""
            expected = f" (expected {ev.expected})" if ev.expected is not None else ""
            console.print(f"    [dim]{ev.metric}:[/dim] {ev.observed}{unit}{expected}")
        if finding.suggested_action:
            console.print(f"    [dim]action:[/dim] {finding.suggested_action}")
    console.print()


def _severity_rank(severity: str) -> int:
    from bagx.findings import SEVERITY_ORDER
    return SEVERITY_ORDER.get(severity, -1)


def _strip_rich_markup(text: str) -> str:
    """Remove rich markup tags for plain text output (JSON, etc.)."""
    import re
    return re.sub(r"\[/?[^\]]*\]", "", text).replace(":heavy_check_mark:", "OK").replace(":warning:", "WARNING").replace(":x:", "ERROR").replace(":information_source:", "INFO")


def generate_recommendations(report: EvalReport) -> list[str]:
    """Generate actionable recommendations based on eval results."""
    recs: list[str] = []

    # Detect non-SLAM domains first to suppress irrelevant SLAM advice
    domain_names = detect_domain_names(report.topic_info, report.custom_domains)
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
    recs.extend(_generate_workflow_recommendations(report.workflow_metrics))
    for domain in report.custom_domains:
        recs.extend(domain.recommendations)

    # --- TF tree and message content (findings-only checks, no score impact) ---
    prefixes = {"tf_tree": "TF", "message_content": "Data"}
    for finding in report.findings:
        prefix = prefixes.get(finding.category)
        if prefix is None or finding.severity not in ("warning", "error", "critical"):
            continue
        marker = (
            "[yellow]:warning:[/yellow]" if finding.severity == "warning"
            else "[red]:x:[/red]"
        )
        recs.append(f"{marker} {prefix}: {escape(finding.title)}")

    return recs


