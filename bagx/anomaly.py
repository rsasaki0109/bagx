"""Anomaly detection for sensor data in rosbag files.

Detects outliers and anomalies in GNSS, IMU, and general message rate data.
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
from bagx.topic_filters import is_rate_anomaly_candidate

logger = logging.getLogger(__name__)


RATE_ANOMALY_WARMUP_MIN_DURATION_SEC = 15.0
RATE_ANOMALY_WARMUP_SEC = 5.0


@dataclass
class AnomalyEvent:
    timestamp_ns: int
    topic: str
    type: str  # e.g. "gnss_jump", "imu_accel_spike", "rate_gap"
    severity: str  # "low", "medium", "high"
    description: str

    @property
    def timestamp_sec(self) -> float:
        return self.timestamp_ns / 1e9


@dataclass
class AnomalyReport:
    bag_path: str
    total_anomalies: int
    anomalies: list[AnomalyEvent] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "bag_path": self.bag_path,
            "total_anomalies": self.total_anomalies,
            "anomalies": [asdict(a) for a in self.anomalies],
        }


def detect_anomalies(
    bag_path: str,
    topics: list[str] | None = None,
    output_json: TextIO | None = None,
) -> AnomalyReport:
    """Run anomaly detection on a bag file.

    Args:
        bag_path: Path to the bag file.
        topics: Optional list of topics to analyze. If None, all topics are analyzed.
        output_json: Optional file handle for JSON output.

    Returns:
        AnomalyReport with detected anomaly events.
    """
    reader = BagReader(bag_path)
    summary = reader.summary()
    warmup_cutoff_ns = _rate_anomaly_warmup_cutoff_ns(summary.start_time_ns, summary.duration_sec)

    # Classify topics
    gnss_topics: list[str] = []
    imu_topics: list[str] = []
    all_topics: list[str] = []

    for name, info in summary.topics.items():
        if topics and name not in topics:
            continue
        all_topics.append(name)
        if "NavSatFix" in info.type or "navsatfix" in info.type.lower():
            gnss_topics.append(name)
        elif "Imu" in info.type or "imu" in info.type.lower():
            imu_topics.append(name)

    # Collect messages
    gnss_messages: dict[str, list[Message]] = {t: [] for t in gnss_topics}
    imu_messages: dict[str, list[Message]] = {t: [] for t in imu_topics}
    topic_timestamps: dict[str, list[int]] = {t: [] for t in all_topics}

    for msg in reader.read_messages(topics=topics):
        if msg.topic in topic_timestamps:
            topic_timestamps[msg.topic].append(msg.timestamp_ns)
        if msg.topic in gnss_topics:
            gnss_messages[msg.topic].append(msg)
        elif msg.topic in imu_topics:
            imu_messages[msg.topic].append(msg)

    anomalies: list[AnomalyEvent] = []

    # GNSS anomalies
    for topic, messages in gnss_messages.items():
        anomalies.extend(_detect_gnss_anomalies(topic, messages))

    # IMU anomalies
    for topic, messages in imu_messages.items():
        anomalies.extend(_detect_imu_anomalies(topic, messages, warmup_cutoff_ns=warmup_cutoff_ns))

    # General rate anomalies (skip GNSS/IMU topics — already covered above)
    specialized_topics = set(gnss_topics) | set(imu_topics)
    for topic, timestamps in topic_timestamps.items():
        if topic not in specialized_topics and is_rate_anomaly_candidate(topic, summary.topics[topic].type):
            anomalies.extend(
                _detect_rate_anomalies(
                    topic,
                    timestamps,
                    warmup_cutoff_ns=warmup_cutoff_ns,
                )
            )

    # Sort by timestamp
    anomalies.sort(key=lambda a: a.timestamp_ns)

    if not anomalies:
        logger.info("No anomalies detected in %s", bag_path)

    report = AnomalyReport(
        bag_path=str(bag_path),
        total_anomalies=len(anomalies),
        anomalies=anomalies,
    )

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def _detect_gnss_anomalies(
    topic: str, messages: list[Message]
) -> list[AnomalyEvent]:
    """Detect GNSS-specific anomalies: position jumps, fix drops, HDOP spikes."""
    anomalies: list[AnomalyEvent] = []

    if len(messages) < 2:
        return anomalies

    # Collect positions and HDOP values
    positions: list[tuple[int, float, float]] = []  # (timestamp_ns, lat, lon)
    hdops: list[tuple[int, float]] = []  # (timestamp_ns, hdop)
    statuses: list[tuple[int, int]] = []  # (timestamp_ns, status)

    for msg in messages:
        d = msg.data
        status = d.get("status", -1)
        statuses.append((msg.timestamp_ns, status))

        lat = d.get("latitude")
        lon = d.get("longitude")
        if lat is not None and lon is not None and not math.isnan(lat) and not math.isnan(lon):
            positions.append((msg.timestamp_ns, lat, lon))

        cov = d.get("position_covariance", [])
        if cov and len(cov) >= 1 and cov[0] > 0:
            hdops.append((msg.timestamp_ns, math.sqrt(cov[0])))

    # Detect position jumps (distance > 50m between consecutive fixes)
    jump_threshold_deg = 50.0 / 111_000.0  # ~50m in degrees at equator
    for i in range(1, len(positions)):
        ts, lat, lon = positions[i]
        _, prev_lat, prev_lon = positions[i - 1]
        dlat = abs(lat - prev_lat)
        dlon = abs(lon - prev_lon)
        dist_deg = math.sqrt(dlat**2 + dlon**2)
        if dist_deg > jump_threshold_deg:
            dist_m = dist_deg * 111_000.0
            anomalies.append(AnomalyEvent(
                timestamp_ns=ts,
                topic=topic,
                type="gnss_jump",
                severity="high" if dist_m > 200 else "medium",
                description=f"Position jump of {dist_m:.1f}m",
            ))

    # Detect fix status drops (transition from fix to no-fix)
    for i in range(1, len(statuses)):
        ts, status = statuses[i]
        _, prev_status = statuses[i - 1]
        if prev_status >= 0 and status < 0:
            anomalies.append(AnomalyEvent(
                timestamp_ns=ts,
                topic=topic,
                type="gnss_fix_drop",
                severity="medium",
                description="GNSS fix lost",
            ))

    # Detect HDOP spikes (> mean + 3*std)
    if len(hdops) >= 5:
        hdop_values = np.array([h for _, h in hdops])
        mean_hdop = float(np.mean(hdop_values))
        std_hdop = float(np.std(hdop_values))
        threshold = mean_hdop + 3 * std_hdop
        for ts, hdop in hdops:
            if hdop > threshold:
                anomalies.append(AnomalyEvent(
                    timestamp_ns=ts,
                    topic=topic,
                    type="gnss_hdop_spike",
                    severity="low",
                    description=f"HDOP spike: {hdop:.2f} (threshold: {threshold:.2f})",
                ))

    return anomalies


def _detect_imu_anomalies(
    topic: str,
    messages: list[Message],
    *,
    warmup_cutoff_ns: int | None = None,
) -> list[AnomalyEvent]:
    """Detect IMU anomalies: acceleration spikes, gyro rate spikes, frequency drops."""
    anomalies: list[AnomalyEvent] = []

    if len(messages) < 10:
        return anomalies

    accel_magnitudes: list[tuple[int, float]] = []
    gyro_magnitudes: list[tuple[int, float]] = []
    timestamps: list[int] = []

    for msg in messages:
        d = msg.data
        timestamps.append(msg.timestamp_ns)

        la = d.get("linear_acceleration", {})
        if la:
            ax, ay, az = la.get("x", 0.0), la.get("y", 0.0), la.get("z", 0.0)
            mag = math.sqrt(ax**2 + ay**2 + az**2)
            accel_magnitudes.append((msg.timestamp_ns, mag))

        av = d.get("angular_velocity", {})
        if av:
            gx, gy, gz = av.get("x", 0.0), av.get("y", 0.0), av.get("z", 0.0)
            mag = math.sqrt(gx**2 + gy**2 + gz**2)
            gyro_magnitudes.append((msg.timestamp_ns, mag))

    # Detect acceleration spikes (> mean + 4*sigma)
    if accel_magnitudes:
        values = np.array([m for _, m in accel_magnitudes])
        mean_val = float(np.mean(values))
        std_val = float(np.std(values))
        if std_val > 0:
            threshold = mean_val + 4 * std_val
            for ts, mag in accel_magnitudes:
                if mag > threshold:
                    anomalies.append(AnomalyEvent(
                        timestamp_ns=ts,
                        topic=topic,
                        type="imu_accel_spike",
                        severity="high" if mag > mean_val + 6 * std_val else "medium",
                        description=f"Acceleration spike: {mag:.2f} m/s² (threshold: {threshold:.2f})",
                    ))

    # Detect gyro rate spikes (> mean + 4*sigma)
    if gyro_magnitudes:
        values = np.array([m for _, m in gyro_magnitudes])
        mean_val = float(np.mean(values))
        std_val = float(np.std(values))
        if std_val > 0:
            threshold = mean_val + 4 * std_val
            for ts, mag in gyro_magnitudes:
                if mag > threshold:
                    anomalies.append(AnomalyEvent(
                        timestamp_ns=ts,
                        topic=topic,
                        type="imu_gyro_spike",
                        severity="medium",
                        description=f"Gyro rate spike: {mag:.4f} rad/s (threshold: {threshold:.4f})",
                    ))

    # Detect frequency drops (gap > 3x median interval)
    if len(timestamps) >= 3:
        ts_arr = np.array(sorted(timestamps), dtype=np.int64)
        intervals = np.diff(ts_arr)
        median_interval = float(np.median(intervals))
        # Minimum 100µs to avoid false positives from bursty recording timestamps
        # Minimum 50ms absolute gap to avoid noise on bursty timestamps
        min_gap_ns = 50_000_000
        if median_interval > 100_000:  # 100µs in ns
            gap_threshold = 3 * median_interval
            for i, interval in enumerate(intervals):
                if warmup_cutoff_ns is not None and int(ts_arr[i + 1]) <= warmup_cutoff_ns:
                    continue
                if interval > gap_threshold and interval > min_gap_ns:
                    anomalies.append(AnomalyEvent(
                        timestamp_ns=int(ts_arr[i + 1]),
                        topic=topic,
                        type="imu_frequency_drop",
                        severity="medium",
                        description=f"Message gap: {interval / 1e6:.1f}ms (median: {median_interval / 1e6:.1f}ms)",
                    ))

    return anomalies


def _detect_rate_anomalies(
    topic: str,
    timestamps: list[int],
    *,
    warmup_cutoff_ns: int | None = None,
) -> list[AnomalyEvent]:
    """Detect message rate anomalies: gaps > 3x median interval."""
    anomalies: list[AnomalyEvent] = []

    if len(timestamps) < 10:
        return anomalies

    ts_arr = np.array(sorted(timestamps), dtype=np.int64)
    intervals = np.diff(ts_arr)
    median_interval = float(np.median(intervals))

    # Minimum 100µs to avoid false positives from bursty recording timestamps
    if median_interval <= 100_000:  # 100µs in ns
        return anomalies

    gap_threshold = 3 * median_interval
    # Minimum absolute gap of 50ms to avoid noise on high-rate topics
    min_gap_ns = 50_000_000  # 50ms
    for i, interval in enumerate(intervals):
        if warmup_cutoff_ns is not None and int(ts_arr[i + 1]) <= warmup_cutoff_ns:
            continue
        if interval > gap_threshold and interval > min_gap_ns:
            anomalies.append(AnomalyEvent(
                timestamp_ns=int(ts_arr[i + 1]),
                topic=topic,
                type="rate_gap",
                severity="high" if interval > 10 * median_interval else "medium",
                description=f"Message gap: {interval / 1e6:.1f}ms (expected: {median_interval / 1e6:.1f}ms)",
            ))

    return anomalies


def _rate_anomaly_warmup_cutoff_ns(start_time_ns: int, duration_sec: float) -> int | None:
    """Ignore startup transients for long recordings only."""
    if duration_sec < RATE_ANOMALY_WARMUP_MIN_DURATION_SEC:
        return None
    return start_time_ns + int(RATE_ANOMALY_WARMUP_SEC * 1e9)


def print_anomaly_report(report: AnomalyReport, console: Console | None = None) -> None:
    """Pretty-print an anomaly detection report."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Anomaly Detection: [cyan]{report.bag_path}[/cyan][/bold]")
    console.print(f"Total anomalies found: {report.total_anomalies}")

    if not report.anomalies:
        console.print("[green]No anomalies detected. Data looks clean.[/green]\n")
        return

    # Summary by type
    from collections import Counter

    type_counts = Counter(e.type for e in report.anomalies)
    severity_counts = Counter(e.severity for e in report.anomalies)

    # Type breakdown table
    table = Table(title="Anomaly Summary", show_header=True)
    table.add_column("Type", style="bold")
    table.add_column("Count", justify="right")
    table.add_column("Worst Topic")
    for atype, count in type_counts.most_common():
        worst_topic = Counter(
            e.topic for e in report.anomalies if e.type == atype
        ).most_common(1)[0][0]
        table.add_row(atype, str(count), worst_topic)
    console.print(table)

    # Severity bar
    console.print(
        f"  [red]High: {severity_counts.get('high', 0)}[/red] | "
        f"[dark_orange]Medium: {severity_counts.get('medium', 0)}[/dark_orange] | "
        f"[yellow]Low: {severity_counts.get('low', 0)}[/yellow]"
    )

    # Show first few high-severity events
    high_events = [e for e in report.anomalies if e.severity == "high"]
    if high_events:
        console.print(f"\n[bold red]Top issues ({min(len(high_events), 5)} of {len(high_events)} high-severity):[/bold red]")
        for event in high_events[:5]:
            console.print(
                f"  [red]{event.timestamp_sec:.1f}s[/red] {event.topic}: {event.description}"
            )
    console.print()
