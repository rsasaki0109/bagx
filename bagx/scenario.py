"""Scenario/dangerous scene extraction from rosbag files.

Identifies and extracts time segments where interesting or dangerous
scenarios occur, such as GNSS loss, sensor dropouts, high dynamics, etc.
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

from bagx.reader import BagReader

logger = logging.getLogger(__name__)


@dataclass
class Scenario:
    start_time_ns: int
    end_time_ns: int
    duration_ns: int
    type: str  # "gnss_lost", "sensor_dropout", "high_dynamics", "sync_degraded"
    severity: str  # "low", "medium", "high"
    description: str

    @property
    def start_time_sec(self) -> float:
        return self.start_time_ns / 1e9

    @property
    def end_time_sec(self) -> float:
        return self.end_time_ns / 1e9

    @property
    def duration_sec(self) -> float:
        return self.duration_ns / 1e9


@dataclass
class ScenarioReport:
    bag_path: str
    total_scenarios: int
    scenarios: list[Scenario] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "bag_path": self.bag_path,
            "total_scenarios": self.total_scenarios,
            "scenarios": [asdict(s) for s in self.scenarios],
        }


def detect_scenarios(
    bag_path: str,
    gnss_lost_threshold_sec: float = 2.0,
    dropout_threshold_sec: float = 2.0,
    accel_threshold_mps2: float = 15.0,
    sync_delay_threshold_ms: float = 100.0,
    output_json: TextIO | None = None,
) -> ScenarioReport:
    """Detect interesting/dangerous scenarios in a bag file.

    Args:
        bag_path: Path to the bag file.
        gnss_lost_threshold_sec: Duration of no-fix to consider GNSS lost.
        dropout_threshold_sec: Duration of topic silence to consider a dropout.
        accel_threshold_mps2: Acceleration magnitude threshold for high dynamics.
        sync_delay_threshold_ms: Inter-topic delay threshold for sync degradation.
        output_json: Optional file handle for JSON output.

    Returns:
        ScenarioReport with detected scenarios.
    """
    reader = BagReader(bag_path)
    summary = reader.summary()

    # Classify topics
    gnss_topics: list[str] = []
    imu_topics: list[str] = []
    all_topics: list[str] = list(summary.topics.keys())

    for name, info in summary.topics.items():
        if "NavSatFix" in info.type or "navsatfix" in info.type.lower():
            gnss_topics.append(name)
        elif "Imu" in info.type or "imu" in info.type.lower():
            imu_topics.append(name)

    # Collect only lightweight extracted data per topic (avoid storing full messages
    # for heavy topics like PointCloud2).
    # GNSS: store (timestamp_ns, status) tuples
    gnss_data: dict[str, list[tuple[int, int]]] = {t: [] for t in gnss_topics}
    # IMU: store (timestamp_ns, accel_x, accel_y, accel_z) tuples
    imu_data: dict[str, list[tuple[int, float, float, float]]] = {t: [] for t in imu_topics}
    # All topics: timestamps only
    topic_timestamps: dict[str, list[int]] = {t: [] for t in all_topics}

    gnss_set = set(gnss_topics)
    imu_set = set(imu_topics)

    for msg in reader.read_messages(topics=None):
        if msg.topic in topic_timestamps:
            topic_timestamps[msg.topic].append(msg.timestamp_ns)
        if msg.topic in gnss_set:
            status = msg.data.get("status", -1)
            gnss_data[msg.topic].append((msg.timestamp_ns, status))
        elif msg.topic in imu_set:
            la = msg.data.get("linear_acceleration", {})
            if la:
                imu_data[msg.topic].append((
                    msg.timestamp_ns,
                    la.get("x", 0.0),
                    la.get("y", 0.0),
                    la.get("z", 0.0),
                ))
        # Other topics: only timestamp kept above -> no memory for heavy data.

    scenarios: list[Scenario] = []

    # Detect GNSS lost scenarios
    for topic, data in gnss_data.items():
        scenarios.extend(_detect_gnss_lost(topic, data, gnss_lost_threshold_sec))

    # Detect sensor dropouts
    for topic, timestamps in topic_timestamps.items():
        scenarios.extend(_detect_sensor_dropout(topic, timestamps, dropout_threshold_sec))

    # Detect high dynamics
    for topic, data in imu_data.items():
        scenarios.extend(_detect_high_dynamics(topic, data, accel_threshold_mps2))

    # Detect sync degradation
    active_topics = [t for t in all_topics if len(topic_timestamps.get(t, [])) > 5]
    if len(active_topics) >= 2:
        scenarios.extend(
            _detect_sync_degraded(topic_timestamps, active_topics, sync_delay_threshold_ms)
        )

    # Sort by start time
    scenarios.sort(key=lambda s: s.start_time_ns)

    report = ScenarioReport(
        bag_path=str(bag_path),
        total_scenarios=len(scenarios),
        scenarios=scenarios,
    )

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def _detect_gnss_lost(
    topic: str,
    data: list[tuple[int, int]],
    threshold_sec: float,
) -> list[Scenario]:
    """Detect periods where GNSS fix is lost for > threshold duration.

    Args:
        topic: Topic name.
        data: List of (timestamp_ns, status) tuples.
        threshold_sec: Minimum duration of no-fix to report.
    """
    scenarios: list[Scenario] = []

    if not data:
        return scenarios

    threshold_ns = int(threshold_sec * 1e9)
    no_fix_start: int | None = None

    for timestamp_ns, status in data:
        if status < 0:
            if no_fix_start is None:
                no_fix_start = timestamp_ns
        else:
            if no_fix_start is not None:
                duration_ns = timestamp_ns - no_fix_start
                if duration_ns >= threshold_ns:
                    scenarios.append(Scenario(
                        start_time_ns=no_fix_start,
                        end_time_ns=timestamp_ns,
                        duration_ns=duration_ns,
                        type="gnss_lost",
                        severity="high" if duration_ns > threshold_ns * 5 else "medium",
                        description=f"GNSS fix lost for {duration_ns / 1e9:.1f}s on {topic}",
                    ))
                no_fix_start = None

    # Handle case where no-fix extends to end of data
    if no_fix_start is not None and data:
        last_timestamp_ns = data[-1][0]
        duration_ns = last_timestamp_ns - no_fix_start
        if duration_ns >= threshold_ns:
            scenarios.append(Scenario(
                start_time_ns=no_fix_start,
                end_time_ns=last_timestamp_ns,
                duration_ns=duration_ns,
                type="gnss_lost",
                severity="high" if duration_ns > threshold_ns * 5 else "medium",
                description=f"GNSS fix lost for {duration_ns / 1e9:.1f}s on {topic} (until end)",
            ))

    return scenarios


def _detect_sensor_dropout(
    topic: str, timestamps: list[int], threshold_sec: float
) -> list[Scenario]:
    """Detect periods where a topic stops publishing for > threshold duration."""
    scenarios: list[Scenario] = []

    if len(timestamps) < 2:
        return scenarios

    threshold_ns = int(threshold_sec * 1e9)
    ts_sorted = sorted(timestamps)

    for i in range(1, len(ts_sorted)):
        gap_ns = ts_sorted[i] - ts_sorted[i - 1]
        if gap_ns >= threshold_ns:
            scenarios.append(Scenario(
                start_time_ns=ts_sorted[i - 1],
                end_time_ns=ts_sorted[i],
                duration_ns=gap_ns,
                type="sensor_dropout",
                severity="high" if gap_ns > threshold_ns * 5 else "medium",
                description=f"Topic {topic} silent for {gap_ns / 1e9:.1f}s",
            ))

    return scenarios


def _detect_high_dynamics(
    topic: str,
    data: list[tuple[int, float, float, float]],
    accel_threshold_mps2: float,
) -> list[Scenario]:
    """Detect periods of high acceleration (hard braking, sharp turns).

    Groups consecutive high-acceleration samples into a single scenario.

    Args:
        topic: Topic name.
        data: List of (timestamp_ns, accel_x, accel_y, accel_z) tuples.
        accel_threshold_mps2: Acceleration magnitude threshold.
    """
    scenarios: list[Scenario] = []

    if not data:
        return scenarios

    high_start: int | None = None
    high_end: int | None = None
    max_accel: float = 0.0

    for timestamp_ns, ax, ay, az in data:
        # Subtract gravity approximation for magnitude check
        mag = math.sqrt(ax**2 + ay**2 + (az - 9.81)**2)

        if mag > accel_threshold_mps2:
            if high_start is None:
                high_start = timestamp_ns
            high_end = timestamp_ns
            max_accel = max(max_accel, mag)
        else:
            if high_start is not None and high_end is not None:
                duration_ns = max(high_end - high_start, 1)
                scenarios.append(Scenario(
                    start_time_ns=high_start,
                    end_time_ns=high_end,
                    duration_ns=duration_ns,
                    type="high_dynamics",
                    severity="high" if max_accel > accel_threshold_mps2 * 2 else "medium",
                    description=f"High acceleration on {topic}: peak {max_accel:.1f} m/s²",
                ))
                high_start = None
                high_end = None
                max_accel = 0.0

    # Close any open segment
    if high_start is not None and high_end is not None:
        duration_ns = max(high_end - high_start, 1)
        scenarios.append(Scenario(
            start_time_ns=high_start,
            end_time_ns=high_end,
            duration_ns=duration_ns,
            type="high_dynamics",
            severity="high" if max_accel > accel_threshold_mps2 * 2 else "medium",
            description=f"High acceleration on {topic}: peak {max_accel:.1f} m/s²",
        ))

    return scenarios


def _detect_sync_degraded(
    topic_timestamps: dict[str, list[int]],
    topics: list[str],
    delay_threshold_ms: float,
) -> list[Scenario]:
    """Detect periods where inter-topic sync delay exceeds threshold.

    Checks pairs of topics and finds time windows where the nearest-neighbor
    delay exceeds the threshold for sustained periods.
    """
    scenarios: list[Scenario] = []

    sorted_topics = sorted(topics)
    pairs = list(zip(sorted_topics[:-1], sorted_topics[1:]))

    for t1, t2 in pairs[:5]:  # limit pairs
        ts1 = np.array(sorted(topic_timestamps[t1]), dtype=np.int64)
        ts2 = np.array(sorted(topic_timestamps[t2]), dtype=np.int64)

        if len(ts1) < 2 or len(ts2) < 2:
            continue

        # For each message in t1, find nearest in t2 and compute delay
        delays: list[tuple[int, float]] = []  # (timestamp_ns, delay_ms)
        j = 0
        for t in ts1:
            while j < len(ts2) - 1 and abs(ts2[j + 1] - t) < abs(ts2[j] - t):
                j += 1
            if j < len(ts2):
                delay_ms = abs(int(ts2[j] - t)) / 1e6
                delays.append((int(t), delay_ms))

        # Find sustained periods above threshold
        degraded_start: int | None = None
        degraded_end: int | None = None
        max_delay: float = 0.0
        consecutive_count = 0
        min_sustained = 3  # require at least 3 consecutive high-delay samples

        for ts, delay_ms in delays:
            if delay_ms > delay_threshold_ms:
                if degraded_start is None:
                    degraded_start = ts
                degraded_end = ts
                max_delay = max(max_delay, delay_ms)
                consecutive_count += 1
            else:
                if degraded_start is not None and consecutive_count >= min_sustained:
                    duration_ns = degraded_end - degraded_start
                    if duration_ns > 0:
                        scenarios.append(Scenario(
                            start_time_ns=degraded_start,
                            end_time_ns=degraded_end,
                            duration_ns=duration_ns,
                            type="sync_degraded",
                            severity="high" if max_delay > delay_threshold_ms * 3 else "medium",
                            description=(
                                f"Sync degraded between {t1} and {t2}: "
                                f"max delay {max_delay:.1f}ms over {duration_ns / 1e9:.1f}s"
                            ),
                        ))
                degraded_start = None
                degraded_end = None
                max_delay = 0.0
                consecutive_count = 0

        # Close any open segment
        if degraded_start is not None and consecutive_count >= min_sustained:
            duration_ns = degraded_end - degraded_start
            if duration_ns > 0:
                scenarios.append(Scenario(
                    start_time_ns=degraded_start,
                    end_time_ns=degraded_end,
                    duration_ns=duration_ns,
                    type="sync_degraded",
                    severity="high" if max_delay > delay_threshold_ms * 3 else "medium",
                    description=(
                        f"Sync degraded between {t1} and {t2}: "
                        f"max delay {max_delay:.1f}ms over {duration_ns / 1e9:.1f}s"
                    ),
                ))

    return scenarios


def print_scenario_report(report: ScenarioReport, console: Console | None = None) -> None:
    """Pretty-print a scenario detection report."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Scenario Detection: [cyan]{report.bag_path}[/cyan][/bold]")
    console.print(f"Total scenarios found: {report.total_scenarios}")

    if not report.scenarios:
        console.print("[green]No notable scenarios detected.[/green]")
        return

    table = Table(show_header=True)
    table.add_column("Start (s)", justify="right")
    table.add_column("End (s)", justify="right")
    table.add_column("Duration (s)", justify="right")
    table.add_column("Type", style="bold")
    table.add_column("Severity", justify="center")
    table.add_column("Description")

    for s in report.scenarios:
        severity_colors = {"low": "yellow", "medium": "dark_orange", "high": "red"}
        color = severity_colors.get(s.severity, "white")
        table.add_row(
            f"{s.start_time_sec:.3f}",
            f"{s.end_time_sec:.3f}",
            f"{s.duration_sec:.2f}",
            s.type,
            f"[{color}]{s.severity}[/{color}]",
            s.description,
        )

    console.print(table)

    # Summary by type
    type_counts: dict[str, int] = {}
    for s in report.scenarios:
        type_counts[s.type] = type_counts.get(s.type, 0) + 1

    summary_parts = [f"{t}: {c}" for t, c in sorted(type_counts.items())]
    console.print(f"\n  Summary: {', '.join(summary_parts)}\n")
