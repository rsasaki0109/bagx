"""Inter-topic time synchronization analysis."""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass
from typing import TextIO

import numpy as np
from rich.console import Console
from rich.table import Table

from bagx.reader import BagReader

logger = logging.getLogger(__name__)


@dataclass
class SyncPairResult:
    topic_a: str
    topic_b: str
    count: int
    mean_delay_ms: float
    max_delay_ms: float
    min_delay_ms: float
    std_delay_ms: float
    median_delay_ms: float
    p95_delay_ms: float
    outlier_rate: float  # fraction of delays > 3*std from mean


@dataclass
class SyncReport:
    bag_path: str
    pairs: list[SyncPairResult]

    def to_dict(self) -> dict:
        return {
            "bag_path": self.bag_path,
            "pairs": [asdict(p) for p in self.pairs],
        }


def analyze_sync(
    bag_path: str,
    topic_a: str,
    topic_b: str,
    output_json: TextIO | None = None,
) -> SyncReport:
    """Analyze time synchronization between two topics."""
    reader = BagReader(bag_path)

    timestamps: dict[str, list[int]] = {topic_a: [], topic_b: []}

    for msg in reader.read_messages(topics=[topic_a, topic_b]):
        if msg.topic in timestamps:
            timestamps[msg.topic].append(msg.timestamp_ns)

    pairs = []
    if not timestamps[topic_a]:
        logger.warning("Topic %s not found or has no messages", topic_a)
    if not timestamps[topic_b]:
        logger.warning("Topic %s not found or has no messages", topic_b)
    if timestamps[topic_a] and timestamps[topic_b]:
        result = _compute_sync_pair(topic_a, topic_b, timestamps[topic_a], timestamps[topic_b])
        pairs.append(result)

    report = SyncReport(bag_path=str(bag_path), pairs=pairs)

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def analyze_sync_multi(
    bag_path: str,
    topics: list[str] | None = None,
    output_json: TextIO | None = None,
) -> SyncReport:
    """Analyze sync across all topic pairs (or specified topics)."""
    reader = BagReader(bag_path)
    summary = reader.summary()

    if topics is None:
        topics = list(summary.topics.keys())

    timestamps: dict[str, list[int]] = {t: [] for t in topics}

    for msg in reader.read_messages(topics=topics):
        if msg.topic in timestamps:
            timestamps[msg.topic].append(msg.timestamp_ns)

    # Filter topics with enough messages
    active_topics = [t for t in topics if len(timestamps[t]) > 5]

    pairs = []
    for i, t1 in enumerate(active_topics):
        for t2 in active_topics[i + 1 :]:
            result = _compute_sync_pair(t1, t2, timestamps[t1], timestamps[t2])
            pairs.append(result)

    report = SyncReport(bag_path=str(bag_path), pairs=pairs)

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def _compute_sync_pair(
    topic_a: str, topic_b: str, ts_a: list[int], ts_b: list[int]
) -> SyncPairResult:
    """Compute sync metrics between two timestamp arrays."""
    # Handle empty inputs
    if not ts_a or not ts_b:
        return SyncPairResult(
            topic_a=topic_a,
            topic_b=topic_b,
            count=0,
            mean_delay_ms=0.0,
            max_delay_ms=0.0,
            min_delay_ms=0.0,
            std_delay_ms=0.0,
            median_delay_ms=0.0,
            p95_delay_ms=0.0,
            outlier_rate=0.0,
        )

    arr_a = np.array(sorted(ts_a), dtype=np.int64)
    arr_b = np.array(sorted(ts_b), dtype=np.int64)

    # For each message in A, find nearest timestamp in B
    delays_ns = []
    j = 0
    for t in arr_a:
        while j < len(arr_b) - 1 and abs(arr_b[j + 1] - t) < abs(arr_b[j] - t):
            j += 1
        if j < len(arr_b):
            delays_ns.append(abs(int(arr_b[j] - t)))

    if not delays_ns:
        return SyncPairResult(
            topic_a=topic_a,
            topic_b=topic_b,
            count=0,
            mean_delay_ms=0.0,
            max_delay_ms=0.0,
            min_delay_ms=0.0,
            std_delay_ms=0.0,
            median_delay_ms=0.0,
            p95_delay_ms=0.0,
            outlier_rate=0.0,
        )

    delays_ms = np.array(delays_ns, dtype=np.float64) / 1e6

    mean_d = float(np.mean(delays_ms))
    std_d = float(np.std(delays_ms))

    # Outliers: > mean + 3*std
    outlier_threshold = mean_d + 3 * std_d
    outlier_count = int(np.sum(delays_ms > outlier_threshold))

    return SyncPairResult(
        topic_a=topic_a,
        topic_b=topic_b,
        count=len(delays_ms),
        mean_delay_ms=mean_d,
        max_delay_ms=float(np.max(delays_ms)),
        min_delay_ms=float(np.min(delays_ms)),
        std_delay_ms=std_d,
        median_delay_ms=float(np.median(delays_ms)),
        p95_delay_ms=float(np.percentile(delays_ms, 95)),
        outlier_rate=outlier_count / max(len(delays_ms), 1),
    )


def print_sync_report(report: SyncReport, console: Console | None = None) -> None:
    """Pretty-print a sync analysis report."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Sync Analysis: [cyan]{report.bag_path}[/cyan][/bold]")

    if not report.pairs:
        console.print("[yellow]No topic pairs found with sufficient messages.[/yellow]")
        return

    table = Table(show_header=True)
    table.add_column("Topic Pair", style="bold")
    table.add_column("Count", justify="right")
    table.add_column("Mean (ms)", justify="right")
    table.add_column("Max (ms)", justify="right")
    table.add_column("Median (ms)", justify="right")
    table.add_column("P95 (ms)", justify="right")
    table.add_column("Std (ms)", justify="right")
    table.add_column("Outlier %", justify="right")

    for p in report.pairs:
        color = "green" if p.mean_delay_ms < 10 else "yellow" if p.mean_delay_ms < 50 else "red"
        table.add_row(
            f"{p.topic_a} ↔ {p.topic_b}",
            str(p.count),
            f"[{color}]{p.mean_delay_ms:.2f}[/{color}]",
            f"{p.max_delay_ms:.2f}",
            f"{p.median_delay_ms:.2f}",
            f"{p.p95_delay_ms:.2f}",
            f"{p.std_delay_ms:.2f}",
            f"{p.outlier_rate:.1%}",
        )

    console.print(table)

    # Verdict
    for p in report.pairs:
        if p.mean_delay_ms < 5:
            console.print(
                f"  [green]:heavy_check_mark:[/green] {p.topic_a} ↔ {p.topic_b}: "
                f"[green]{p.mean_delay_ms:.1f}ms[/green] — excellent for tightly-coupled fusion"
            )
        elif p.mean_delay_ms < 20:
            console.print(
                f"  [green]:heavy_check_mark:[/green] {p.topic_a} ↔ {p.topic_b}: "
                f"[green]{p.mean_delay_ms:.1f}ms[/green] — good for most SLAM methods"
            )
        else:
            console.print(
                f"  [yellow]:warning:[/yellow] {p.topic_a} ↔ {p.topic_b}: "
                f"[yellow]{p.mean_delay_ms:.0f}ms[/yellow] — enable per-point deskew / timestamp compensation"
            )
    console.print()
