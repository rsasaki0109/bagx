"""Compare two bag files and report quality differences."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import TextIO

from rich.console import Console
from rich.table import Table

from bagx.eval import evaluate_bag


@dataclass
class ComparisonItem:
    metric: str
    value_a: float
    value_b: float
    diff: float
    diff_pct: float
    verdict: str  # "improved", "degraded", "unchanged"


@dataclass
class CompareReport:
    bag_a: str
    bag_b: str
    items: list[ComparisonItem]
    winner: str  # "A", "B", or "tie"
    summary: str

    def to_dict(self) -> dict:
        return {
            "bag_a": self.bag_a,
            "bag_b": self.bag_b,
            "items": [
                {
                    "metric": i.metric,
                    "value_a": None if math.isnan(i.value_a) else i.value_a,
                    "value_b": None if math.isnan(i.value_b) else i.value_b,
                    "diff": None if math.isnan(i.diff) else i.diff,
                    "diff_pct": None if math.isnan(i.diff_pct) else i.diff_pct,
                    "verdict": i.verdict,
                }
                for i in self.items
            ],
            "winner": self.winner,
            "summary": self.summary,
        }


def _compare_metric(
    name: str, val_a: float, val_b: float, higher_is_better: bool = True
) -> ComparisonItem:
    if math.isnan(val_a) or math.isnan(val_b):
        return ComparisonItem(
            metric=name,
            value_a=val_a,
            value_b=val_b,
            diff=float("nan"),
            diff_pct=float("nan"),
            verdict="unchanged",
        )

    diff = val_b - val_a
    base = abs(val_a) if val_a != 0 else 1.0
    diff_pct = (diff / base) * 100

    threshold = 1.0  # 1% threshold for significance
    if abs(diff_pct) < threshold:
        verdict = "unchanged"
    elif (diff > 0 and higher_is_better) or (diff < 0 and not higher_is_better):
        verdict = "improved"
    else:
        verdict = "degraded"

    return ComparisonItem(
        metric=name,
        value_a=val_a,
        value_b=val_b,
        diff=diff,
        diff_pct=diff_pct,
        verdict=verdict,
    )


def compare_bags(
    bag_a: str, bag_b: str, output_json: TextIO | None = None
) -> CompareReport:
    """Compare two bags by evaluating each and diffing metrics."""
    report_a = evaluate_bag(bag_a)
    report_b = evaluate_bag(bag_b)

    items: list[ComparisonItem] = []

    # Overall
    items.append(_compare_metric("Overall Score", report_a.overall_score, report_b.overall_score, True))

    # GNSS
    if report_a.gnss and report_b.gnss:
        ga, gb = report_a.gnss, report_b.gnss
        items.append(_compare_metric("GNSS Fix Rate", ga.fix_rate, gb.fix_rate, True))
        items.append(_compare_metric("GNSS HDOP Mean", ga.hdop_mean, gb.hdop_mean, False))
        items.append(_compare_metric("GNSS HDOP Max", ga.hdop_max, gb.hdop_max, False))
        items.append(_compare_metric("GNSS Score", ga.score, gb.score, True))

    # IMU
    if report_a.imu and report_b.imu:
        ia, ib = report_a.imu, report_b.imu
        items.append(_compare_metric("IMU Accel Noise X", ia.accel_noise_x, ib.accel_noise_x, False))
        items.append(_compare_metric("IMU Gyro Noise X", ia.gyro_noise_x, ib.gyro_noise_x, False))
        items.append(_compare_metric("IMU Frequency", ia.frequency_hz, ib.frequency_hz, True))
        items.append(_compare_metric("IMU Score", ia.score, ib.score, True))

    # Sync
    if report_a.sync and report_b.sync:
        sa, sb = report_a.sync, report_b.sync
        if sa.mean_delay_ms and sb.mean_delay_ms:
            import numpy as np

            items.append(_compare_metric(
                "Sync Mean Delay (ms)",
                float(np.mean(sa.mean_delay_ms)),
                float(np.mean(sb.mean_delay_ms)),
                False,
            ))
        items.append(_compare_metric("Sync Score", sa.score, sb.score, True))

    # Determine winner
    improved = sum(1 for i in items if i.verdict == "improved")
    degraded = sum(1 for i in items if i.verdict == "degraded")

    if improved > degraded:
        winner = "B"
        summary = f"B is better ({improved} improved, {degraded} degraded)"
    elif degraded > improved:
        winner = "A"
        summary = f"A is better ({degraded} metrics where B degraded)"
    else:
        winner = "tie"
        summary = "No significant difference"

    report = CompareReport(
        bag_a=bag_a, bag_b=bag_b, items=items, winner=winner, summary=summary
    )

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def print_compare_report(report: CompareReport, console: Console | None = None) -> None:
    """Pretty-print a comparison report."""
    if console is None:
        console = Console()

    console.print("\n[bold]Bag Comparison[/bold]")
    console.print(f"  A: [cyan]{report.bag_a}[/cyan]")
    console.print(f"  B: [cyan]{report.bag_b}[/cyan]")

    table = Table(show_header=True)
    table.add_column("Metric", style="bold")
    table.add_column("A", justify="right")
    table.add_column("B", justify="right")
    table.add_column("Diff", justify="right")
    table.add_column("Verdict", justify="center")

    for item in report.items:
        va = f"{item.value_a:.3f}" if not math.isnan(item.value_a) else "N/A"
        vb = f"{item.value_b:.3f}" if not math.isnan(item.value_b) else "N/A"

        if math.isnan(item.diff):
            diff_str = "N/A"
        else:
            sign = "+" if item.diff > 0 else ""
            diff_str = f"{sign}{item.diff:.3f} ({sign}{item.diff_pct:.1f}%)"

        verdict_colors = {"improved": "green", "degraded": "red", "unchanged": "dim"}
        color = verdict_colors.get(item.verdict, "white")
        verdict_str = f"[{color}]{item.verdict}[/{color}]"

        table.add_row(item.metric, va, vb, diff_str, verdict_str)

    console.print(table)

    winner_color = "green" if report.winner != "tie" else "yellow"
    console.print(f"\n[bold]Result: [{winner_color}]{report.summary}[/{winner_color}][/bold]\n")
