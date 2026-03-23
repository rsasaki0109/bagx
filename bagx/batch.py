"""Batch processing for multiple bag files.

Supports glob patterns and directory scanning to evaluate
or run anomaly detection on multiple bags at once.
"""

from __future__ import annotations

import csv
import glob as glob_module
import json
import logging
from pathlib import Path

from rich.console import Console
from rich.table import Table

from bagx.anomaly import detect_anomalies
from bagx.contracts import report_metadata
from bagx.eval import EvalReport, evaluate_bag

logger = logging.getLogger(__name__)

BAG_EXTENSIONS = {".db3", ".mcap"}


def resolve_bag_paths(patterns: list[str]) -> list[Path]:
    """Expand glob patterns and directories to actual bag file paths.

    Args:
        patterns: List of glob patterns, file paths, or directory paths.

    Returns:
        Sorted deduplicated list of resolved bag file paths.
    """
    paths: set[Path] = set()

    for pattern in patterns:
        p = Path(pattern)

        if p.is_file() and p.suffix in BAG_EXTENSIONS:
            paths.add(p.resolve())
        elif p.is_dir():
            # Scan directory recursively for bag files
            for ext in BAG_EXTENSIONS:
                for found in p.rglob(f"*{ext}"):
                    paths.add(found.resolve())
        else:
            # Treat as glob pattern
            for match in glob_module.glob(pattern, recursive=True):
                mp = Path(match)
                if mp.is_file() and mp.suffix in BAG_EXTENSIONS:
                    paths.add(mp.resolve())

    return sorted(paths)


def batch_eval(
    paths: list[str],
    output_csv: str | None = None,
) -> list[EvalReport]:
    """Evaluate multiple bags and optionally output a summary CSV.

    Args:
        paths: List of glob patterns, file paths, or directory paths.
        output_csv: Optional path to write CSV summary.

    Returns:
        List of EvalReport for each bag.
    """
    bag_paths = resolve_bag_paths(paths)
    if not bag_paths:
        logger.warning("No bag files found matching the given paths")
        return []

    reports: list[EvalReport] = []
    for bag_path in bag_paths:
        report = evaluate_bag(str(bag_path))
        reports.append(report)

    if output_csv:
        _write_eval_csv(reports, output_csv)

    return reports


def _write_eval_csv(reports: list[EvalReport], output_path: str) -> None:
    """Write evaluation reports to a CSV file."""
    fieldnames = [
        "bag_path",
        "duration_sec",
        "message_count",
        "gnss_score",
        "imu_score",
        "sync_score",
        "overall_score",
    ]

    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in reports:
            writer.writerow({
                "bag_path": r.bag_path,
                "duration_sec": f"{r.duration_sec:.2f}",
                "message_count": r.total_messages,
                "gnss_score": f"{r.gnss.score:.1f}" if r.gnss else "",
                "imu_score": f"{r.imu.score:.1f}" if r.imu else "",
                "sync_score": f"{r.sync.score:.1f}" if r.sync else "",
                "overall_score": f"{r.overall_score:.1f}",
            })


def batch_anomaly(
    paths: list[str],
    output_json: str | None = None,
) -> dict:
    """Run anomaly detection on multiple bags.

    Args:
        paths: List of glob patterns, file paths, or directory paths.
        output_json: Optional path to write aggregated JSON report.

    Returns:
        Aggregated results dict with per-bag anomaly reports.
    """
    bag_paths = resolve_bag_paths(paths)
    if not bag_paths:
        return {"total_bags": 0, "total_anomalies": 0, "bags": []}

    results: list[dict] = []
    total_anomalies = 0

    for bag_path in bag_paths:
        report = detect_anomalies(str(bag_path))
        total_anomalies += report.total_anomalies
        results.append(report.to_dict())

    aggregated = {
        "total_bags": len(results),
        "total_anomalies": total_anomalies,
        "bags": results,
    }
    aggregated.update(report_metadata("batch_anomaly"))

    if output_json:
        with open(output_json, "w") as f:
            json.dump(aggregated, f, indent=2)

    return aggregated


def print_batch_eval_table(
    reports: list[EvalReport], console: Console | None = None
) -> None:
    """Print a summary table of batch evaluation results with color coding."""
    if console is None:
        console = Console()

    if not reports:
        console.print("[yellow]No bags found to evaluate.[/yellow]")
        return

    table = Table(title="Batch Evaluation Summary", show_header=True)
    table.add_column("Bag", style="bold", max_width=50)
    table.add_column("Duration (s)", justify="right")
    table.add_column("Messages", justify="right")
    table.add_column("GNSS", justify="right")
    table.add_column("IMU", justify="right")
    table.add_column("Sync", justify="right")
    table.add_column("Overall", justify="right")

    for r in reports:
        bag_name = Path(r.bag_path).name

        def _score_cell(score: float | None) -> str:
            if score is None:
                return "[dim]N/A[/dim]"
            color = "green" if score >= 70 else "yellow" if score >= 40 else "red"
            return f"[{color}]{score:.1f}[/{color}]"

        gnss_score = r.gnss.score if r.gnss else None
        imu_score = r.imu.score if r.imu else None
        sync_score = r.sync.score if r.sync else None

        overall_color = (
            "green" if r.overall_score >= 70
            else "yellow" if r.overall_score >= 40
            else "red"
        )

        table.add_row(
            bag_name,
            f"{r.duration_sec:.1f}",
            f"{r.total_messages:,}",
            _score_cell(gnss_score),
            _score_cell(imu_score),
            _score_cell(sync_score),
            f"[bold {overall_color}]{r.overall_score:.1f}[/bold {overall_color}]",
        )

    console.print()
    console.print(table)

    # Rank and recommend
    ranked = sorted(reports, key=lambda r: r.overall_score, reverse=True)
    best = ranked[0]
    worst = ranked[-1]
    best_name = Path(best.bag_path).name
    worst_name = Path(worst.bag_path).name

    console.print(f"\n[bold]{len(reports)} bag(s) evaluated.[/bold]")
    if len(reports) > 1:
        console.print(f"  [green]:heavy_check_mark:[/green] Best: [bold]{best_name}[/bold] ({best.overall_score:.1f})")
        if worst.overall_score < best.overall_score:
            console.print(f"  [red]:x:[/red] Worst: [bold]{worst_name}[/bold] ({worst.overall_score:.1f})")
    console.print()
