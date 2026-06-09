"""bagx CLI - Post-processing analysis engine for ROS2 rosbag data."""

from __future__ import annotations

import json
import logging
import sys
from typing import Optional

import typer
from rich.console import Console

from bagx import __version__


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"bagx {__version__}")
        raise typer.Exit()


app = typer.Typer(
    name="bagx",
    help="Post-processing analysis engine for ROS2 rosbag data.",
    no_args_is_help=True,
)
console = Console()

batch_app = typer.Typer(help="Batch operations on multiple bags")
app.add_typer(batch_app, name="batch")
rules_app = typer.Typer(help="Custom rule plugins")
app.add_typer(rules_app, name="rules")
domains_app = typer.Typer(help="Domain detection plugins")
app.add_typer(domains_app, name="domains")
tune_app = typer.Typer(help="SLAM framework config generation", invoke_without_command=True)
app.add_typer(tune_app, name="tune")


@app.callback()
def main_callback(
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Enable debug logging"),
    quiet: bool = typer.Option(False, "--quiet", "-q", help="Suppress all but error messages"),
    version: Optional[bool] = typer.Option(
        None, "--version", callback=_version_callback, is_eager=True, help="Show version and exit"
    ),
) -> None:
    """Post-processing analysis engine for ROS2 rosbag data."""
    import os

    level = logging.WARNING
    ros_level = "WARN"
    if verbose:
        level = logging.DEBUG
        ros_level = "INFO"
        os.environ.pop("BAGX_SUPPRESS_NATIVE_STDERR", None)
    elif quiet:
        level = logging.ERROR
        ros_level = "ERROR"
        os.environ["BAGX_SUPPRESS_NATIVE_STDERR"] = "1"
    else:
        os.environ["BAGX_SUPPRESS_NATIVE_STDERR"] = "1"
    # Suppress ROS2 internal rcutils logging (not controlled by Python logging)
    os.environ.setdefault("RCUTILS_LOGGING_MIN_SEVERITY", ros_level)
    logging.basicConfig(format="%(levelname)s: %(message)s", level=level)


@batch_app.command("eval")
def batch_eval_cmd(
    paths: list[str] = typer.Argument(..., help="Bag files, directories, or glob patterns"),
    csv_output: Optional[str] = typer.Option(
        None, "--csv", "-c", help="Output CSV summary to file"
    ),
) -> None:
    """Evaluate quality of multiple bag files.

    Accepts glob patterns (e.g. *.db3), directories, or individual file paths.
    """
    from bagx.batch import batch_eval, print_batch_eval_table

    try:
        reports = batch_eval(paths, output_csv=csv_output)
        print_batch_eval_table(reports, console)
        if csv_output:
            console.print(f"CSV summary saved to: [cyan]{csv_output}[/cyan]")
    except Exception as e:
        console.print(f"[red]Error in batch eval: {e}[/red]")
        raise typer.Exit(1)


@batch_app.command("anomaly")
def batch_anomaly_cmd(
    paths: list[str] = typer.Argument(..., help="Bag files, directories, or glob patterns"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
) -> None:
    """Run anomaly detection on multiple bag files.

    Accepts glob patterns (e.g. *.db3), directories, or individual file paths.
    """
    from bagx.batch import batch_anomaly

    try:
        result = batch_anomaly(paths, output_json=json_output)
        console.print("\n[bold]Batch Anomaly Detection[/bold]")
        console.print(f"  Bags analyzed: {result['total_bags']}")
        console.print(f"  Total anomalies: {result['total_anomalies']}\n")
        if json_output:
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
    except Exception as e:
        console.print(f"[red]Error in batch anomaly: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def benchmark(
    manifest: str = typer.Argument(..., help="Path to a benchmark manifest JSON file"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
    rules_path: Optional[str] = typer.Option(
        None, "--rules", help="Optional custom rule file or plugin name applied to all benchmark cases"
    ),
    case: Optional[list[str]] = typer.Option(
        None, "--case", help="Run only the named benchmark case(s)"
    ),
    fail_on_missing: bool = typer.Option(
        False, "--fail-on-missing", help="Treat missing bag paths as failures"
    ),
    exit_on: Optional[str] = typer.Option(
        None, "--exit-on", help="Exit non-zero when any case has a finding with this severity or higher: info, warning, error, critical"
    ),
) -> None:
    """Run a manifest-driven benchmark suite for curated rosbag checks."""
    from bagx.benchmark import print_benchmark_report, run_benchmark_suite
    from bagx.findings import SEVERITY_ORDER, severity_at_least

    if exit_on is not None and exit_on not in SEVERITY_ORDER:
        console.print(
            f"[red]Error: --exit-on must be one of {list(SEVERITY_ORDER)}, got {exit_on!r}[/red]"
        )
        raise typer.Exit(2)

    try:
        report = run_benchmark_suite(
            manifest,
            fail_on_missing=fail_on_missing,
            selected_cases=case,
            rules_path=rules_path,
        )
        print_benchmark_report(report, console)
        if json_output:
            with open(json_output, "w") as f:
                json.dump(report.to_dict(), f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
        if report.failed_cases > 0:
            raise typer.Exit(1)
        if exit_on and report.worst_severity and severity_at_least(report.worst_severity, exit_on):
            console.print(
                f"[red]Exiting non-zero: worst finding severity {report.worst_severity!r} >= --exit-on {exit_on!r}[/red]"
            )
            raise typer.Exit(1)
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except typer.Exit:
        raise
    except Exception as e:
        console.print(f"[red]Error running benchmark suite: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def eval(
    bag: str = typer.Argument(..., help="Path to the bag file (.db3 or directory)"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
    rules_path: Optional[str] = typer.Option(
        None, "--rules", help="Optional custom rule file or plugin name for custom topic/message conventions"
    ),
    findings_only: bool = typer.Option(
        False, "--findings-only", help="Show only the structured findings list, suppressing sensor tables and recommendations"
    ),
    severity_min: Optional[str] = typer.Option(
        None, "--severity-min", help="Filter findings to this severity or higher: info, warning, error, critical"
    ),
    include_anomaly: bool = typer.Option(
        False, "--include-anomaly", help="Also run anomaly detection and merge its temporal findings (fix-lost segments, spike clusters, rate gaps) into the report"
    ),
    badge_output: Optional[str] = typer.Option(
        None, "--badge", help="Write a shields.io endpoint JSON badge of the readiness score to this file (reference via https://img.shields.io/endpoint?url=...)"
    ),
    badge_label: Optional[str] = typer.Option(
        None, "--badge-label", help="Override the badge label (default: '<domains> readiness')"
    ),
    tune_framework: Optional[str] = typer.Option(
        None, "--tune", help="Generate a SLAM framework config YAML (e.g. fast_lio, kiss_icp)"
    ),
    tune_output: Optional[str] = typer.Option(
        None, "-o", "--output", help="Output YAML path or directory for --tune"
    ),
    html_output: Optional[str] = typer.Option(
        None, "--html", help="Write a self-contained visual HTML report to this file"
    ),
) -> None:
    """Evaluate quality of a single bag file.

    Analyzes GNSS fix rate/HDOP, IMU noise/bias, and inter-topic sync,
    producing a composite quality score.
    """
    from bagx.eval import evaluate_bag, print_eval_report
    from bagx.findings import SEVERITY_ORDER, severity_at_least

    if severity_min is not None and severity_min not in SEVERITY_ORDER:
        console.print(
            f"[red]Error: --severity-min must be one of {list(SEVERITY_ORDER)}, got {severity_min!r}[/red]"
        )
        raise typer.Exit(2)

    try:
        report = evaluate_bag(
            bag,
            custom_rules_path=rules_path,
            include_anomaly=include_anomaly,
        )
        print_eval_report(
            report,
            console,
            findings_only=findings_only,
            severity_min=severity_min,
        )
        if json_output:
            data = report.to_dict()
            if severity_min:
                data["findings"] = [
                    f for f in data["findings"] if severity_at_least(f["severity"], severity_min)
                ]
            with open(json_output, "w") as f:
                json.dump(data, f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
        if badge_output:
            from bagx.badge import eval_badge

            with open(badge_output, "w") as f:
                json.dump(eval_badge(report, label=badge_label), f, indent=2)
            console.print(f"Readiness badge saved to: [cyan]{badge_output}[/cyan]")
        if tune_framework:
            from bagx.tune import default_output_path, tune_report

            yaml_text = tune_report(report, tune_framework)
            out_path = default_output_path(tune_framework, tune_output)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(yaml_text, encoding="utf-8")
            console.print(f"Tuned config saved to: [cyan]{out_path}[/cyan]")
        if html_output:
            from bagx.report_html import write_eval_html

            data = report.to_dict()
            if severity_min:
                data["findings"] = [
                    f for f in data["findings"] if severity_at_least(f["severity"], severity_min)
                ]
            out_path = write_eval_html(data, html_output)
            console.print(f"HTML report saved to: [cyan]{out_path}[/cyan]")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error evaluating bag: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def report(
    json_report: str = typer.Argument(..., help="Path to a bagx eval JSON report"),
    html_output: Optional[str] = typer.Option(
        None, "--html", help="Write a self-contained visual HTML report to this file"
    ),
) -> None:
    """Convert a saved bagx eval JSON report into other formats."""
    if not html_output:
        console.print("[red]Error: specify --html <path> to render a visual report[/red]")
        raise typer.Exit(2)

    try:
        from bagx.report_html import load_eval_report, write_eval_html

        payload = load_eval_report(json_report)
        out_path = write_eval_html(payload, html_output)
        console.print(f"HTML report saved to: [cyan]{out_path}[/cyan]")
    except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def diff(
    baseline: str = typer.Argument(..., help="Path to baseline bagx eval JSON report"),
    current: str = typer.Argument(..., help="Path to current bagx eval JSON report"),
    format: str = typer.Option(
        "text", "--format", help="Output format: text, json, markdown"
    ),
    output: Optional[str] = typer.Option(
        None, "--output", "-o", help="Write output to a file instead of stdout"
    ),
    include_same: bool = typer.Option(
        False, "--include-same", help="Include findings whose severity is unchanged"
    ),
    exit_on: Optional[str] = typer.Option(
        None,
        "--exit-on",
        help="Exit non-zero when a regression at this severity or higher is detected (info, warning, error, critical)",
    ),
) -> None:
    """Diff two bagx eval JSON reports by finding id."""
    from bagx.diff import (
        print_diff_markdown,
        print_diff_text,
        run_diff,
        write_diff_json,
    )
    from bagx.findings import SEVERITY_ORDER

    if format not in {"text", "json", "markdown"}:
        console.print(f"[red]Error: --format must be one of text, json, markdown (got {format!r})[/red]")
        raise typer.Exit(2)
    if exit_on is not None and exit_on not in SEVERITY_ORDER:
        console.print(
            f"[red]Error: --exit-on must be one of {list(SEVERITY_ORDER)}, got {exit_on!r}[/red]"
        )
        raise typer.Exit(2)

    try:
        diff_result = run_diff(baseline, current, include_same=include_same)
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)

    if format == "json":
        if output:
            with open(output, "w") as f:
                write_diff_json(diff_result, f)
            console.print(f"Diff JSON saved to: [cyan]{output}[/cyan]")
        else:
            write_diff_json(diff_result, sys.stdout)
            sys.stdout.write("\n")
    elif format == "markdown":
        if output:
            with open(output, "w") as f:
                print_diff_markdown(diff_result, f)
            console.print(f"Diff markdown saved to: [cyan]{output}[/cyan]")
        else:
            print_diff_markdown(diff_result, sys.stdout)
    else:
        target_console = Console(file=open(output, "w")) if output else console
        print_diff_text(diff_result, target_console)
        if output:
            console.print(f"Diff text saved to: [cyan]{output}[/cyan]")

    if exit_on and diff_result.has_regression(exit_on):
        raise typer.Exit(1)


@rules_app.command("list")
def rules_list() -> None:
    """List discoverable custom-rule plugins."""
    from rich.table import Table

    from bagx.custom_rules import discover_rule_plugins

    plugins = discover_rule_plugins()
    if not plugins:
        console.print("No custom-rule plugins found.")
        return

    table = Table(title="Custom Rule Plugins", show_header=True)
    table.add_column("Name", style="bold")
    table.add_column("Source")
    table.add_column("Description")
    for plugin in plugins:
        table.add_row(plugin.name, plugin.source, plugin.description or "-")
    console.print(table)


@tune_app.callback()
def tune_main(
    ctx: typer.Context,
    list_frameworks: bool = typer.Option(False, "--list", help="List supported SLAM frameworks"),
) -> None:
    """Generate SLAM framework configs from bag metrics."""
    if ctx.invoked_subcommand is not None:
        return
    if list_frameworks:
        from rich.table import Table

        from bagx.tune import discover_tuners, plugin_source_label as tuner_source_label

        tuners = discover_tuners()
        table = Table(title="SLAM Framework Tuners", show_header=True)
        table.add_column("Name", style="bold")
        table.add_column("Aliases")
        table.add_column("Source")
        for tuner in tuners:
            aliases = ", ".join(tuner.aliases) if tuner.aliases else "-"
            table.add_row(tuner.name, aliases, tuner_source_label(tuner))
        console.print(table)
        raise typer.Exit()
    console.print(ctx.get_help())
    raise typer.Exit()


@domains_app.command("list")
def domains_list() -> None:
    """List built-in and installed domain detection plugins."""
    from rich.table import Table

    from bagx.domain_plugins import discover_domain_plugins, plugin_source_label

    plugins = discover_domain_plugins()
    table = Table(title="Domain Plugins", show_header=True)
    table.add_column("Name", style="bold")
    table.add_column("Source")
    for plugin in plugins:
        table.add_row(plugin.name, plugin_source_label(plugin))
    console.print(table)


@app.command()
def compare(
    bag_a: str = typer.Argument(..., help="Path to the first bag file"),
    bag_b: str = typer.Argument(..., help="Path to the second bag file"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
) -> None:
    """Compare quality metrics of two bag files.

    Evaluates both bags and reports per-metric differences,
    indicating which bag is better overall.
    """
    from bagx.compare import compare_bags, print_compare_report

    try:
        report = compare_bags(bag_a, bag_b)
        print_compare_report(report, console)
        if json_output:
            with open(json_output, "w") as f:
                json.dump(report.to_dict(), f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error comparing bags: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def sync(
    bag: str = typer.Argument(..., help="Path to the bag file"),
    topic_a: str = typer.Argument(..., help="First topic name"),
    topic_b: str = typer.Argument(..., help="Second topic name"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
) -> None:
    """Analyze time synchronization between two topics.

    Reports mean/max/median delay, variance, P95, and outlier rate.
    """
    from bagx.sync import analyze_sync, print_sync_report

    try:
        report = analyze_sync(bag, topic_a, topic_b)
        print_sync_report(report, console)
        if json_output:
            with open(json_output, "w") as f:
                json.dump(report.to_dict(), f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error analyzing sync: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def export(
    bag: str = typer.Argument(..., help="Path to the bag file"),
    output_dir: str = typer.Option("./export", "--output", "-o", help="Output directory"),
    fmt: str = typer.Option("parquet", "--format", "-f", help="Output format: parquet or json"),
    topics: Optional[str] = typer.Option(
        None, "--topics", "-t", help="Comma-separated topic names (default: all)"
    ),
    ai: bool = typer.Option(
        False, "--ai", help="Enable AI-friendly mode (relative timestamps, normalized)"
    ),
    flatten: bool = typer.Option(
        True, "--flatten/--no-flatten", help="Flatten nested message fields"
    ),
) -> None:
    """Export bag data to AI/analytics-friendly formats.

    Outputs one file per topic in JSON or Parquet format,
    with optional timestamp normalization and field flattening.
    """
    from bagx.export import export_bag, print_export_summary

    if fmt not in ("parquet", "json"):
        console.print(f"[red]Unsupported format: {fmt}. Use 'parquet' or 'json'.[/red]")
        raise typer.Exit(1)

    topic_list = None
    if topics:
        topic_list = [t.strip() for t in topics.split(",")]

    try:
        output_files = export_bag(
            bag, output_dir, fmt=fmt, topics=topic_list, flatten=flatten, ai_mode=ai
        )
        print_export_summary(output_files, console)
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error exporting bag: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def anomaly(
    bag: str = typer.Argument(..., help="Path to the bag file"),
    topic: Optional[str] = typer.Option(None, "--topic", "-t", help="Analyze only this topic"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
) -> None:
    """Detect anomalies and outliers in sensor data.

    Finds GNSS position jumps, IMU spikes, message rate gaps,
    and other anomalous events in the bag file.
    """
    from bagx.anomaly import detect_anomalies, print_anomaly_report

    topics = [topic] if topic else None

    try:
        report = detect_anomalies(bag, topics=topics)
        print_anomaly_report(report, console)
        if json_output:
            with open(json_output, "w") as f:
                json.dump(report.to_dict(), f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error detecting anomalies: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def scenario(
    bag: str = typer.Argument(..., help="Path to the bag file"),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
) -> None:
    """Identify and extract dangerous or interesting scenarios.

    Detects GNSS loss, sensor dropouts, high dynamics events,
    and sync degradation periods.
    """
    from bagx.scenario import detect_scenarios, print_scenario_report

    try:
        report = detect_scenarios(bag)
        print_scenario_report(report, console)
        if json_output:
            with open(json_output, "w") as f:
                json.dump(report.to_dict(), f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error detecting scenarios: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def ask(
    bag: str = typer.Argument(..., help="Path to the bag file (.db3 or directory)"),
    question: str = typer.Argument(..., help="Natural language question about the bag"),
    provider: str = typer.Option(
        "anthropic", "--provider", "-p", help="LLM provider: 'anthropic' or 'openai'"
    ),
) -> None:
    """Ask a natural language question about a bag file, answered by an LLM.

    Gathers bag context (summary, eval, message samples) and sends it
    along with your question to an LLM for analysis.
    """
    from bagx.ask import ask_bag

    try:
        answer = ask_bag(bag, question, provider=provider)
        console.print(f"\n{answer}\n")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except EnvironmentError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def scene(
    bag: str = typer.Argument(..., help="Path to the bag file"),
    csv_output: Optional[str] = typer.Option(
        None, "--csv", "-c", help="Export scene states to CSV file"
    ),
    json_output: Optional[str] = typer.Option(
        None, "--json", "-j", help="Output JSON report to file"
    ),
    topics: Optional[str] = typer.Option(
        None, "--topics", "-t", help="Comma-separated topic names (default: auto-detect)"
    ),
) -> None:
    """Extract 3D state (position, orientation, velocity) time series.

    Auto-detects topics with scene-relevant message types (PoseStamped,
    Odometry, Imu, NavSatFix, TFMessage) unless specific topics are given.
    """
    from pathlib import Path as _Path

    from bagx.scene import export_scene_csv, extract_scene, print_scene_report

    topic_list = None
    if topics:
        topic_list = [t.strip() for t in topics.split(",")]

    try:
        report = extract_scene(bag, topics=topic_list)
        print_scene_report(report, console)

        if csv_output:
            export_scene_csv(report, _Path(csv_output))
            console.print(f"CSV exported to: [cyan]{csv_output}[/cyan]")

        if json_output:
            with open(json_output, "w") as f:
                json.dump(report.to_dict(), f, indent=2)
            console.print(f"JSON report saved to: [cyan]{json_output}[/cyan]")
    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error extracting scene: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def info(
    bag: str = typer.Argument(..., help="Path to the bag file"),
) -> None:
    """Show bag file summary information."""
    from bagx.reader import BagReader

    try:
        reader = BagReader(bag)
        summary = reader.summary()

        console.print(f"\n[bold]Bag Info: [cyan]{summary.path}[/cyan][/bold]")
        console.print(f"  Duration: {summary.duration_sec:.2f} s")
        console.print(f"  Messages: {summary.message_count:,}")
        console.print(f"  Topics:   {len(summary.topics)}")
        console.print()

        from rich.table import Table

        table = Table(show_header=True)
        table.add_column("Topic", style="bold")
        table.add_column("Type")
        table.add_column("Count", justify="right")
        table.add_column("Format")

        for name, info in sorted(summary.topics.items()):
            table.add_row(name, info.type, str(info.count), info.serialization_format)

        console.print(table)
        console.print()

    except FileNotFoundError as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)
    except Exception as e:
        console.print(f"[red]Error reading bag: {e}[/red]")
        raise typer.Exit(1)


def main() -> None:
    app()


if __name__ == "__main__":
    main()
