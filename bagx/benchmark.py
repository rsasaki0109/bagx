"""Benchmark suite runner for curated rosbag quality checks."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.table import Table

from bagx.contracts import report_metadata
from bagx.eval import detect_domain_names, evaluate_bag


@dataclass
class BenchmarkCheck:
    name: str
    passed: bool
    detail: str


@dataclass
class BenchmarkCaseResult:
    name: str
    bag_path: str
    resolved_bag_path: str
    report_type: str
    status: str
    summary: str
    checks: list[BenchmarkCheck] = field(default_factory=list)
    overall_score: float | None = None
    domain_score: float | None = None
    detected_domains: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data.update(report_metadata("benchmark_case"))
        return data


@dataclass
class BenchmarkSuiteReport:
    manifest_path: str
    suite_name: str
    total_cases: int
    passed_cases: int
    failed_cases: int
    skipped_cases: int
    cases: list[BenchmarkCaseResult] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        data = {
            "manifest_path": self.manifest_path,
            "suite_name": self.suite_name,
            "total_cases": self.total_cases,
            "passed_cases": self.passed_cases,
            "failed_cases": self.failed_cases,
            "skipped_cases": self.skipped_cases,
            "cases": [case.to_dict() for case in self.cases],
        }
        data.update(report_metadata("benchmark_suite"))
        return data


def run_benchmark_suite(
    manifest_path: str,
    *,
    fail_on_missing: bool = False,
    selected_cases: list[str] | None = None,
    rules_path: str | None = None,
) -> BenchmarkSuiteReport:
    """Run a manifest-driven benchmark suite."""
    manifest_file = Path(manifest_path)
    with open(manifest_file) as f:
        manifest = json.load(f)

    suite_name = manifest.get("suite_name", manifest_file.stem)
    manifest_rules_path = manifest.get("rules_path")
    cli_rules_path = None
    if rules_path:
        cli_rules_path = _resolve_rules_spec(str(rules_path), manifest_file.parent)
    raw_cases = manifest.get("cases", [])
    if selected_cases:
        selected = set(selected_cases)
        raw_cases = [case for case in raw_cases if case.get("name") in selected]

    case_results = [
        _run_benchmark_case(
            case,
            manifest_file.parent,
            fail_on_missing=fail_on_missing,
            default_rules_path=cli_rules_path or manifest_rules_path,
        )
        for case in raw_cases
    ]

    return BenchmarkSuiteReport(
        manifest_path=str(manifest_file),
        suite_name=suite_name,
        total_cases=len(case_results),
        passed_cases=sum(case.status == "passed" for case in case_results),
        failed_cases=sum(case.status == "failed" for case in case_results),
        skipped_cases=sum(case.status == "skipped" for case in case_results),
        cases=case_results,
    )


def _run_benchmark_case(
    case: dict[str, Any],
    manifest_dir: Path,
    *,
    fail_on_missing: bool,
    default_rules_path: str | None,
) -> BenchmarkCaseResult:
    name = str(case.get("name", "unnamed"))
    bag_path = str(case.get("bag_path", ""))
    resolved_path = _resolve_manifest_path(bag_path, manifest_dir)
    raw_rules_path = case.get("rules_path", default_rules_path)
    resolved_rules_path = _resolve_rules_spec(str(raw_rules_path), manifest_dir) if raw_rules_path else None
    report_type = str(case.get("report_type", "eval"))

    if report_type != "eval":
        return BenchmarkCaseResult(
            name=name,
            bag_path=bag_path,
            resolved_bag_path=str(resolved_path),
            report_type=report_type,
            status="failed",
            summary=f"Unsupported report_type: {report_type}",
        )

    if not resolved_path.exists():
        status = "failed" if fail_on_missing else "skipped"
        summary = f"Missing bag: {resolved_path}"
        return BenchmarkCaseResult(
            name=name,
            bag_path=bag_path,
            resolved_bag_path=str(resolved_path),
            report_type=report_type,
            status=status,
            summary=summary,
        )

    report = evaluate_bag(str(resolved_path), custom_rules_path=resolved_rules_path)
    expectations = case.get("expect", {})
    recommendations = report.to_dict()["recommendations"]
    recommendation_text = "\n".join(recommendations)
    detected_domains = sorted(detect_domain_names(report.topic_info, report.custom_domains))
    checks = _evaluate_expectations(
        expectations=expectations,
        overall_score=report.overall_score,
        domain_score=report.domain_score,
        detected_domains=detected_domains,
        recommendation_text=recommendation_text,
        topic_info=report.topic_info,
    )
    passed = all(check.passed for check in checks)
    summary = f"{sum(check.passed for check in checks)}/{len(checks)} checks passed" if checks else "No checks configured"

    return BenchmarkCaseResult(
        name=name,
        bag_path=bag_path,
        resolved_bag_path=str(resolved_path),
        report_type=report_type,
        status="passed" if passed else "failed",
        summary=summary,
        checks=checks,
        overall_score=report.overall_score,
        domain_score=report.domain_score,
        detected_domains=detected_domains,
    )


def _resolve_manifest_path(raw_path: str, manifest_dir: Path) -> Path:
    expanded_env = Path(os.path.expandvars(raw_path)).expanduser()
    if expanded_env.is_absolute():
        return expanded_env
    return (manifest_dir / expanded_env).resolve()


def _resolve_rules_spec(raw_path: str, manifest_dir: Path) -> str:
    expanded = Path(os.path.expandvars(raw_path)).expanduser()
    if expanded.is_absolute():
        return str(expanded.resolve())

    manifest_relative = (manifest_dir / expanded).resolve()
    if manifest_relative.exists():
        return str(manifest_relative)

    cwd_relative = expanded.resolve()
    if cwd_relative.exists():
        return str(cwd_relative)

    return raw_path


def _evaluate_expectations(
    *,
    expectations: dict[str, Any],
    overall_score: float,
    domain_score: float | None,
    detected_domains: list[str],
    recommendation_text: str,
    topic_info: dict[str, dict[str, Any]],
) -> list[BenchmarkCheck]:
    checks: list[BenchmarkCheck] = []

    min_overall = expectations.get("min_overall_score")
    if min_overall is not None:
        passed = overall_score >= float(min_overall)
        checks.append(BenchmarkCheck(
            name="min_overall_score",
            passed=passed,
            detail=f"overall={overall_score:.1f}, expected>={float(min_overall):.1f}",
        ))

    max_overall = expectations.get("max_overall_score")
    if max_overall is not None:
        passed = overall_score <= float(max_overall)
        checks.append(BenchmarkCheck(
            name="max_overall_score",
            passed=passed,
            detail=f"overall={overall_score:.1f}, expected<={float(max_overall):.1f}",
        ))

    min_domain = expectations.get("min_domain_score")
    if min_domain is not None:
        actual_domain = domain_score if domain_score is not None else 0.0
        passed = actual_domain >= float(min_domain)
        checks.append(BenchmarkCheck(
            name="min_domain_score",
            passed=passed,
            detail=f"domain={actual_domain:.1f}, expected>={float(min_domain):.1f}",
        ))

    for domain in expectations.get("required_domains", []):
        passed = domain in detected_domains
        checks.append(BenchmarkCheck(
            name=f"required_domain:{domain}",
            passed=passed,
            detail=f"detected={detected_domains}",
        ))

    for fragment in expectations.get("required_recommendations", []):
        passed = fragment in recommendation_text
        checks.append(BenchmarkCheck(
            name=f"required_recommendation:{fragment}",
            passed=passed,
            detail="present" if passed else "missing",
        ))

    for fragment in expectations.get("forbidden_recommendations", []):
        passed = fragment not in recommendation_text
        checks.append(BenchmarkCheck(
            name=f"forbidden_recommendation:{fragment}",
            passed=passed,
            detail="absent" if passed else "present",
        ))

    for topic_name, min_rate in expectations.get("min_topic_rates", {}).items():
        actual_rate = float(topic_info.get(topic_name, {}).get("rate_hz", 0.0) or 0.0)
        passed = actual_rate >= float(min_rate)
        checks.append(BenchmarkCheck(
            name=f"min_topic_rate:{topic_name}",
            passed=passed,
            detail=f"rate={actual_rate:.1f}, expected>={float(min_rate):.1f}",
        ))

    for topic_name, required_type in expectations.get("required_topics", {}).items():
        actual_type = str(topic_info.get(topic_name, {}).get("type", ""))
        passed = actual_type == required_type
        checks.append(BenchmarkCheck(
            name=f"required_topic:{topic_name}",
            passed=passed,
            detail=f"type={actual_type or 'missing'}",
        ))

    return checks


def print_benchmark_report(report: BenchmarkSuiteReport, console: Console | None = None) -> None:
    """Pretty-print benchmark suite results."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Benchmark Suite: [cyan]{report.suite_name}[/cyan][/bold]")
    console.print(f"  Manifest: {report.manifest_path}")
    console.print(
        f"  Cases: {report.total_cases} "
        f"(passed={report.passed_cases}, failed={report.failed_cases}, skipped={report.skipped_cases})"
    )

    if not report.cases:
        console.print("[yellow]No benchmark cases matched.[/yellow]\n")
        return

    table = Table(show_header=True)
    table.add_column("Case", style="bold")
    table.add_column("Status")
    table.add_column("Overall", justify="right")
    table.add_column("Domain", justify="right")
    table.add_column("Domains")
    table.add_column("Summary")

    for case in report.cases:
        status_color = {
            "passed": "green",
            "failed": "red",
            "skipped": "yellow",
        }.get(case.status, "white")
        overall = f"{case.overall_score:.1f}" if case.overall_score is not None else "-"
        domain = f"{case.domain_score:.1f}" if case.domain_score is not None else "-"
        table.add_row(
            case.name,
            f"[{status_color}]{case.status}[/{status_color}]",
            overall,
            domain,
            ", ".join(case.detected_domains) or "-",
            case.summary,
        )

    console.print(table)

    failed_or_skipped = [case for case in report.cases if case.status != "passed"]
    for case in failed_or_skipped:
        console.print(f"\n[bold]{case.name}[/bold] ({case.status})")
        if case.checks:
            for check in case.checks:
                icon = "[green]:heavy_check_mark:[/green]" if check.passed else "[red]:x:[/red]"
                console.print(f"  {icon} {check.name}: {check.detail}")
        else:
            console.print(f"  [yellow]:warning:[/yellow] {case.summary}")

    console.print()
