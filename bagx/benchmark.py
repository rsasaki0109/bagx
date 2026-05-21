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
from bagx.findings import SEVERITY_ORDER, severity_at_least


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
    finding_ids: list[str] = field(default_factory=list)
    worst_severity: str | None = None

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
    worst_severity: str | None = None

    def to_dict(self) -> dict[str, Any]:
        data = {
            "manifest_path": self.manifest_path,
            "suite_name": self.suite_name,
            "total_cases": self.total_cases,
            "passed_cases": self.passed_cases,
            "failed_cases": self.failed_cases,
            "skipped_cases": self.skipped_cases,
            "worst_severity": self.worst_severity,
            "cases": [case.to_dict() for case in self.cases],
        }
        data.update(report_metadata("benchmark_suite"))
        return data


def _worst_severity(findings: list[dict[str, Any]]) -> str | None:
    """Return the worst severity in a finding list, or None if empty."""
    worst: str | None = None
    worst_rank = -1
    for finding in findings:
        sev = str(finding.get("severity", ""))
        rank = SEVERITY_ORDER.get(sev, -1)
        if rank > worst_rank:
            worst_rank = rank
            worst = sev
    return worst


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

    case_worst_severities = [
        case.worst_severity for case in case_results if case.worst_severity
    ]
    suite_worst = None
    if case_worst_severities:
        suite_worst = max(
            case_worst_severities,
            key=lambda sev: SEVERITY_ORDER.get(sev, -1),
        )

    return BenchmarkSuiteReport(
        manifest_path=str(manifest_file),
        suite_name=suite_name,
        total_cases=len(case_results),
        passed_cases=sum(case.status == "passed" for case in case_results),
        failed_cases=sum(case.status == "failed" for case in case_results),
        skipped_cases=sum(case.status == "skipped" for case in case_results),
        cases=case_results,
        worst_severity=suite_worst,
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
    report_dict = report.to_dict()
    recommendations = report_dict["recommendations"]
    recommendation_text = "\n".join(recommendations)
    findings = report_dict.get("findings", [])
    detected_domains = sorted(detect_domain_names(report.topic_info, report.custom_domains))
    checks = _evaluate_expectations(
        expectations=expectations,
        overall_score=report.overall_score,
        domain_score=report.domain_score,
        detected_domains=detected_domains,
        recommendation_text=recommendation_text,
        findings=findings,
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
        finding_ids=sorted(str(finding.get("id", "")) for finding in findings if finding.get("id")),
        worst_severity=_worst_severity(findings),
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
    findings: list[dict[str, Any]],
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

    for expected in expectations.get("expected_findings", []):
        checks.extend(_evaluate_expected_finding(expected, findings))

    for forbidden in expectations.get("forbidden_findings", []):
        checks.extend(_evaluate_forbidden_finding(forbidden, findings))

    max_severity_map = expectations.get("max_severity") or {}
    if isinstance(max_severity_map, dict) and max_severity_map:
        checks.extend(_evaluate_max_severity(max_severity_map, findings))

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


def _evaluate_expected_finding(
    expected: str | dict[str, Any],
    findings: list[dict[str, Any]],
) -> list[BenchmarkCheck]:
    if isinstance(expected, str):
        finding_id = expected
        expected_severity = None
        expected_topics: list[str] = []
        expected_domain = None
        expected_category = None
    else:
        finding_id = str(expected.get("id", ""))
        expected_severity = expected.get("severity")
        expected_topics = [str(topic) for topic in expected.get("affected_topics", [])]
        expected_domain = expected.get("domain")
        expected_category = expected.get("category")

    matches = [finding for finding in findings if finding.get("id") == finding_id]
    checks = [
        BenchmarkCheck(
            name=f"expected_finding:{finding_id}",
            passed=bool(matches),
            detail="present" if matches else "missing",
        )
    ]
    if not matches:
        return checks

    finding = matches[0]
    if expected_severity is not None:
        actual = finding.get("severity")
        checks.append(BenchmarkCheck(
            name=f"expected_finding_severity:{finding_id}",
            passed=actual == expected_severity,
            detail=f"severity={actual}, expected={expected_severity}",
        ))
    if expected_domain is not None:
        actual = finding.get("domain")
        checks.append(BenchmarkCheck(
            name=f"expected_finding_domain:{finding_id}",
            passed=actual == expected_domain,
            detail=f"domain={actual}, expected={expected_domain}",
        ))
    if expected_category is not None:
        actual = finding.get("category")
        checks.append(BenchmarkCheck(
            name=f"expected_finding_category:{finding_id}",
            passed=actual == expected_category,
            detail=f"category={actual}, expected={expected_category}",
        ))
    if expected_topics:
        actual_topics = set(str(topic) for topic in finding.get("affected_topics", []))
        missing = [topic for topic in expected_topics if topic not in actual_topics]
        checks.append(BenchmarkCheck(
            name=f"expected_finding_topics:{finding_id}",
            passed=not missing,
            detail="present" if not missing else f"missing={missing}",
        ))

    return checks


def _evaluate_forbidden_finding(
    forbidden: str | dict[str, Any],
    findings: list[dict[str, Any]],
) -> list[BenchmarkCheck]:
    """Fail if a forbidden finding id appears.

    A dict form like {"id": "x", "severity_min": "warning"} only forbids the
    finding when its severity is at or above severity_min — useful when an info
    finding is acceptable but warning+ is not.
    """
    if isinstance(forbidden, str):
        finding_id = forbidden
        severity_min = None
    else:
        finding_id = str(forbidden.get("id", ""))
        severity_min = forbidden.get("severity_min")

    matches = [f for f in findings if f.get("id") == finding_id]
    if severity_min:
        matches = [
            f for f in matches if severity_at_least(str(f.get("severity", "")), severity_min)
        ]

    suffix = f" (severity>={severity_min})" if severity_min else ""
    return [
        BenchmarkCheck(
            name=f"forbidden_finding:{finding_id}{suffix}",
            passed=not matches,
            detail="absent" if not matches else f"present (severity={matches[0].get('severity')})",
        )
    ]


def _evaluate_max_severity(
    max_severity_map: dict[str, str],
    findings: list[dict[str, Any]],
) -> list[BenchmarkCheck]:
    """For each category, fail if any finding exceeds the allowed severity."""
    checks: list[BenchmarkCheck] = []
    for category, ceiling in sorted(max_severity_map.items()):
        ceiling = str(ceiling)
        if ceiling not in SEVERITY_ORDER:
            checks.append(BenchmarkCheck(
                name=f"max_severity:{category}",
                passed=False,
                detail=f"unknown severity {ceiling!r}",
            ))
            continue
        ceiling_rank = SEVERITY_ORDER[ceiling]
        offenders = [
            f for f in findings
            if str(f.get("category")) == category
            and SEVERITY_ORDER.get(str(f.get("severity", "")), -1) > ceiling_rank
        ]
        if offenders:
            worst = max(
                offenders,
                key=lambda f: SEVERITY_ORDER.get(str(f.get("severity", "")), -1),
            )
            detail = (
                f"{worst.get('id')} has severity {worst.get('severity')}, "
                f"max={ceiling}"
            )
            passed = False
        else:
            detail = f"all {category} findings <= {ceiling}"
            passed = True
        checks.append(BenchmarkCheck(
            name=f"max_severity:{category}",
            passed=passed,
            detail=detail,
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
