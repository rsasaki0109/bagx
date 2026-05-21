"""Tests for benchmark suite execution."""

from __future__ import annotations

import json
from pathlib import Path

from typer.testing import CliRunner

from bagx.benchmark import run_benchmark_suite
from bagx.cli import app
from bagx.contracts import REPORT_SCHEMA_VERSION

runner = CliRunner()


def _write_manifest(tmp_path: Path, cases: list[dict]) -> Path:
    manifest_path = tmp_path / "benchmark.json"
    manifest_path.write_text(json.dumps({"suite_name": "test-suite", "cases": cases}, indent=2))
    return manifest_path


class TestBenchmarkSuite:
    def test_run_benchmark_suite_pass_and_skip(self, tmp_path: Path, nav2_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "nav2-pass",
                    "bag_path": str(nav2_bag),
                    "expect": {
                        "required_domains": ["Nav2"],
                        "required_recommendations": ["Nav2 topics detected"],
                        "expected_findings": [
                            {
                                "id": "nav2.detected",
                                "severity": "info",
                                "domain": "nav2",
                                "category": "domain_detection",
                            }
                        ],
                        "min_topic_rates": {"/robot/scan": 10},
                    },
                },
                {
                    "name": "missing-bag",
                    "bag_path": str(tmp_path / "missing.db3"),
                    "expect": {"min_overall_score": 1},
                },
            ],
        )

        report = run_benchmark_suite(str(manifest_path))

        assert report.passed_cases == 1
        assert report.failed_cases == 0
        assert report.skipped_cases == 1
        assert report.cases[0].status == "passed"
        assert "nav2.detected" in report.cases[0].finding_ids
        assert report.cases[1].status == "skipped"

    def test_run_benchmark_suite_detects_failed_expectation(self, tmp_path: Path, gnss_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "bad-expectation",
                    "bag_path": str(gnss_bag),
                    "expect": {
                        "required_domains": ["Nav2"],
                        "forbidden_recommendations": ["No GNSS data"],
                    },
                }
            ],
        )

        report = run_benchmark_suite(str(manifest_path), fail_on_missing=True)

        assert report.failed_cases == 1
        assert report.cases[0].status == "failed"
        assert any(not check.passed for check in report.cases[0].checks)

    def test_run_benchmark_suite_detects_missing_expected_finding(
        self, tmp_path: Path, nav2_bag: Path
    ):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "bad-finding-expectation",
                    "bag_path": str(nav2_bag),
                    "expect": {
                        "expected_findings": [
                            {
                                "id": "nav2.missing_cmd_vel",
                                "severity": "warning",
                                "affected_topics": ["/cmd_vel"],
                            }
                        ]
                    },
                }
            ],
        )

        report = run_benchmark_suite(str(manifest_path))

        assert report.failed_cases == 1
        assert report.cases[0].status == "failed"
        assert any(
            check.name == "expected_finding:nav2.missing_cmd_vel" and not check.passed
            for check in report.cases[0].checks
        )

    def test_run_benchmark_suite_case_filter(self, tmp_path: Path, nav2_bag: Path, gnss_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {"name": "keep", "bag_path": str(nav2_bag), "expect": {"required_domains": ["Nav2"]}},
                {"name": "drop", "bag_path": str(gnss_bag), "expect": {"min_overall_score": 1}},
            ],
        )

        report = run_benchmark_suite(str(manifest_path), selected_cases=["keep"])

        assert report.total_cases == 1
        assert report.cases[0].name == "keep"

    def test_run_benchmark_suite_with_perception_domain(self, tmp_path: Path, perception_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "perception-pass",
                    "bag_path": str(perception_bag),
                    "expect": {
                        "required_domains": ["Perception"],
                        "required_recommendations": [
                            "Perception topics detected",
                            "Camera calibration topics are recorded",
                        ],
                        "forbidden_recommendations": ["No GNSS data", "No IMU data"],
                        "min_domain_score": 90,
                    },
                }
            ],
        )

        report = run_benchmark_suite(str(manifest_path))

        assert report.passed_cases == 1
        assert report.cases[0].detected_domains == ["Perception"]

    def test_run_benchmark_suite_with_generic_control_domain(self, tmp_path: Path, generic_control_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "control-pass",
                    "bag_path": str(generic_control_bag),
                    "expect": {
                        "required_domains": ["Control"],
                        "required_recommendations": [
                            "Planning/control topics detected",
                            "planner → command onset",
                            "Action result (/mission/result) recorded",
                            "Service event (/planner/compute_path/_service_event) recorded",
                        ],
                        "forbidden_recommendations": ["Nav2 topics detected"],
                        "min_domain_score": 90,
                    },
                }
            ],
        )

        report = run_benchmark_suite(str(manifest_path))

        assert report.passed_cases == 1
        assert report.cases[0].detected_domains == ["Control"]

    def test_run_benchmark_suite_with_manifest_rules_path(
        self,
        tmp_path: Path,
        custom_rule_bag: Path,
        custom_rules_file: Path,
    ):
        manifest_path = tmp_path / "benchmark.json"
        manifest_path.write_text(json.dumps({
            "suite_name": "custom-rules-suite",
            "rules_path": str(custom_rules_file),
            "cases": [
                {
                    "name": "warehouse-bot",
                    "bag_path": str(custom_rule_bag),
                    "expect": {
                        "required_domains": ["WarehouseBot"],
                        "required_recommendations": ["WarehouseBot custom rules matched"],
                        "forbidden_recommendations": ["No GNSS data", "No IMU data"],
                        "min_domain_score": 90,
                    },
                }
            ],
        }, indent=2))

        report = run_benchmark_suite(str(manifest_path))

        assert report.passed_cases == 1
        assert "WarehouseBot" in report.cases[0].detected_domains

    def test_run_benchmark_suite_with_builtin_plugin_name(self, tmp_path: Path, custom_rule_bag: Path):
        manifest_path = tmp_path / "benchmark.json"
        manifest_path.write_text(json.dumps({
            "suite_name": "custom-rules-suite",
            "rules_path": "warehouse_bot",
            "cases": [
                {
                    "name": "warehouse-bot",
                    "bag_path": str(custom_rule_bag),
                    "expect": {
                        "required_domains": ["WarehouseBot"],
                        "required_recommendations": ["WarehouseBot custom rules matched"],
                    },
                }
            ],
        }, indent=2))

        report = run_benchmark_suite(str(manifest_path))

        assert report.passed_cases == 1
        assert "WarehouseBot" in report.cases[0].detected_domains


class TestBenchmarkCli:
    def test_benchmark_cli_json_output(self, tmp_path: Path, nav2_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "nav2-pass",
                    "bag_path": str(nav2_bag),
                    "expect": {"required_domains": ["Nav2"]},
                }
            ],
        )
        json_path = tmp_path / "benchmark-report.json"

        result = runner.invoke(app, ["benchmark", str(manifest_path), "--json", str(json_path)])

        assert result.exit_code == 0
        with open(json_path) as f:
            data = json.load(f)
        assert data["schema_version"] == REPORT_SCHEMA_VERSION
        assert data["report_type"] == "benchmark_suite"
        assert data["passed_cases"] == 1

    def test_benchmark_cli_exit_code_on_failure(self, tmp_path: Path, gnss_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "bad-expectation",
                    "bag_path": str(gnss_bag),
                    "expect": {"required_domains": ["MoveIt"]},
                }
            ],
        )

        result = runner.invoke(app, ["benchmark", str(manifest_path)])

        assert result.exit_code == 1
        assert "failed" in result.output


class TestBenchmarkSeverityGate:
    def test_forbidden_findings_fails_when_present(self, tmp_path: Path, nav2_bag: Path):
        # nav2_bag emits nav2.detected, so forbidding it should fail.
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "nav2-no-detected",
                    "bag_path": str(nav2_bag),
                    "expect": {"forbidden_findings": ["nav2.detected"]},
                }
            ],
        )
        report = run_benchmark_suite(str(manifest_path))
        case = report.cases[0]
        assert case.status == "failed"
        offender = next(
            c for c in case.checks
            if c.name.startswith("forbidden_finding:nav2.detected")
        )
        assert not offender.passed
        assert "present" in offender.detail

    def test_forbidden_findings_passes_when_absent(self, tmp_path: Path, nav2_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "nav2-no-fake",
                    "bag_path": str(nav2_bag),
                    "expect": {"forbidden_findings": ["fake.id.that.does.not.exist"]},
                }
            ],
        )
        report = run_benchmark_suite(str(manifest_path))
        case = report.cases[0]
        assert case.status == "passed"
        assert all(c.passed for c in case.checks)

    def test_forbidden_findings_with_severity_min(self, tmp_path: Path, multi_bag: Path):
        # multi_bag emits sync.delay.high at severity=warning.
        # severity_min=error means warning is still acceptable.
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "ok-at-error",
                    "bag_path": str(multi_bag),
                    "expect": {
                        "forbidden_findings": [
                            {"id": "sync.delay.high", "severity_min": "error"}
                        ]
                    },
                }
            ],
        )
        report = run_benchmark_suite(str(manifest_path))
        assert report.cases[0].status == "passed"

    def test_max_severity_per_category(self, tmp_path: Path, multi_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "sync-quality-warning-not-allowed",
                    "bag_path": str(multi_bag),
                    "expect": {"max_severity": {"sync_quality": "info"}},
                }
            ],
        )
        report = run_benchmark_suite(str(manifest_path))
        case = report.cases[0]
        # sync.delay.high is warning, exceeds info -> fail
        assert case.status == "failed"
        check = next(c for c in case.checks if c.name == "max_severity:sync_quality")
        assert not check.passed
        assert "sync.delay.high" in check.detail

    def test_max_severity_passes_when_within_ceiling(self, tmp_path: Path, multi_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {
                    "name": "sync-quality-warning-allowed",
                    "bag_path": str(multi_bag),
                    "expect": {"max_severity": {"sync_quality": "warning"}},
                }
            ],
        )
        report = run_benchmark_suite(str(manifest_path))
        assert report.cases[0].status == "passed"

    def test_worst_severity_aggregation(self, tmp_path: Path, multi_bag: Path, nav2_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [
                {"name": "multi", "bag_path": str(multi_bag), "expect": {}},
                {"name": "nav2", "bag_path": str(nav2_bag), "expect": {}},
            ],
        )
        report = run_benchmark_suite(str(manifest_path))
        # multi_bag has sync.delay.high (warning); nav2_bag has missing-* warnings
        assert report.worst_severity == "warning"
        assert report.cases[0].worst_severity in {"info", "warning"}

    def test_cli_exit_on_warning(self, tmp_path: Path, multi_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [{"name": "multi", "bag_path": str(multi_bag), "expect": {}}],
        )
        result = runner.invoke(
            app, ["benchmark", str(manifest_path), "--exit-on", "warning"]
        )
        assert result.exit_code == 1
        assert "worst finding severity" in result.output

    def test_cli_exit_on_error_passes_warning_bag(self, tmp_path: Path, multi_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [{"name": "multi", "bag_path": str(multi_bag), "expect": {}}],
        )
        result = runner.invoke(
            app, ["benchmark", str(manifest_path), "--exit-on", "error"]
        )
        # multi_bag's worst is warning, which is below error threshold
        assert result.exit_code == 0

    def test_cli_exit_on_invalid_severity(self, tmp_path: Path, nav2_bag: Path):
        manifest_path = _write_manifest(
            tmp_path,
            [{"name": "nav2", "bag_path": str(nav2_bag), "expect": {}}],
        )
        result = runner.invoke(
            app, ["benchmark", str(manifest_path), "--exit-on", "fatal"]
        )
        assert result.exit_code == 2
        assert "--exit-on" in result.output
