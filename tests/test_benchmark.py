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
