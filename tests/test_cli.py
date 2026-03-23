"""Tests for bagx.cli module."""

import csv
import json
import os
from pathlib import Path

from typer.testing import CliRunner

from bagx import __version__
from bagx.cli import app

runner = CliRunner()


class TestCliInfo:
    def test_info(self, gnss_bag: Path):
        result = runner.invoke(app, ["info", str(gnss_bag)])
        assert result.exit_code == 0
        assert "/gnss" in result.output
        assert "NavSatFix" in result.output

    def test_info_multi(self, multi_bag: Path):
        result = runner.invoke(app, ["info", str(multi_bag)])
        assert result.exit_code == 0
        assert "/gnss" in result.output
        assert "/imu" in result.output
        assert "/lidar" in result.output

    def test_info_not_found(self):
        result = runner.invoke(app, ["info", "/nonexistent.db3"])
        assert result.exit_code == 1

    def test_version_option(self):
        result = runner.invoke(app, ["--version"])
        assert result.exit_code == 0
        assert f"bagx {__version__}" in result.output

    def test_quiet_sets_logging_environment(self, monkeypatch):
        monkeypatch.delenv("BAGX_SUPPRESS_NATIVE_STDERR", raising=False)
        monkeypatch.delenv("RCUTILS_LOGGING_MIN_SEVERITY", raising=False)

        result = runner.invoke(app, ["--quiet", "info", "/nonexistent.db3"])

        assert result.exit_code == 1
        assert os.environ["BAGX_SUPPRESS_NATIVE_STDERR"] == "1"
        assert os.environ["RCUTILS_LOGGING_MIN_SEVERITY"] == "ERROR"

    def test_verbose_clears_stderr_suppression(self, monkeypatch):
        monkeypatch.setenv("BAGX_SUPPRESS_NATIVE_STDERR", "1")
        monkeypatch.delenv("RCUTILS_LOGGING_MIN_SEVERITY", raising=False)

        result = runner.invoke(app, ["--verbose", "info", "/nonexistent.db3"])

        assert result.exit_code == 1
        assert "BAGX_SUPPRESS_NATIVE_STDERR" not in os.environ
        assert os.environ["RCUTILS_LOGGING_MIN_SEVERITY"] == "INFO"


class TestCliEval:
    def test_eval(self, gnss_bag: Path):
        result = runner.invoke(app, ["eval", str(gnss_bag)])
        assert result.exit_code == 0
        assert "GNSS" in result.output
        assert "Score" in result.output

    def test_eval_json(self, gnss_bag: Path, tmp_path: Path):
        import json

        json_path = tmp_path / "eval.json"
        result = runner.invoke(app, ["eval", str(gnss_bag), "--json", str(json_path)])
        assert result.exit_code == 0

        with open(json_path) as f:
            data = json.load(f)
        assert "gnss" in data

    def test_eval_not_found(self):
        result = runner.invoke(app, ["eval", "/nonexistent.db3"])
        assert result.exit_code == 1

    def test_eval_nav2_domain_output(self, nav2_bag: Path):
        result = runner.invoke(app, ["eval", str(nav2_bag)])
        assert result.exit_code == 0
        assert "Nav2 topics detected" in result.output
        assert "/robot/odom" in result.output

    def test_eval_autoware_domain_output(self, autoware_bag: Path):
        result = runner.invoke(app, ["eval", str(autoware_bag)])
        assert result.exit_code == 0
        assert "Autoware topics detected" in result.output
        assert "/sensing/lidar/top/pointcloud_raw_ex" in result.output

    def test_eval_moveit_domain_output(self, moveit_bag: Path):
        result = runner.invoke(app, ["eval", str(moveit_bag)])
        assert result.exit_code == 0
        assert "MoveIt topics detected" in result.output
        assert "/fr3/joint_states" in result.output


class TestCliCompare:
    def test_compare(self, gnss_bag: Path, gnss_bag_degraded: Path):
        result = runner.invoke(app, ["compare", str(gnss_bag), str(gnss_bag_degraded)])
        assert result.exit_code == 0
        assert "Comparison" in result.output

    def test_compare_json(self, gnss_bag: Path, gnss_bag_degraded: Path, tmp_path: Path):
        json_path = tmp_path / "compare.json"
        result = runner.invoke(
            app,
            ["compare", str(gnss_bag), str(gnss_bag_degraded), "--json", str(json_path)],
        )
        assert result.exit_code == 0

        with open(json_path) as f:
            data = json.load(f)
        assert "items" in data
        assert "winner" in data

    def test_compare_not_found(self):
        result = runner.invoke(app, ["compare", "/a.db3", "/b.db3"])
        assert result.exit_code == 1


class TestCliSync:
    def test_sync(self, multi_bag: Path):
        result = runner.invoke(app, ["sync", str(multi_bag), "/gnss", "/imu"])
        assert result.exit_code == 0
        assert "Sync" in result.output

    def test_sync_json(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "sync.json"
        result = runner.invoke(
            app,
            ["sync", str(multi_bag), "/gnss", "/imu", "--json", str(json_path)],
        )
        assert result.exit_code == 0

        with open(json_path) as f:
            data = json.load(f)
        assert "pairs" in data
        assert data["pairs"][0]["topic_a"] == "/gnss"

    def test_sync_not_found(self):
        result = runner.invoke(app, ["sync", "/nonexistent.db3", "/a", "/b"])
        assert result.exit_code == 1


class TestCliExport:
    def test_export_parquet(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "export"
        result = runner.invoke(app, ["export", str(gnss_bag), "--output", str(out_dir)])
        assert result.exit_code == 0
        assert "Exported" in result.output

    def test_export_json(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "export"
        result = runner.invoke(app, ["export", str(gnss_bag), "-o", str(out_dir), "-f", "json"])
        assert result.exit_code == 0

    def test_export_ai_mode(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "export"
        result = runner.invoke(app, ["export", str(gnss_bag), "-o", str(out_dir), "--ai"])
        assert result.exit_code == 0

    def test_export_invalid_format(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "export"
        result = runner.invoke(app, ["export", str(gnss_bag), "-o", str(out_dir), "-f", "csv"])
        assert result.exit_code == 1


class TestCliAnomalyScenarioScene:
    def test_anomaly_json(self, gnss_bag: Path, tmp_path: Path):
        json_path = tmp_path / "anomaly.json"
        result = runner.invoke(app, ["anomaly", str(gnss_bag), "--json", str(json_path)])
        assert result.exit_code == 0

        with open(json_path) as f:
            data = json.load(f)
        assert "anomalies" in data
        assert "total_anomalies" in data

    def test_anomaly_topic_filter(self, gnss_bag: Path):
        result = runner.invoke(app, ["anomaly", str(gnss_bag), "--topic", "/gnss"])
        assert result.exit_code == 0
        assert "/gnss" in result.output

    def test_scenario_json(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scenario.json"
        result = runner.invoke(app, ["scenario", str(multi_bag), "--json", str(json_path)])
        assert result.exit_code == 0

        with open(json_path) as f:
            data = json.load(f)
        assert "scenarios" in data
        assert "total_scenarios" in data

    def test_scene_csv_and_json(self, nav2_bag: Path, tmp_path: Path):
        csv_path = tmp_path / "scene.csv"
        json_path = tmp_path / "scene.json"
        result = runner.invoke(
            app,
            [
                "scene",
                str(nav2_bag),
                "--topics",
                "/robot/odom,/robot/pose",
                "--csv",
                str(csv_path),
                "--json",
                str(json_path),
            ],
        )
        assert result.exit_code == 0
        assert csv_path.exists()
        assert json_path.exists()

        with open(json_path) as f:
            data = json.load(f)
        assert data["state_count"] > 0
        assert "/robot/odom" in data["sources"]


class TestCliBatch:
    def test_batch_eval_csv(self, tmp_path: Path, gnss_bag: Path, imu_bag: Path):
        csv_path = tmp_path / "batch_eval.csv"
        result = runner.invoke(
            app,
            ["batch", "eval", str(gnss_bag), str(imu_bag), "--csv", str(csv_path)],
        )
        assert result.exit_code == 0
        assert "Batch Evaluation Summary" in result.output
        assert csv_path.exists()

        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == 2

    def test_batch_anomaly_json(self, tmp_path: Path, gnss_bag: Path, imu_bag: Path):
        json_path = tmp_path / "batch_anomaly.json"
        result = runner.invoke(
            app,
            ["batch", "anomaly", str(gnss_bag), str(imu_bag), "--json", str(json_path)],
        )
        assert result.exit_code == 0
        assert "Batch Anomaly Detection" in result.output
        assert json_path.exists()

        with open(json_path) as f:
            data = json.load(f)
        assert data["total_bags"] == 2
        assert "bags" in data


class TestCliAsk:
    def test_ask_success(self, gnss_bag: Path, monkeypatch):
        monkeypatch.setattr("bagx.ask.ask_bag", lambda bag, question, provider="anthropic": "stub answer")

        result = runner.invoke(app, ["ask", str(gnss_bag), "What is in this bag?"])

        assert result.exit_code == 0
        assert "stub answer" in result.output

    def test_ask_environment_error(self, gnss_bag: Path, monkeypatch):
        def _raise_env_error(*_args, **_kwargs):
            raise EnvironmentError("missing API key")

        monkeypatch.setattr("bagx.ask.ask_bag", _raise_env_error)

        result = runner.invoke(app, ["ask", str(gnss_bag), "What is in this bag?"])

        assert result.exit_code == 1
        assert "missing API key" in result.output
