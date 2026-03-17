"""Tests for bagx.cli module."""

from pathlib import Path

import pytest
from typer.testing import CliRunner

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


class TestCliCompare:
    def test_compare(self, gnss_bag: Path, gnss_bag_degraded: Path):
        result = runner.invoke(app, ["compare", str(gnss_bag), str(gnss_bag_degraded)])
        assert result.exit_code == 0
        assert "Comparison" in result.output

    def test_compare_not_found(self):
        result = runner.invoke(app, ["compare", "/a.db3", "/b.db3"])
        assert result.exit_code == 1


class TestCliSync:
    def test_sync(self, multi_bag: Path):
        result = runner.invoke(app, ["sync", str(multi_bag), "/gnss", "/imu"])
        assert result.exit_code == 0
        assert "Sync" in result.output

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
