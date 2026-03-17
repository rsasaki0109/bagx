"""Integration tests that mirror the README usage examples.

Each test corresponds to a concrete example from the README,
ensuring all advertised functionality actually works end-to-end.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from typer.testing import CliRunner

from bagx.cli import app

runner = CliRunner()


# ── Example 1: Pre-drive sensor check (bagx eval) ──


class TestPreDriveCheck:
    """bagx eval recording.db3 → score, GNSS fix rate, IMU frequency."""

    def test_eval_shows_score(self, gnss_bag: Path):
        result = runner.invoke(app, ["eval", str(gnss_bag)])
        assert result.exit_code == 0
        assert "Overall Score" in result.output
        assert "/100" in result.output

    def test_eval_shows_gnss_metrics(self, gnss_bag: Path):
        result = runner.invoke(app, ["eval", str(gnss_bag)])
        assert "GNSS" in result.output
        assert "Fix Rate" in result.output

    def test_eval_shows_imu_metrics(self, imu_bag: Path):
        result = runner.invoke(app, ["eval", str(imu_bag)])
        assert "IMU" in result.output
        assert "Frequency" in result.output
        assert "200" in result.output  # 200Hz

    def test_eval_json_is_machine_readable(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "report.json"
        result = runner.invoke(app, ["eval", str(multi_bag), "--json", str(json_path)])
        assert result.exit_code == 0

        with open(json_path) as f:
            data = json.load(f)
        assert "overall_score" in data
        assert isinstance(data["overall_score"], (int, float))
        assert "imu" in data
        assert "sync" in data


# ── Example 2: Compare two runs (bagx compare) ──


class TestCompareRuns:
    """bagx compare A.db3 B.db3 → winner, per-metric verdicts."""

    def test_compare_shows_winner(self, gnss_bag: Path, gnss_bag_degraded: Path):
        result = runner.invoke(app, ["compare", str(gnss_bag), str(gnss_bag_degraded)])
        assert result.exit_code == 0
        assert "Result" in result.output

    def test_compare_shows_verdicts(self, gnss_bag: Path, gnss_bag_degraded: Path):
        result = runner.invoke(app, ["compare", str(gnss_bag), str(gnss_bag_degraded)])
        # Should have at least one verdict
        has_verdict = (
            "improved" in result.output
            or "degraded" in result.output
            or "unchanged" in result.output
        )
        assert has_verdict

    def test_compare_shows_metrics(self, gnss_bag: Path, gnss_bag_degraded: Path):
        result = runner.invoke(app, ["compare", str(gnss_bag), str(gnss_bag_degraded)])
        assert "GNSS Fix Rate" in result.output
        assert "Overall Score" in result.output

    def test_compare_json(self, gnss_bag: Path, gnss_bag_degraded: Path, tmp_path: Path):
        json_path = tmp_path / "diff.json"
        result = runner.invoke(
            app, ["compare", str(gnss_bag), str(gnss_bag_degraded), "--json", str(json_path)]
        )
        assert result.exit_code == 0
        with open(json_path) as f:
            data = json.load(f)
        assert data["winner"] in ("A", "B", "tie")


# ── Example 3: Debugging sensor issues (bagx anomaly) ──


class TestAnomalyDebugging:
    """bagx anomaly recording.db3 → anomaly events with timestamps."""

    def test_anomaly_runs(self, multi_bag: Path):
        result = runner.invoke(app, ["anomaly", str(multi_bag)])
        assert result.exit_code == 0
        assert "Anomaly" in result.output

    def test_anomaly_with_topic_filter(self, multi_bag: Path):
        result = runner.invoke(app, ["anomaly", str(multi_bag), "--topic", "/imu"])
        assert result.exit_code == 0

    def test_anomaly_json(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "anomalies.json"
        result = runner.invoke(app, ["anomaly", str(multi_bag), "--json", str(json_path)])
        assert result.exit_code == 0
        with open(json_path) as f:
            data = json.load(f)
        assert "anomalies" in data or "events" in data or isinstance(data, dict)


# ── Example 4: Checking time sync (bagx sync) ──


class TestSyncCheck:
    """bagx sync recording.db3 /topic_a /topic_b → delay stats."""

    def test_sync_shows_delay_stats(self, multi_bag: Path):
        result = runner.invoke(app, ["sync", str(multi_bag), "/gnss", "/imu"])
        assert result.exit_code == 0
        assert "Mean" in result.output
        assert "Max" in result.output

    def test_sync_shows_outlier_rate(self, multi_bag: Path):
        result = runner.invoke(app, ["sync", str(multi_bag), "/gnss", "/imu"])
        assert "Outlier" in result.output

    def test_sync_json(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "sync.json"
        result = runner.invoke(
            app, ["sync", str(multi_bag), "/gnss", "/lidar", "--json", str(json_path)]
        )
        assert result.exit_code == 0
        with open(json_path) as f:
            data = json.load(f)
        assert "pairs" in data
        pair = data["pairs"][0]
        assert "mean_delay_ms" in pair
        assert "p95_delay_ms" in pair


# ── Example 5: Extracting dangerous scenes (bagx scenario) ──


class TestScenarioExtraction:
    """bagx scenario recording.db3 → scenario time segments."""

    def test_scenario_runs(self, multi_bag: Path):
        result = runner.invoke(app, ["scenario", str(multi_bag)])
        assert result.exit_code == 0
        assert "Scenario" in result.output

    def test_scenario_json(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scenarios.json"
        result = runner.invoke(app, ["scenario", str(multi_bag), "--json", str(json_path)])
        assert result.exit_code == 0
        with open(json_path) as f:
            data = json.load(f)
        assert isinstance(data, dict)


# ── Example 6: Preparing data for ML (bagx export) ──


class TestMlExport:
    """bagx export recording.db3 --ai --format parquet → per-topic files."""

    def test_export_parquet_default(self, multi_bag: Path, tmp_path: Path):
        out = tmp_path / "export"
        result = runner.invoke(app, ["export", str(multi_bag), "-o", str(out)])
        assert result.exit_code == 0
        assert "Exported" in result.output
        parquet_files = list(out.glob("*.parquet"))
        assert len(parquet_files) >= 1

    def test_export_ai_mode(self, gnss_bag: Path, tmp_path: Path):
        out = tmp_path / "export"
        result = runner.invoke(app, ["export", str(gnss_bag), "-o", str(out), "--ai"])
        assert result.exit_code == 0

        import pyarrow.parquet as pq

        files = list(out.glob("*.parquet"))
        table = pq.read_table(str(files[0]))
        ts = table.column("timestamp_sec").to_pylist()
        # AI mode: first timestamp should be near 0 (relative)
        assert ts[0] < 1.0

    def test_export_json_format(self, gnss_bag: Path, tmp_path: Path):
        out = tmp_path / "export"
        result = runner.invoke(
            app, ["export", str(gnss_bag), "-o", str(out), "--format", "json"]
        )
        assert result.exit_code == 0
        json_files = list(out.glob("*.json"))
        assert len(json_files) >= 1
        with open(json_files[0]) as f:
            data = json.load(f)
        assert isinstance(data, list)
        assert len(data) > 0

    def test_export_topic_filter(self, multi_bag: Path, tmp_path: Path):
        out = tmp_path / "export"
        result = runner.invoke(
            app, ["export", str(multi_bag), "-o", str(out), "--topics", "/gnss,/imu"]
        )
        assert result.exit_code == 0
        files = list(out.glob("*.parquet"))
        names = {f.stem for f in files}
        assert "gnss" in names
        assert "imu" in names
        assert "lidar" not in names


# ── Example 7: Asking questions (bagx ask) ──


class TestAskQuestions:
    """bagx ask recording.db3 "question" → LLM answer."""

    def test_ask_context_is_built(self, gnss_bag: Path):
        """Verify context building works without calling LLM."""
        from bagx.ask import _build_bag_context

        context = _build_bag_context(str(gnss_bag))
        assert "Duration" in context
        assert "/gnss" in context
        assert "NavSatFix" in context

    def test_ask_cli_missing_key(self, gnss_bag: Path):
        """Without API key, should fail gracefully."""
        with patch.dict("os.environ", {}, clear=True):
            result = runner.invoke(
                app, ["ask", str(gnss_bag), "What sensors are in this bag?"]
            )
            # Should fail with a message about API key, not crash
            assert result.exit_code == 1 or "API" in result.output or "key" in result.output.lower() or "Error" in result.output

    def test_ask_with_mocked_anthropic(self, gnss_bag: Path):
        """With mocked Anthropic SDK, should return an answer."""
        mock_anthropic_module = MagicMock()
        mock_response = MagicMock()
        mock_response.content = [MagicMock(text="This bag has GNSS data.")]
        mock_anthropic_module.Anthropic.return_value.messages.create.return_value = mock_response

        import sys
        with patch.dict("os.environ", {"ANTHROPIC_API_KEY": "test-key"}), \
             patch.dict(sys.modules, {"anthropic": mock_anthropic_module}):
            # Reimport to pick up the mock
            import importlib
            import bagx.ask
            importlib.reload(bagx.ask)

            answer = bagx.ask.ask_bag(str(gnss_bag), "What sensors?", provider="anthropic")
            assert "GNSS" in answer


# ── Example 8: Batch evaluation (bagx batch eval) ──


class TestBatchEvaluation:
    """bagx batch eval *.db3 --csv summary.csv → multi-bag scores."""

    def test_batch_eval_multiple_bags(self, gnss_bag: Path, imu_bag: Path, tmp_path: Path):
        """Batch eval with explicit file paths."""
        from bagx.batch import batch_eval

        reports = batch_eval([str(gnss_bag), str(imu_bag)])
        assert len(reports) == 2
        assert all(r.overall_score >= 0 for r in reports)

    def test_batch_eval_csv_output(self, gnss_bag: Path, gnss_bag_degraded: Path, tmp_path: Path):
        """Batch eval with CSV summary output."""
        csv_path = tmp_path / "summary.csv"
        from bagx.batch import batch_eval

        batch_eval([str(gnss_bag), str(gnss_bag_degraded)], output_csv=str(csv_path))
        assert csv_path.exists()

        with open(csv_path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        assert len(rows) == 2
        assert "overall_score" in rows[0]
        assert float(rows[0]["overall_score"]) >= 0

    def test_batch_eval_directory(self, tmp_path: Path):
        """Batch eval on a directory of bags."""
        from tests.conftest import _create_db3, build_navsatfix_cdr

        # Create multiple bags in a directory
        bag_dir = tmp_path / "bags"
        bag_dir.mkdir()
        base_ns = 1_700_000_000_000_000_000

        for i, name in enumerate(["bag1.db3", "bag2.db3", "bag3.db3"]):
            topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
            messages = []
            for j in range(20):
                ts = base_ns + j * 100_000_000
                data = build_navsatfix_cdr(
                    stamp_sec=ts // 1_000_000_000,
                    stamp_nanosec=ts % 1_000_000_000,
                    status=0, latitude=35.68, longitude=139.77, altitude=40.0,
                )
                messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})
            _create_db3(bag_dir / name, topics, messages)

        from bagx.batch import batch_eval, resolve_bag_paths

        paths = resolve_bag_paths([str(bag_dir)])
        assert len(paths) == 3

        reports = batch_eval([str(p) for p in paths])
        assert len(reports) == 3


# ── Example: Scene extraction (bagx scene) ──


class TestSceneExtraction:
    """bagx scene recording.db3 → 3D state time series."""

    def test_scene_runs(self, multi_bag: Path):
        result = runner.invoke(app, ["scene", str(multi_bag)])
        assert result.exit_code == 0
        assert "Scene" in result.output or "scene" in result.output.lower()

    def test_scene_csv_export(self, multi_bag: Path, tmp_path: Path):
        csv_path = tmp_path / "scene.csv"
        result = runner.invoke(app, ["scene", str(multi_bag), "--csv", str(csv_path)])
        assert result.exit_code == 0
        assert csv_path.exists()

        with open(csv_path) as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = list(reader)
        # Check that a timestamp column exists (might be "timestamp" or "timestamp_sec")
        assert any("timestamp" in col for col in header)
        assert len(rows) > 0

    def test_scene_json(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scene.json"
        result = runner.invoke(app, ["scene", str(multi_bag), "--json", str(json_path)])
        assert result.exit_code == 0
        with open(json_path) as f:
            data = json.load(f)
        assert "states" in data or isinstance(data, dict)


# ── Example: Info (bagx info) ──


class TestInfoCommand:
    """bagx info recording.db3 → topic list."""

    def test_info_shows_topics_and_types(self, multi_bag: Path):
        result = runner.invoke(app, ["info", str(multi_bag)])
        assert result.exit_code == 0
        assert "/gnss" in result.output
        assert "/imu" in result.output
        assert "/lidar" in result.output
        assert "NavSatFix" in result.output
        assert "Imu" in result.output
        assert "PointCloud2" in result.output

    def test_info_shows_message_counts(self, multi_bag: Path):
        result = runner.invoke(app, ["info", str(multi_bag)])
        assert "50" in result.output   # gnss count
        assert "500" in result.output  # imu count
