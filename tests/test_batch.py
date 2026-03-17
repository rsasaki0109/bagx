"""Tests for bagx.batch module."""

from __future__ import annotations

import csv
import json
from pathlib import Path


from bagx.batch import batch_anomaly, batch_eval, resolve_bag_paths
from tests.conftest import (
    _create_db3,
    build_navsatfix_cdr,
)


def _make_gnss_bag(path: Path, n_messages: int = 20) -> Path:
    """Helper to create a small GNSS bag at a given path."""
    topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    for i in range(n_messages):
        ts = base_ns + i * 100_000_000
        data = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=0,
            latitude=35.6812,
            longitude=139.7671,
            altitude=40.0,
        )
        messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

    return _create_db3(path, topics, messages)


class TestResolveBagPaths:
    def test_resolve_single_file(self, tmp_path: Path):
        bag = _make_gnss_bag(tmp_path / "test.db3")
        result = resolve_bag_paths([str(bag)])
        assert len(result) == 1
        assert result[0] == bag.resolve()

    def test_resolve_directory(self, tmp_path: Path):
        subdir = tmp_path / "bags"
        subdir.mkdir()
        _make_gnss_bag(subdir / "a.db3")
        _make_gnss_bag(subdir / "b.db3")
        # Also create a non-bag file that should be ignored
        (subdir / "notes.txt").write_text("not a bag")

        result = resolve_bag_paths([str(subdir)])
        assert len(result) == 2
        resolved = {p.name for p in result}
        assert resolved == {"a.db3", "b.db3"}

    def test_resolve_glob_pattern(self, tmp_path: Path):
        _make_gnss_bag(tmp_path / "first.db3")
        _make_gnss_bag(tmp_path / "second.db3")
        _make_gnss_bag(tmp_path / "other.mcap")  # won't match *.db3

        pattern = str(tmp_path / "*.db3")
        result = resolve_bag_paths([pattern])
        assert len(result) == 2

    def test_resolve_nested_directory(self, tmp_path: Path):
        nested = tmp_path / "a" / "b"
        nested.mkdir(parents=True)
        _make_gnss_bag(nested / "deep.db3")

        result = resolve_bag_paths([str(tmp_path)])
        assert len(result) == 1
        assert result[0].name == "deep.db3"

    def test_resolve_deduplication(self, tmp_path: Path):
        bag = _make_gnss_bag(tmp_path / "dup.db3")
        # Pass the same file twice
        result = resolve_bag_paths([str(bag), str(bag)])
        assert len(result) == 1

    def test_resolve_empty(self, tmp_path: Path):
        result = resolve_bag_paths([str(tmp_path / "nonexistent*.db3")])
        assert result == []

    def test_resolve_mcap_extension(self, tmp_path: Path):
        # .mcap files should also be found
        bag = _make_gnss_bag(tmp_path / "test.mcap")
        result = resolve_bag_paths([str(bag)])
        assert len(result) == 1


class TestBatchEval:
    def test_batch_eval_multiple_bags(self, tmp_path: Path):
        bag1 = _make_gnss_bag(tmp_path / "a.db3")
        bag2 = _make_gnss_bag(tmp_path / "b.db3")

        reports = batch_eval([str(bag1), str(bag2)])
        assert len(reports) == 2
        for r in reports:
            assert r.overall_score > 0
            assert r.gnss is not None

    def test_batch_eval_with_directory(self, tmp_path: Path):
        subdir = tmp_path / "recordings"
        subdir.mkdir()
        _make_gnss_bag(subdir / "rec1.db3")
        _make_gnss_bag(subdir / "rec2.db3")

        reports = batch_eval([str(subdir)])
        assert len(reports) == 2

    def test_batch_eval_empty(self, tmp_path: Path):
        reports = batch_eval([str(tmp_path / "nothing*.db3")])
        assert reports == []

    def test_batch_eval_csv_output(self, tmp_path: Path):
        bag1 = _make_gnss_bag(tmp_path / "a.db3")
        bag2 = _make_gnss_bag(tmp_path / "b.db3")
        csv_path = str(tmp_path / "summary.csv")

        reports = batch_eval([str(bag1), str(bag2)], output_csv=csv_path)

        assert len(reports) == 2
        assert Path(csv_path).exists()

        with open(csv_path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        assert len(rows) == 2
        # Verify CSV columns
        expected_cols = {
            "bag_path", "duration_sec", "message_count",
            "gnss_score", "imu_score", "sync_score", "overall_score",
        }
        assert set(rows[0].keys()) == expected_cols

        # Scores should be non-empty for GNSS bags
        for row in rows:
            assert row["gnss_score"] != ""
            assert float(row["overall_score"]) > 0

    def test_batch_eval_with_existing_fixtures(self, gnss_bag: Path, imu_bag: Path):
        """Test using conftest fixtures directly."""
        reports = batch_eval([str(gnss_bag), str(imu_bag)])
        assert len(reports) == 2

        # First should have GNSS, second should have IMU
        gnss_report = next(r for r in reports if r.gnss is not None)
        imu_report = next(r for r in reports if r.imu is not None)
        assert gnss_report.gnss.fix_rate > 0
        assert imu_report.imu.total_messages > 0


class TestBatchAnomaly:
    def test_batch_anomaly_multiple_bags(self, tmp_path: Path):
        bag1 = _make_gnss_bag(tmp_path / "a.db3")
        bag2 = _make_gnss_bag(tmp_path / "b.db3")

        result = batch_anomaly([str(bag1), str(bag2)])
        assert result["total_bags"] == 2
        assert len(result["bags"]) == 2
        assert "total_anomalies" in result

    def test_batch_anomaly_json_output(self, tmp_path: Path):
        bag = _make_gnss_bag(tmp_path / "test.db3")
        json_path = str(tmp_path / "anomalies.json")

        batch_anomaly([str(bag)], output_json=json_path)

        assert Path(json_path).exists()
        with open(json_path) as f:
            data = json.load(f)
        assert data["total_bags"] == 1

    def test_batch_anomaly_empty(self, tmp_path: Path):
        result = batch_anomaly([str(tmp_path / "nothing*.db3")])
        assert result["total_bags"] == 0
        assert result["total_anomalies"] == 0
        assert result["bags"] == []
