"""Tests for bagx.scene module."""

import csv
import json
from pathlib import Path

import pytest

from bagx.scene import SceneReport, SceneState, export_scene_csv, extract_scene


class TestAutoDetection:
    """Test auto-detection of relevant topics."""

    def test_detects_gnss_topic(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        assert "/gnss" in report.sources

    def test_detects_imu_topic(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        assert "/imu" in report.sources

    def test_ignores_lidar_topic(self, multi_bag: Path):
        """PointCloud2 is not a scene-relevant type."""
        report = extract_scene(str(multi_bag))
        assert "/lidar" not in report.sources

    def test_state_count_matches_sources(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        total_from_sources = sum(report.sources.values())
        assert len(report.states) == total_from_sources


class TestGnssStates:
    """Test SceneState fields from GNSS (NavSatFix)."""

    def test_gnss_has_position(self, gnss_bag: Path):
        report = extract_scene(str(gnss_bag))
        gnss_states = [s for s in report.states if s.source_topic == "/gnss"]
        assert len(gnss_states) > 0
        for s in gnss_states:
            assert s.position is not None
            assert len(s.position) == 3

    def test_gnss_position_is_latlon(self, multi_bag: Path):
        """multi_bag has GNSS at lat=35.6812, lon=139.7671."""
        report = extract_scene(str(multi_bag))
        gnss_states = [s for s in report.states if s.source_topic == "/gnss"]
        assert len(gnss_states) > 0
        s = gnss_states[0]
        assert abs(s.position[0] - 35.6812) < 0.01  # latitude as x
        assert abs(s.position[1] - 139.7671) < 0.01  # longitude as y

    def test_gnss_no_velocity(self, gnss_bag: Path):
        """NavSatFix does not provide velocity."""
        report = extract_scene(str(gnss_bag))
        gnss_states = [s for s in report.states if s.source_topic == "/gnss"]
        for s in gnss_states:
            assert s.linear_velocity is None
            assert s.angular_velocity is None

    def test_gnss_no_orientation(self, gnss_bag: Path):
        """NavSatFix does not provide orientation."""
        report = extract_scene(str(gnss_bag))
        gnss_states = [s for s in report.states if s.source_topic == "/gnss"]
        for s in gnss_states:
            assert s.orientation is None


class TestImuStates:
    """Test SceneState fields from IMU."""

    def test_imu_has_angular_velocity(self, imu_bag: Path):
        report = extract_scene(str(imu_bag))
        imu_states = [s for s in report.states if s.source_topic == "/imu"]
        assert len(imu_states) > 0
        for s in imu_states:
            assert s.angular_velocity is not None
            assert len(s.angular_velocity) == 3

    def test_imu_has_acceleration(self, imu_bag: Path):
        report = extract_scene(str(imu_bag))
        imu_states = [s for s in report.states if s.source_topic == "/imu"]
        assert len(imu_states) > 0
        for s in imu_states:
            assert s.acceleration is not None
            assert len(s.acceleration) == 3

    def test_imu_acceleration_values(self, multi_bag: Path):
        """multi_bag IMU has accel=(0.01, -0.02, 9.81)."""
        report = extract_scene(str(multi_bag))
        imu_states = [s for s in report.states if s.source_topic == "/imu"]
        assert len(imu_states) > 0
        s = imu_states[0]
        assert abs(s.acceleration[0] - 0.01) < 0.1
        assert abs(s.acceleration[1] - (-0.02)) < 0.1
        assert abs(s.acceleration[2] - 9.81) < 0.1

    def test_imu_no_position(self, imu_bag: Path):
        """IMU does not provide position."""
        report = extract_scene(str(imu_bag))
        imu_states = [s for s in report.states if s.source_topic == "/imu"]
        for s in imu_states:
            assert s.position is None

    def test_imu_no_linear_velocity(self, imu_bag: Path):
        """IMU does not provide linear velocity."""
        report = extract_scene(str(imu_bag))
        imu_states = [s for s in report.states if s.source_topic == "/imu"]
        for s in imu_states:
            assert s.linear_velocity is None


class TestMultiBag:
    """Test with multi_bag fixture (has GNSS + IMU + LiDAR)."""

    def test_merged_timeline(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        # Should have GNSS (50) + IMU (500) states
        assert len(report.states) == 550

    def test_time_ordered(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        for i in range(1, len(report.states)):
            assert report.states[i].timestamp_sec >= report.states[i - 1].timestamp_sec

    def test_duration_positive(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        assert report.duration_sec > 0

    def test_sources_count(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        assert report.sources["/gnss"] == 50
        assert report.sources["/imu"] == 500


class TestTopicFiltering:
    """Test topic filtering."""

    def test_filter_single_topic(self, multi_bag: Path):
        report = extract_scene(str(multi_bag), topics=["/gnss"])
        assert "/gnss" in report.sources
        assert "/imu" not in report.sources
        assert len(report.states) == 50

    def test_filter_multiple_topics(self, multi_bag: Path):
        report = extract_scene(str(multi_bag), topics=["/gnss", "/imu"])
        assert "/gnss" in report.sources
        assert "/imu" in report.sources
        assert len(report.states) == 550

    def test_filter_nonexistent_topic(self, multi_bag: Path):
        report = extract_scene(str(multi_bag), topics=["/nonexistent"])
        assert len(report.states) == 0
        assert len(report.sources) == 0


class TestCsvExport:
    """Test CSV export."""

    def test_csv_created(self, multi_bag: Path, tmp_path: Path):
        report = extract_scene(str(multi_bag))
        csv_path = tmp_path / "scene.csv"
        export_scene_csv(report, csv_path)
        assert csv_path.exists()

    def test_csv_header(self, multi_bag: Path, tmp_path: Path):
        report = extract_scene(str(multi_bag))
        csv_path = tmp_path / "scene.csv"
        export_scene_csv(report, csv_path)

        with open(csv_path) as f:
            reader = csv.DictReader(f)
            fields = reader.fieldnames
            assert "timestamp" in fields
            assert "pos_x" in fields
            assert "pos_y" in fields
            assert "pos_z" in fields
            assert "ori_x" in fields
            assert "ori_w" in fields
            assert "vel_x" in fields
            assert "ang_vel_x" in fields
            assert "accel_x" in fields
            assert "source" in fields

    def test_csv_row_count(self, multi_bag: Path, tmp_path: Path):
        report = extract_scene(str(multi_bag))
        csv_path = tmp_path / "scene.csv"
        export_scene_csv(report, csv_path)

        with open(csv_path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            assert len(rows) == len(report.states)

    def test_csv_gnss_row_has_position(self, multi_bag: Path, tmp_path: Path):
        report = extract_scene(str(multi_bag), topics=["/gnss"])
        csv_path = tmp_path / "scene.csv"
        export_scene_csv(report, csv_path)

        with open(csv_path) as f:
            reader = csv.DictReader(f)
            row = next(reader)
            assert row["pos_x"] != ""
            assert row["pos_y"] != ""
            assert float(row["pos_x"]) == pytest.approx(35.6812, abs=0.01)


class TestJsonOutput:
    """Test JSON output."""

    def test_json_output(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scene.json"
        with open(json_path, "w") as f:
            extract_scene(str(multi_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "bag_path" in data
        assert "states" in data
        assert "sources" in data
        assert "duration_sec" in data
        assert "state_count" in data
        assert data["state_count"] == 550

    def test_json_state_structure(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scene.json"
        with open(json_path, "w") as f:
            extract_scene(str(multi_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        # Find a GNSS state
        gnss_states = [s for s in data["states"] if s["source_topic"] == "/gnss"]
        assert len(gnss_states) > 0
        s = gnss_states[0]
        assert "timestamp_sec" in s
        assert "position" in s
        assert s["position"]["x"] == pytest.approx(35.6812, abs=0.01)

    def test_json_sources(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scene.json"
        with open(json_path, "w") as f:
            extract_scene(str(multi_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert data["sources"]["/gnss"] == 50
        assert data["sources"]["/imu"] == 500


class TestSceneReport:
    """Test SceneReport dataclass."""

    def test_to_dict(self, multi_bag: Path):
        report = extract_scene(str(multi_bag))
        d = report.to_dict()
        assert isinstance(d, dict)
        assert d["state_count"] == len(report.states)
        assert d["duration_sec"] == report.duration_sec

    def test_empty_report(self, multi_bag: Path):
        report = extract_scene(str(multi_bag), topics=["/nonexistent"])
        assert len(report.states) == 0
        assert report.duration_sec == 0.0
        d = report.to_dict()
        assert d["state_count"] == 0
