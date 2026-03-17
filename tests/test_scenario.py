"""Tests for bagx.scenario module."""

from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from bagx.scenario import detect_scenarios, ScenarioReport


class TestGnssLost:
    def test_gnss_lost_detected(self, tmp_path: Path):
        """Create a bag where GNSS fix is lost for a sustained period."""
        from tests.conftest import _create_db3, build_navsatfix_cdr

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(100):
            ts = base_ns + i * 100_000_000  # 10Hz
            # Fix for first 30, no-fix for 30-70, fix again 70-100
            if i < 30:
                status = 0
            elif i < 70:
                status = -1
            else:
                status = 0
            data = build_navsatfix_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                status=status,
                latitude=35.6812,
                longitude=139.7671,
                altitude=40.0,
            )
            messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

        bag_path = tmp_path / "gnss_lost.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_scenarios(str(bag_path), gnss_lost_threshold_sec=1.0)
        gnss_lost = [s for s in report.scenarios if s.type == "gnss_lost"]
        assert len(gnss_lost) >= 1
        # The no-fix period is 4.0 seconds (messages 30-70 at 10Hz)
        assert gnss_lost[0].duration_ns > 1_000_000_000  # > 1s

    def test_no_gnss_lost_when_all_fixed(self, tmp_path: Path):
        """All-fix data should not trigger GNSS lost."""
        from tests.conftest import _create_db3, build_navsatfix_cdr

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(50):
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

        bag_path = tmp_path / "gnss_ok.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_scenarios(str(bag_path))
        gnss_lost = [s for s in report.scenarios if s.type == "gnss_lost"]
        assert len(gnss_lost) == 0

    def test_gnss_lost_at_end(self, gnss_bag: Path):
        """The gnss_bag fixture has no-fix for last 10 messages (1 second at 10Hz)."""
        report = detect_scenarios(str(gnss_bag), gnss_lost_threshold_sec=0.5)
        gnss_lost = [s for s in report.scenarios if s.type == "gnss_lost"]
        # The no-fix section is 0.9s (10 messages * 100ms - last interval)
        assert len(gnss_lost) >= 1


class TestSensorDropout:
    def test_dropout_detected(self, tmp_path: Path):
        """Create a bag with a large gap in one topic."""
        from tests.conftest import _create_db3, build_imu_cdr

        topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        # 50 messages, then a 5-second gap, then 50 more
        for i in range(50):
            ts = base_ns + i * 5_000_000
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )})

        gap_ts = base_ns + 50 * 5_000_000 + 5_000_000_000  # 5s gap
        for i in range(50):
            ts = gap_ts + i * 5_000_000
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )})

        bag_path = tmp_path / "dropout.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_scenarios(str(bag_path), dropout_threshold_sec=2.0)
        dropouts = [s for s in report.scenarios if s.type == "sensor_dropout"]
        assert len(dropouts) >= 1
        assert dropouts[0].duration_ns >= 5_000_000_000

    def test_no_dropout_in_continuous_data(self, imu_bag: Path):
        """Continuous 200Hz data should not trigger dropout."""
        report = detect_scenarios(str(imu_bag), dropout_threshold_sec=2.0)
        dropouts = [s for s in report.scenarios if s.type == "sensor_dropout"]
        assert len(dropouts) == 0


class TestHighDynamics:
    def test_high_dynamics_detected(self, tmp_path: Path):
        """Create a bag with a high-acceleration event."""
        from tests.conftest import _create_db3, build_imu_cdr

        topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(200):
            ts = base_ns + i * 5_000_000
            # Messages 80-90: simulate hard braking (20 m/s² forward decel)
            if 80 <= i <= 90:
                accel = (20.0, 0.0, 9.81)
            else:
                accel = (0.01, -0.01, 9.81)
            data = build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                accel=accel,
                gyro=(0.001, -0.001, 0.0),
            )
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": data})

        bag_path = tmp_path / "dynamics.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_scenarios(str(bag_path), accel_threshold_mps2=15.0)
        dynamics = [s for s in report.scenarios if s.type == "high_dynamics"]
        assert len(dynamics) >= 1
        assert dynamics[0].duration_ns > 0

    def test_no_high_dynamics_in_normal_data(self, imu_bag: Path):
        """Normal IMU data should not trigger high dynamics."""
        report = detect_scenarios(str(imu_bag), accel_threshold_mps2=15.0)
        dynamics = [s for s in report.scenarios if s.type == "high_dynamics"]
        assert len(dynamics) == 0


class TestScenarioReport:
    def test_report_structure(self, multi_bag: Path):
        report = detect_scenarios(str(multi_bag))
        assert isinstance(report, ScenarioReport)
        assert report.bag_path == str(multi_bag)
        assert report.total_scenarios == len(report.scenarios)

    def test_to_dict(self, multi_bag: Path):
        report = detect_scenarios(str(multi_bag))
        d = report.to_dict()
        assert "bag_path" in d
        assert "total_scenarios" in d
        assert "scenarios" in d
        assert isinstance(d["scenarios"], list)

    def test_json_output(self, multi_bag: Path, tmp_path: Path):
        json_path = tmp_path / "scenarios.json"
        with open(json_path, "w") as f:
            detect_scenarios(str(multi_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "bag_path" in data
        assert "total_scenarios" in data
        assert isinstance(data["scenarios"], list)

    def test_scenarios_sorted_by_start_time(self, multi_bag: Path):
        report = detect_scenarios(str(multi_bag))
        start_times = [s.start_time_ns for s in report.scenarios]
        assert start_times == sorted(start_times)

    def test_scenario_fields(self, tmp_path: Path):
        """Verify all scenario fields are populated correctly."""
        from tests.conftest import _create_db3, build_imu_cdr

        topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        # Create a dropout scenario
        for i in range(20):
            ts = base_ns + i * 5_000_000
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )})

        gap_ts = base_ns + 20 * 5_000_000 + 3_000_000_000  # 3s gap
        for i in range(20):
            ts = gap_ts + i * 5_000_000
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )})

        bag_path = tmp_path / "fields.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_scenarios(str(bag_path), dropout_threshold_sec=2.0)
        dropouts = [s for s in report.scenarios if s.type == "sensor_dropout"]
        assert len(dropouts) >= 1

        s = dropouts[0]
        assert s.start_time_ns > 0
        assert s.end_time_ns > s.start_time_ns
        assert s.duration_ns == s.end_time_ns - s.start_time_ns
        assert s.type == "sensor_dropout"
        assert s.severity in ("low", "medium", "high")
        assert s.description != ""
        assert s.start_time_sec > 0
        assert s.end_time_sec > s.start_time_sec
        assert s.duration_sec > 0
