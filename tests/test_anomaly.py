"""Tests for bagx.anomaly module."""

from __future__ import annotations

import json
from pathlib import Path


from bagx.anomaly import detect_anomalies, AnomalyReport


class TestGnssAnomalies:
    def test_gnss_fix_drop_detected(self, gnss_bag: Path):
        """The gnss_bag fixture has fix for first 90, then no-fix for last 10."""
        report = detect_anomalies(str(gnss_bag))
        fix_drops = [a for a in report.anomalies if a.type == "gnss_fix_drop"]
        # There should be at least one fix drop (transition from fix to no-fix)
        assert len(fix_drops) >= 1

    def test_gnss_topic_filter(self, gnss_bag: Path):
        """Filtering by topic should only return anomalies for that topic."""
        report = detect_anomalies(str(gnss_bag), topics=["/gnss"])
        for a in report.anomalies:
            assert a.topic == "/gnss"

    def test_gnss_no_jump_in_normal_data(self, gnss_bag: Path):
        """Normal GNSS data (gaussian noise ~0.0001 deg) should not trigger jumps."""
        report = detect_anomalies(str(gnss_bag))
        jumps = [a for a in report.anomalies if a.type == "gnss_jump"]
        # With std=0.0001 deg, jump threshold is ~50m (~0.00045 deg), jumps unlikely
        assert len(jumps) == 0

    def test_gnss_anomaly_has_required_fields(self, gnss_bag: Path):
        report = detect_anomalies(str(gnss_bag))
        for a in report.anomalies:
            assert a.timestamp_ns > 0
            assert a.topic != ""
            assert a.type != ""
            assert a.severity in ("low", "medium", "high")
            assert a.description != ""


class TestGnssJumpDetection:
    def test_position_jump_detected(self, tmp_path: Path):
        """Create a bag with a deliberate position jump and verify detection."""
        from tests.conftest import _create_db3, build_navsatfix_cdr

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(20):
            ts = base_ns + i * 100_000_000
            # At message 10, jump latitude by 0.01 deg (~1.1km)
            lat = 35.6812 if i < 10 else 35.6912
            data = build_navsatfix_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                status=0,
                latitude=lat,
                longitude=139.7671,
                altitude=40.0,
            )
            messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

        bag_path = tmp_path / "jump.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        jumps = [a for a in report.anomalies if a.type == "gnss_jump"]
        assert len(jumps) >= 1
        assert jumps[0].severity in ("medium", "high")


class TestImuAnomalies:
    def test_no_spikes_in_normal_data(self, imu_bag: Path):
        """Normal IMU data should have very few or no accel/gyro spikes."""
        report = detect_anomalies(str(imu_bag))
        spikes = [a for a in report.anomalies if a.type in ("imu_accel_spike", "imu_gyro_spike")]
        # With gaussian noise, >4 sigma events are very rare in 1000 samples
        assert len(spikes) < 5

    def test_accel_spike_detected(self, tmp_path: Path):
        """Create a bag with an acceleration spike and verify detection."""
        from tests.conftest import _create_db3, build_imu_cdr

        topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(200):
            ts = base_ns + i * 5_000_000
            # At message 100, inject a large acceleration spike
            if i == 100:
                accel = (50.0, 0.0, 9.81)
            else:
                accel = (0.01, -0.01, 9.81)
            data = build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                accel=accel,
                gyro=(0.001, -0.001, 0.0),
            )
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": data})

        bag_path = tmp_path / "spike.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        accel_spikes = [a for a in report.anomalies if a.type == "imu_accel_spike"]
        assert len(accel_spikes) >= 1


class TestRateAnomalies:
    def test_rate_gap_detected(self, tmp_path: Path):
        """Create a bag with a large gap and verify rate anomaly detection."""
        from tests.conftest import _create_db3, build_imu_cdr

        topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(50):
            ts = base_ns + i * 5_000_000  # 5ms interval
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )})

        # Add a 500ms gap
        gap_ts = base_ns + 50 * 5_000_000 + 500_000_000
        for i in range(50):
            ts = gap_ts + i * 5_000_000
            messages.append({"topic": "/imu", "timestamp_ns": ts, "data": build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )})

        bag_path = tmp_path / "gap.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        # IMU topics are detected via imu_frequency_drop (not rate_gap)
        freq_drops = [a for a in report.anomalies if a.type == "imu_frequency_drop"]
        assert len(freq_drops) >= 1

    def test_no_rate_anomaly_in_uniform_data(self, imu_bag: Path):
        """Uniform 200Hz data should not produce rate gap anomalies."""
        report = detect_anomalies(str(imu_bag))
        rate_gaps = [a for a in report.anomalies if a.type == "rate_gap"]
        assert len(rate_gaps) == 0

    def test_long_bag_startup_gap_is_ignored(self, tmp_path: Path):
        """Startup transients in long bags should not be reported as rate gaps."""
        from tests.conftest import _create_db3, build_stub_cdr

        topics = [{"name": "/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        messages.append({"topic": "/odom", "timestamp_ns": base_ns, "data": build_stub_cdr()})
        messages.append({"topic": "/odom", "timestamp_ns": base_ns + 400_000_000, "data": build_stub_cdr()})

        start_ns = base_ns + 500_000_000
        for i in range(180):
            ts = start_ns + i * 100_000_000
            messages.append({"topic": "/odom", "timestamp_ns": ts, "data": build_stub_cdr()})

        bag_path = tmp_path / "startup_gap_long.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        rate_gaps = [a for a in report.anomalies if a.type == "rate_gap"]
        assert rate_gaps == []

    def test_late_gap_still_detected_after_warmup(self, tmp_path: Path):
        """Warmup suppression should not hide later runtime gaps."""
        from tests.conftest import _create_db3, build_stub_cdr

        topics = [{"name": "/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(70):
            ts = base_ns + i * 100_000_000
            messages.append({"topic": "/odom", "timestamp_ns": ts, "data": build_stub_cdr()})

        gap_base_ns = base_ns + 70 * 100_000_000 + 600_000_000
        for i in range(60):
            ts = gap_base_ns + i * 100_000_000
            messages.append({"topic": "/odom", "timestamp_ns": ts, "data": build_stub_cdr()})

        bag_path = tmp_path / "late_gap_long.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        rate_gaps = [a for a in report.anomalies if a.type == "rate_gap"]
        assert len(rate_gaps) >= 1

    def test_clock_topic_is_excluded_from_generic_rate_anomalies(self, tmp_path: Path):
        """Meta timing topics should not dominate anomaly reports."""
        from tests.conftest import _create_db3, build_stub_cdr

        topics = [{"name": "/clock", "type": "rosgraph_msgs/msg/Clock", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        for i in range(50):
            ts = base_ns + i * 10_000_000
            messages.append({"topic": "/clock", "timestamp_ns": ts, "data": build_stub_cdr()})

        gap_base_ns = base_ns + 50 * 10_000_000 + 500_000_000
        for i in range(50):
            ts = gap_base_ns + i * 10_000_000
            messages.append({"topic": "/clock", "timestamp_ns": ts, "data": build_stub_cdr()})

        bag_path = tmp_path / "clock_gap.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        rate_gaps = [a for a in report.anomalies if a.type == "rate_gap"]
        assert rate_gaps == []

    def test_planning_scene_topic_is_excluded_from_generic_rate_anomalies(self, tmp_path: Path):
        """Internal planning scene updates should not be treated as sensor rate faults."""
        from tests.conftest import _create_db3, build_stub_cdr

        topics = [{"name": "/monitored_planning_scene", "type": "moveit_msgs/msg/PlanningScene", "format": "cdr"}]
        messages = []
        base_ns = 1_700_000_000_000_000_000

        regular_offsets = [0, 250_000_000, 500_000_000, 750_000_000, 1_000_000_000]
        for cycle in range(3):
            cycle_base = base_ns + cycle * 3_000_000_000
            for offset in regular_offsets:
                messages.append({
                    "topic": "/monitored_planning_scene",
                    "timestamp_ns": cycle_base + offset,
                    "data": build_stub_cdr(),
                })

        bag_path = tmp_path / "planning_scene_gap.db3"
        _create_db3(bag_path, topics, messages)

        report = detect_anomalies(str(bag_path))
        rate_gaps = [a for a in report.anomalies if a.type == "rate_gap"]
        assert rate_gaps == []


class TestAnomalyReport:
    def test_report_structure(self, gnss_bag: Path):
        report = detect_anomalies(str(gnss_bag))
        assert isinstance(report, AnomalyReport)
        assert report.bag_path == str(gnss_bag)
        assert report.total_anomalies == len(report.anomalies)

    def test_to_dict(self, gnss_bag: Path):
        report = detect_anomalies(str(gnss_bag))
        d = report.to_dict()
        assert "bag_path" in d
        assert "total_anomalies" in d
        assert "anomalies" in d
        assert isinstance(d["anomalies"], list)

    def test_json_output(self, gnss_bag: Path, tmp_path: Path):
        json_path = tmp_path / "anomalies.json"
        with open(json_path, "w") as f:
            detect_anomalies(str(gnss_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "bag_path" in data
        assert "total_anomalies" in data
        assert isinstance(data["anomalies"], list)

    def test_anomalies_sorted_by_timestamp(self, multi_bag: Path):
        report = detect_anomalies(str(multi_bag))
        timestamps = [a.timestamp_ns for a in report.anomalies]
        assert timestamps == sorted(timestamps)

    def test_multi_bag_anomalies(self, multi_bag: Path):
        """Multi-topic bag should be analyzable without errors."""
        report = detect_anomalies(str(multi_bag))
        assert isinstance(report, AnomalyReport)
        assert report.total_anomalies >= 0
