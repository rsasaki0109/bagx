"""Tests for bagx.eval module."""

import math
from pathlib import Path

from bagx.eval import evaluate_bag


class TestEvalGnss:
    def test_gnss_fix_rate(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        assert report.gnss is not None
        assert report.gnss.fix_rate == pytest.approx(0.9, abs=0.01)
        assert report.gnss.fix_count == 90
        assert report.gnss.no_fix_count == 10

    def test_gnss_hdop(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        g = report.gnss
        assert not math.isnan(g.hdop_mean)
        assert g.hdop_mean > 0
        assert g.hdop_max >= g.hdop_mean

    def test_gnss_coordinates(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        g = report.gnss
        lat_min, lat_max = g.latitude_range
        assert abs(lat_min - 35.6812) < 0.01
        assert abs(lat_max - 35.6812) < 0.01

    def test_gnss_score_range(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        assert 0 <= report.gnss.score <= 100


class TestEvalImu:
    def test_imu_metrics(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        assert report.imu is not None
        assert report.imu.total_messages == 1000

    def test_imu_noise(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        m = report.imu
        # Noise should be small (generated with std=0.05 for accel)
        assert not math.isnan(m.accel_noise_x)
        assert m.accel_noise_x < 1.0
        assert not math.isnan(m.gyro_noise_x)
        assert m.gyro_noise_x < 0.1

    def test_imu_frequency(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        # Generated at 200Hz (5ms interval)
        assert report.imu.frequency_hz == pytest.approx(200.0, rel=0.05)

    def test_imu_bias_stability(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        assert not math.isnan(report.imu.accel_bias_stability)
        assert not math.isnan(report.imu.gyro_bias_stability)

    def test_imu_score_range(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        assert 0 <= report.imu.score <= 100


class TestEvalMulti:
    def test_overall_score(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.overall_score > 0
        assert report.overall_score <= 100

    def test_sync_detected(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.sync is not None
        assert len(report.sync.topic_pairs) > 0

    def test_topic_count(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.topic_count == 3
        assert report.total_messages == 600

    def test_duration(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.duration_sec > 0


class TestEvalReport:
    def test_to_dict_no_nan(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        d = report.to_dict()
        # NaN values should be replaced with None
        _assert_no_nan(d)

    def test_json_output(self, gnss_bag: Path, tmp_path: Path):
        import json

        json_path = tmp_path / "report.json"
        with open(json_path, "w") as f:
            evaluate_bag(str(gnss_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "gnss" in data
        assert data["gnss"]["fix_rate"] == pytest.approx(0.9, abs=0.01)


def _assert_no_nan(obj):
    if isinstance(obj, dict):
        for v in obj.values():
            _assert_no_nan(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            _assert_no_nan(v)
    elif isinstance(obj, float):
        assert not math.isnan(obj), f"Found NaN in report dict"


import pytest
