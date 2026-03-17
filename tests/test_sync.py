"""Tests for bagx.sync module."""

from pathlib import Path

import pytest

from bagx.sync import analyze_sync, analyze_sync_multi


class TestSync:
    def test_sync_two_topics(self, multi_bag: Path):
        report = analyze_sync(str(multi_bag), "/gnss", "/imu")

        assert len(report.pairs) == 1
        pair = report.pairs[0]
        assert pair.topic_a == "/gnss"
        assert pair.topic_b == "/imu"
        assert pair.count > 0
        assert pair.mean_delay_ms >= 0
        assert pair.max_delay_ms >= pair.mean_delay_ms
        assert pair.min_delay_ms <= pair.mean_delay_ms

    def test_sync_gnss_lidar(self, multi_bag: Path):
        report = analyze_sync(str(multi_bag), "/gnss", "/lidar")

        assert len(report.pairs) == 1
        pair = report.pairs[0]
        # LiDAR has ±5ms jitter, so mean delay should be in that range
        assert pair.mean_delay_ms < 20.0

    def test_sync_statistics(self, multi_bag: Path):
        report = analyze_sync(str(multi_bag), "/gnss", "/imu")
        pair = report.pairs[0]

        assert pair.std_delay_ms >= 0
        assert pair.median_delay_ms >= 0
        assert pair.p95_delay_ms >= pair.median_delay_ms
        assert 0 <= pair.outlier_rate <= 1.0

    def test_sync_multi(self, multi_bag: Path):
        report = analyze_sync_multi(str(multi_bag))

        # 3 topics -> 3 pairs
        assert len(report.pairs) == 3
        pair_names = [(p.topic_a, p.topic_b) for p in report.pairs]
        # All combinations
        topics = ["/gnss", "/imu", "/lidar"]
        for i, t1 in enumerate(sorted(topics)):
            for t2 in sorted(topics)[i + 1 :]:
                assert (t1, t2) in pair_names

    def test_sync_multi_filtered(self, multi_bag: Path):
        report = analyze_sync_multi(str(multi_bag), topics=["/gnss", "/lidar"])

        assert len(report.pairs) == 1
        assert report.pairs[0].topic_a == "/gnss"
        assert report.pairs[0].topic_b == "/lidar"

    def test_sync_json_output(self, multi_bag: Path, tmp_path: Path):
        import json

        json_path = tmp_path / "sync.json"
        with open(json_path, "w") as f:
            analyze_sync(str(multi_bag), "/gnss", "/imu", output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "pairs" in data
        assert len(data["pairs"]) == 1
        assert "mean_delay_ms" in data["pairs"][0]

    def test_sync_nonexistent_topic(self, multi_bag: Path):
        report = analyze_sync(str(multi_bag), "/gnss", "/nonexistent")

        # Should return empty pairs (one topic has no messages)
        assert len(report.pairs) == 0
