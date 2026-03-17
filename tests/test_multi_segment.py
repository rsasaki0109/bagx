"""Tests for multi-segment (multi-.db3) bag handling."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch


from bagx.reader import BagReader
from helpers import build_imu_cdr, build_navsatfix_cdr, create_db3


BASE_NS = 1_700_000_000_000_000_000

# Patch HAS_ROS to force the sqlite fallback path for directory-based bags.
# The ROS2 backend does not support our hand-crafted multi-.db3 directories.
_no_ros = patch("bagx.reader.HAS_ROS", False)


def _make_gnss_messages(topic: str, base_ns: int, count: int, start_idx: int = 0):
    """Create a list of GNSS messages for insertion into a .db3."""
    messages = []
    for i in range(count):
        ts = base_ns + (start_idx + i) * 100_000_000  # 10 Hz
        data = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=0,
            latitude=35.68 + i * 0.0001,
            longitude=139.77 + i * 0.0001,
            altitude=40.0,
        )
        messages.append({"topic": topic, "timestamp_ns": ts, "data": data})
    return messages


def _make_imu_messages(topic: str, base_ns: int, count: int, start_idx: int = 0):
    messages = []
    for i in range(count):
        ts = base_ns + (start_idx + i) * 5_000_000  # 200 Hz
        data = build_imu_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
        )
        messages.append({"topic": topic, "timestamp_ns": ts, "data": data})
    return messages


class TestMultiSegmentSummary:
    """Test that summary() aggregates across multiple .db3 files."""

    @_no_ros
    def test_summary_aggregates_message_counts(self, tmp_path: Path):
        bag_dir = tmp_path / "multi_bag"
        bag_dir.mkdir()

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        create_db3(bag_dir / "seg_0.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 10, 0))
        create_db3(bag_dir / "seg_1.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 15, 10))
        create_db3(bag_dir / "seg_2.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 5, 25))

        reader = BagReader(bag_dir)
        summary = reader.summary()

        assert summary.message_count == 30
        assert summary.topics["/gnss"].count == 30

    @_no_ros
    def test_summary_timestamps_span_full_range(self, tmp_path: Path):
        bag_dir = tmp_path / "multi_bag"
        bag_dir.mkdir()

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        msgs_a = _make_gnss_messages("/gnss", BASE_NS, 10, 0)
        msgs_b = _make_gnss_messages("/gnss", BASE_NS, 10, 100)  # big gap

        create_db3(bag_dir / "seg_0.db3", topics, msgs_a)
        create_db3(bag_dir / "seg_1.db3", topics, msgs_b)

        reader = BagReader(bag_dir)
        summary = reader.summary()

        expected_start = msgs_a[0]["timestamp_ns"]
        expected_end = msgs_b[-1]["timestamp_ns"]
        assert summary.start_time_ns == expected_start
        assert summary.end_time_ns == expected_end
        assert summary.duration_ns == expected_end - expected_start

    @_no_ros
    def test_topic_info_unioned_across_segments(self, tmp_path: Path):
        bag_dir = tmp_path / "multi_bag"
        bag_dir.mkdir()

        topics_a = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        topics_b = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]

        create_db3(bag_dir / "seg_0.db3", topics_a, _make_gnss_messages("/gnss", BASE_NS, 5, 0))
        create_db3(bag_dir / "seg_1.db3", topics_b, _make_imu_messages("/imu", BASE_NS, 10, 0))

        reader = BagReader(bag_dir)
        summary = reader.summary()

        assert "/gnss" in summary.topics
        assert "/imu" in summary.topics
        assert summary.topics["/gnss"].count == 5
        assert summary.topics["/imu"].count == 10
        assert summary.message_count == 15


class TestMultiSegmentRead:
    """Test that read_messages yields messages from all .db3 files."""

    @_no_ros
    def test_read_all_files(self, tmp_path: Path):
        bag_dir = tmp_path / "multi_bag"
        bag_dir.mkdir()

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        create_db3(bag_dir / "seg_0.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 5, 0))
        create_db3(bag_dir / "seg_1.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 7, 5))

        reader = BagReader(bag_dir)
        messages = list(reader.read_messages())

        assert len(messages) == 12

    @_no_ros
    def test_read_messages_ordered_within_each_file(self, tmp_path: Path):
        bag_dir = tmp_path / "multi_bag"
        bag_dir.mkdir()

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        create_db3(bag_dir / "seg_0.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 5, 0))
        create_db3(bag_dir / "seg_1.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 5, 5))

        reader = BagReader(bag_dir)
        messages = list(reader.read_messages())

        timestamps = [m.timestamp_ns for m in messages]
        assert timestamps == sorted(timestamps)

    @_no_ros
    def test_read_with_topic_filter(self, tmp_path: Path):
        bag_dir = tmp_path / "multi_bag"
        bag_dir.mkdir()

        topics = [
            {"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
            {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        ]
        msgs = _make_gnss_messages("/gnss", BASE_NS, 5, 0) + _make_imu_messages("/imu", BASE_NS, 10, 0)
        msgs.sort(key=lambda m: m["timestamp_ns"])

        create_db3(bag_dir / "seg_0.db3", topics, msgs)

        reader = BagReader(bag_dir)
        gnss_msgs = list(reader.read_messages(topics=["/gnss"]))
        assert len(gnss_msgs) == 5
        assert all(m.topic == "/gnss" for m in gnss_msgs)


class TestSingleDb3Regression:
    """Test that a single .db3 file still works as before."""

    def test_single_db3_summary(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        summary = reader.summary()
        assert summary.message_count == 100
        assert "/gnss" in summary.topics

    def test_single_db3_read(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        messages = list(reader.read_messages())
        assert len(messages) == 100


class TestMixedFilesInDirectory:
    """Test that non-.db3 files are ignored when reading a directory."""

    @_no_ros
    def test_ignores_non_db3_files(self, tmp_path: Path):
        bag_dir = tmp_path / "mixed_bag"
        bag_dir.mkdir()

        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
        create_db3(bag_dir / "data.db3", topics, _make_gnss_messages("/gnss", BASE_NS, 5, 0))

        # Create non-.db3 files that should be ignored
        (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information:")
        (bag_dir / "notes.txt").write_text("some notes")
        (bag_dir / "data.db3-journal").write_bytes(b"\x00" * 100)

        reader = BagReader(bag_dir)
        summary = reader.summary()
        assert summary.message_count == 5

        messages = list(reader.read_messages())
        assert len(messages) == 5
