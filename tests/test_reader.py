"""Tests for bagx.reader module."""

from pathlib import Path

import pytest

from bagx.reader import BagReader


class TestBagReader:
    def test_summary_gnss(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        summary = reader.summary()

        assert summary.message_count == 100
        assert len(summary.topics) == 1
        assert "/gnss" in summary.topics
        assert summary.topics["/gnss"].type == "sensor_msgs/msg/NavSatFix"
        assert summary.topics["/gnss"].count == 100
        assert summary.duration_sec > 0

    def test_summary_multi(self, multi_bag: Path):
        reader = BagReader(multi_bag)
        summary = reader.summary()

        assert len(summary.topics) == 3
        assert "/gnss" in summary.topics
        assert "/imu" in summary.topics
        assert "/lidar" in summary.topics
        assert summary.message_count == 600  # 50 + 500 + 50

    def test_read_all_messages(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        messages = list(reader.read_messages())

        assert len(messages) == 100
        assert all(m.topic == "/gnss" for m in messages)
        # Messages should be ordered by timestamp
        timestamps = [m.timestamp_ns for m in messages]
        assert timestamps == sorted(timestamps)

    def test_read_filtered_topics(self, multi_bag: Path):
        reader = BagReader(multi_bag)

        gnss_msgs = list(reader.read_messages(topics=["/gnss"]))
        assert len(gnss_msgs) == 50
        assert all(m.topic == "/gnss" for m in gnss_msgs)

        imu_msgs = list(reader.read_messages(topics=["/imu"]))
        assert len(imu_msgs) == 500

    def test_message_data_navsatfix(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        messages = list(reader.read_messages())
        msg = messages[0]

        assert "latitude" in msg.data
        assert "longitude" in msg.data
        assert "altitude" in msg.data
        assert "status" in msg.data
        assert abs(msg.data["latitude"] - 35.6812) < 0.01
        assert abs(msg.data["longitude"] - 139.7671) < 0.01

    def test_message_data_imu(self, imu_bag: Path):
        reader = BagReader(imu_bag)
        messages = list(reader.read_messages())
        msg = messages[0]

        assert "linear_acceleration" in msg.data
        assert "angular_velocity" in msg.data
        la = msg.data["linear_acceleration"]
        assert "x" in la and "y" in la and "z" in la
        # z should be close to 9.81 (gravity)
        assert abs(la["z"] - 9.81) < 1.0

    def test_timestamp_sec(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        messages = list(reader.read_messages())
        msg = messages[0]

        assert msg.timestamp_sec > 1_000_000_000  # After year 2001
        assert msg.timestamp_sec == msg.timestamp_ns / 1e9

    def test_file_not_found(self):
        with pytest.raises(FileNotFoundError):
            BagReader("/nonexistent/path.db3")

    def test_summary_cached(self, gnss_bag: Path):
        reader = BagReader(gnss_bag)
        s1 = reader.summary()
        s2 = reader.summary()
        assert s1 is s2  # Same object (cached)
