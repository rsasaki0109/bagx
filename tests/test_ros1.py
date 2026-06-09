"""Tests for ROS1 .bag support via rosbags."""

from __future__ import annotations

from pathlib import Path

import pytest

from bagx.eval import evaluate_bag
from bagx.reader import BagReader, HAS_ROSBAGS
from helpers import create_ros1_bag

pytestmark = pytest.mark.skipif(not HAS_ROSBAGS, reason="rosbags not installed")

BASE_NS = 1_700_000_000_000_000_000


def _make_gnss_ros1_messages(count: int, start_idx: int = 0) -> list[dict]:
    messages = []
    for i in range(count):
        ts = BASE_NS + (start_idx + i) * 100_000_000
        status = 0 if (start_idx + i) < int(count * 0.9) else -1
        messages.append(
            {
                "topic": "/gnss",
                "timestamp_ns": ts,
                "fields": {
                    "stamp_sec": ts // 1_000_000_000,
                    "stamp_nanosec": ts % 1_000_000_000,
                    "seq": start_idx + i,
                    "status": status,
                    "latitude": 35.6812 + i * 0.0001,
                    "longitude": 139.7671,
                    "altitude": 40.0,
                },
            }
        )
    return messages


def _make_imu_ros1_messages(count: int, start_idx: int = 0) -> list[dict]:
    messages = []
    for i in range(count):
        ts = BASE_NS + (start_idx + i) * 5_000_000
        messages.append(
            {
                "topic": "/imu",
                "timestamp_ns": ts,
                "fields": {
                    "stamp_sec": ts // 1_000_000_000,
                    "stamp_nanosec": ts % 1_000_000_000,
                    "seq": start_idx + i,
                    "linear_acceleration": (0.0, 0.0, 9.81),
                },
            }
        )
    return messages


@pytest.fixture
def ros1_gnss_bag(tmp_path: Path) -> Path:
    topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix"}]
    return create_ros1_bag(
        tmp_path / "gnss.bag",
        topics,
        _make_gnss_ros1_messages(100),
    )


@pytest.fixture
def ros1_imu_bag(tmp_path: Path) -> Path:
    topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu"}]
    return create_ros1_bag(
        tmp_path / "imu.bag",
        topics,
        _make_imu_ros1_messages(200),
    )


class TestRos1Reader:
    def test_summary_gnss(self, ros1_gnss_bag: Path):
        reader = BagReader(ros1_gnss_bag)
        summary = reader.summary()

        assert summary.message_count == 100
        assert "/gnss" in summary.topics
        assert summary.topics["/gnss"].type == "sensor_msgs/msg/NavSatFix"
        assert summary.topics["/gnss"].serialization_format == "ros1"
        assert summary.duration_sec > 0

    def test_read_navsatfix_fields(self, ros1_gnss_bag: Path):
        reader = BagReader(ros1_gnss_bag)
        messages = list(reader.read_messages())

        assert len(messages) == 100
        msg = messages[0]
        assert msg.topic == "/gnss"
        assert isinstance(msg.data["status"], int)
        assert abs(msg.data["latitude"] - 35.6812) < 0.01
        assert abs(msg.data["longitude"] - 139.7671) < 0.01

    def test_read_imu_fields(self, ros1_imu_bag: Path):
        reader = BagReader(ros1_imu_bag)
        messages = list(reader.read_messages())
        msg = messages[0]

        la = msg.data["linear_acceleration"]
        assert abs(la["z"] - 9.81) < 1.0

    def test_split_bag_directory(self, tmp_path: Path):
        bag_dir = tmp_path / "split_bag"
        bag_dir.mkdir()
        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix"}]

        create_ros1_bag(
            bag_dir / "data.bag",
            topics,
            _make_gnss_ros1_messages(10, 0),
        )
        create_ros1_bag(
            bag_dir / "data_1.bag",
            topics,
            _make_gnss_ros1_messages(10, 10),
        )

        reader = BagReader(bag_dir)
        summary = reader.summary()
        messages = list(reader.read_messages())

        assert summary.message_count == 20
        assert len(messages) == 20

    def test_split_bag_primary_file(self, tmp_path: Path):
        topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix"}]
        primary = tmp_path / "data.bag"
        create_ros1_bag(primary, topics, _make_gnss_ros1_messages(5, 0))
        create_ros1_bag(
            tmp_path / "data_1.bag",
            topics,
            _make_gnss_ros1_messages(5, 5),
        )

        reader = BagReader(primary)
        summary = reader.summary()

        assert summary.message_count == 10


class TestRos1Eval:
    def test_eval_gnss_bag(self, ros1_gnss_bag: Path):
        report = evaluate_bag(str(ros1_gnss_bag))

        assert report.gnss is not None
        assert report.gnss.total_messages == 100
        assert report.gnss.fix_count > 0
        assert report.overall_score > 0

    def test_eval_imu_bag(self, ros1_imu_bag: Path):
        report = evaluate_bag(str(ros1_imu_bag))

        assert report.imu is not None
        assert report.imu.total_messages == 200
        assert report.imu.frequency_hz > 0
