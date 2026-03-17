"""Tests for MCAP file format support in bagx.reader."""

from __future__ import annotations

import struct
from pathlib import Path

import pytest

mcap = pytest.importorskip("mcap")
mcap_ros2 = pytest.importorskip("mcap_ros2")

from mcap.writer import Writer as McapWriter

from bagx.reader import BagReader


def _build_navsatfix_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    latitude: float,
    longitude: float,
    altitude: float,
    status: int = 0,
) -> bytes:
    """Build a minimal CDR-encoded NavSatFix message."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)  # CDR header
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)  # stamp
    frame_id = b"gps\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    # NavSatStatus: status (i8) + pad + service (u16)
    buf += struct.pack("<b", status)
    buf += b"\x00"  # pad to align u16
    buf += struct.pack("<H", 1)  # SERVICE_GPS
    # Align to 8 bytes for doubles (current payload size after CDR header = 18 bytes)
    payload_len = len(buf) - 4
    pad = (8 - (payload_len % 8)) % 8
    buf += b"\x00" * pad
    buf += struct.pack("<ddd", latitude, longitude, altitude)
    buf += struct.pack("<9d", *([1.0] + [0.0] * 8))  # covariance
    buf += struct.pack("<B", 2)  # covariance_type
    return buf


# ROS2 IDL schema for sensor_msgs/msg/NavSatFix (simplified)
_NAVSATFIX_SCHEMA = """\
# NavSatFix
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
"""


def _create_mcap_bag(
    path: Path,
    topics: list[dict],
    messages: list[dict],
) -> Path:
    """Create a minimal .mcap file with given topics and messages.

    Args:
        path: Output .mcap path.
        topics: List of {"name": str, "type": str, "encoding": str, "schema": str}.
        messages: List of {"topic": str, "timestamp_ns": int, "data": bytes}.
    """
    with open(path, "wb") as f:
        writer = McapWriter(f)
        writer.start()

        # Register schemas and channels
        schema_ids = {}
        channel_ids = {}
        for t in topics:
            schema_id = writer.register_schema(
                name=t["type"],
                encoding="ros2msg",
                data=t.get("schema", "").encode(),
            )
            schema_ids[t["name"]] = schema_id
            channel_id = writer.register_channel(
                topic=t["name"],
                message_encoding=t.get("encoding", "cdr"),
                schema_id=schema_id,
            )
            channel_ids[t["name"]] = channel_id

        for msg in messages:
            writer.add_message(
                channel_id=channel_ids[msg["topic"]],
                log_time=msg["timestamp_ns"],
                data=msg["data"],
                publish_time=msg["timestamp_ns"],
            )

        writer.finish()

    return path


@pytest.fixture
def gnss_mcap(tmp_path: Path) -> Path:
    """Create an mcap bag with 10 GNSS messages."""
    topics = [
        {
            "name": "/gnss",
            "type": "sensor_msgs/msg/NavSatFix",
            "encoding": "cdr",
            "schema": _NAVSATFIX_SCHEMA,
        }
    ]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    for i in range(10):
        ts = base_ns + i * 100_000_000  # 10Hz
        data = _build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            latitude=35.6812 + i * 0.0001,
            longitude=139.7671 + i * 0.0001,
            altitude=40.0 + i,
        )
        messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

    return _create_mcap_bag(tmp_path / "gnss.mcap", topics, messages)


@pytest.fixture
def multi_mcap(tmp_path: Path) -> Path:
    """Create an mcap bag with GNSS + IMU topics."""
    topics = [
        {
            "name": "/gnss",
            "type": "sensor_msgs/msg/NavSatFix",
            "encoding": "cdr",
            "schema": _NAVSATFIX_SCHEMA,
        },
        {
            "name": "/imu",
            "type": "sensor_msgs/msg/Imu",
            "encoding": "cdr",
            "schema": "# Imu\nstd_msgs/Header header\n",
        },
    ]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    # 5 GNSS messages
    for i in range(5):
        ts = base_ns + i * 100_000_000
        data = _build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            latitude=35.6812,
            longitude=139.7671,
            altitude=40.0,
        )
        messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

    # 10 IMU messages (minimal CDR: just header + enough data)
    for i in range(10):
        ts = base_ns + i * 50_000_000
        # Minimal CDR data for IMU (just enough for basic parsing)
        buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)  # CDR header
        buf += struct.pack("<II", ts // 1_000_000_000, ts % 1_000_000_000)
        frame_id = b"imu\x00"
        buf += struct.pack("<I", len(frame_id))
        buf += frame_id
        messages.append({"topic": "/imu", "timestamp_ns": ts, "data": buf})

    # Sort by timestamp
    messages.sort(key=lambda m: m["timestamp_ns"])

    return _create_mcap_bag(tmp_path / "multi.mcap", topics, messages)


class TestMcapSummary:
    def test_summary_gnss(self, gnss_mcap: Path):
        reader = BagReader(gnss_mcap)
        summary = reader.summary()

        assert summary.message_count == 10
        assert len(summary.topics) == 1
        assert "/gnss" in summary.topics
        assert summary.topics["/gnss"].type == "sensor_msgs/msg/NavSatFix"
        assert summary.topics["/gnss"].count == 10
        assert summary.duration_sec > 0

    def test_summary_multi(self, multi_mcap: Path):
        reader = BagReader(multi_mcap)
        summary = reader.summary()

        assert len(summary.topics) == 2
        assert "/gnss" in summary.topics
        assert "/imu" in summary.topics
        assert summary.message_count == 15  # 5 + 10

    def test_summary_cached(self, gnss_mcap: Path):
        reader = BagReader(gnss_mcap)
        s1 = reader.summary()
        s2 = reader.summary()
        assert s1 is s2


class TestMcapReadMessages:
    def test_read_all(self, gnss_mcap: Path):
        reader = BagReader(gnss_mcap)
        messages = list(reader.read_messages())

        assert len(messages) == 10
        assert all(m.topic == "/gnss" for m in messages)
        # Ordered by timestamp
        timestamps = [m.timestamp_ns for m in messages]
        assert timestamps == sorted(timestamps)

    def test_read_filtered(self, multi_mcap: Path):
        reader = BagReader(multi_mcap)

        gnss_msgs = list(reader.read_messages(topics=["/gnss"]))
        assert len(gnss_msgs) == 5
        assert all(m.topic == "/gnss" for m in gnss_msgs)

        imu_msgs = list(reader.read_messages(topics=["/imu"]))
        assert len(imu_msgs) == 10
        assert all(m.topic == "/imu" for m in imu_msgs)

    def test_message_data(self, gnss_mcap: Path):
        """Messages should have data dict (from CDR fallback parser at minimum)."""
        reader = BagReader(gnss_mcap)
        messages = list(reader.read_messages())
        msg = messages[0]

        # Data should be a dict with some content
        assert isinstance(msg.data, dict)
        assert len(msg.data) > 0

    def test_timestamp_sec(self, gnss_mcap: Path):
        reader = BagReader(gnss_mcap)
        messages = list(reader.read_messages())
        msg = messages[0]

        assert msg.timestamp_sec > 1_000_000_000
        assert msg.timestamp_sec == msg.timestamp_ns / 1e9


class TestMcapDetection:
    def test_detect_mcap_file(self, gnss_mcap: Path):
        reader = BagReader(gnss_mcap)
        assert reader._is_mcap is True

    def test_detect_mcap_directory(self, gnss_mcap: Path):
        """BagReader should detect .mcap files inside a directory."""
        reader = BagReader(gnss_mcap.parent)
        assert reader._is_mcap is True

    def test_file_not_found(self):
        with pytest.raises(FileNotFoundError):
            BagReader("/nonexistent/path.mcap")


class TestMcapWithCommands:
    """Test that mcap files work with eval/export commands."""

    def test_eval_mcap(self, gnss_mcap: Path):
        from bagx.eval import evaluate_bag

        report = evaluate_bag(str(gnss_mcap))
        # Should not raise; GNSS data should be evaluated
        assert report.gnss is not None
        assert report.gnss.fix_count > 0

    def test_export_mcap(self, gnss_mcap: Path, tmp_path: Path):
        from bagx.export import export_bag

        out_dir = tmp_path / "out"
        files = export_bag(str(gnss_mcap), str(out_dir))

        assert len(files) >= 1
        assert "/gnss" in files
        assert files["/gnss"].exists()
