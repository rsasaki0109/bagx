"""Shared fixtures for bagx tests.

Creates fake .db3 bag files with known data for deterministic testing,
without requiring ROS2.
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path

import pytest


def _create_db3(path: Path, topics: list[dict], messages: list[dict]) -> Path:
    """Create a minimal rosbag2 .db3 file.

    Args:
        path: Output .db3 path.
        topics: List of {"name": str, "type": str, "format": str}.
        messages: List of {"topic": str, "timestamp_ns": int, "data": bytes}.
    """
    conn = sqlite3.connect(str(path))
    cursor = conn.cursor()

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS topics (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL DEFAULT ''
        )
    """)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS messages (
            id INTEGER PRIMARY KEY,
            topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            data BLOB NOT NULL
        )
    """)

    topic_id_map = {}
    for i, t in enumerate(topics, start=1):
        cursor.execute(
            "INSERT INTO topics (id, name, type, serialization_format) VALUES (?, ?, ?, ?)",
            (i, t["name"], t["type"], t.get("format", "cdr")),
        )
        topic_id_map[t["name"]] = i

    for msg in messages:
        tid = topic_id_map[msg["topic"]]
        cursor.execute(
            "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
            (tid, msg["timestamp_ns"], msg["data"]),
        )

    conn.commit()
    conn.close()
    return path


def _payload_align(buf: bytes, alignment: int, cdr_header_size: int = 4) -> bytes:
    """Pad buffer so the payload offset (after CDR header) is aligned."""
    payload_offset = len(buf) - cdr_header_size
    remainder = payload_offset % alignment
    if remainder != 0:
        buf += b"\x00" * (alignment - remainder)
    return buf


def build_navsatfix_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    status: int,
    latitude: float,
    longitude: float,
    altitude: float,
    hdop_squared: float = 1.0,
) -> bytes:
    """Build a fake CDR-encoded NavSatFix message."""
    # CDR header (little-endian)
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    # Header stamp
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    # frame_id: length + string + null
    frame_id = b"gps\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    # NavSatStatus: status (i8) + padding + service (u16)
    buf += struct.pack("<b", status)
    buf = _payload_align(buf, 2)  # align for u16
    buf += struct.pack("<H", 1)  # SERVICE_GPS
    # Align payload to 8 bytes for doubles
    buf = _payload_align(buf, 8)
    # lat, lon, alt
    buf += struct.pack("<ddd", latitude, longitude, altitude)
    # position_covariance: 9 doubles (use hdop_squared for first element)
    cov = [hdop_squared] + [0.0] * 8
    buf += struct.pack("<9d", *cov)
    # covariance_type
    buf += struct.pack("<B", 2)  # COVARIANCE_TYPE_DIAGONAL_KNOWN
    return buf


def build_imu_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    accel: tuple[float, float, float] = (0.0, 0.0, 9.81),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    """Build a fake CDR-encoded Imu message."""
    # CDR header
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    # Header stamp
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    # frame_id
    frame_id = b"imu\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    # Align payload to 8 for doubles
    buf = _payload_align(buf, 8)
    # orientation (x, y, z, w)
    buf += struct.pack("<4d", *orientation)
    # orientation_covariance (9 doubles)
    buf += struct.pack("<9d", *([0.0] * 9))
    # angular_velocity (x, y, z)
    buf += struct.pack("<3d", *gyro)
    # angular_velocity_covariance (9 doubles)
    buf += struct.pack("<9d", *([0.0] * 9))
    # linear_acceleration (x, y, z)
    buf += struct.pack("<3d", *accel)
    # linear_acceleration_covariance (9 doubles)
    buf += struct.pack("<9d", *([0.0] * 9))
    return buf


@pytest.fixture
def gnss_bag(tmp_path: Path) -> Path:
    """Create a bag with 100 GNSS messages (90% fix, varying HDOP)."""
    import random

    random.seed(42)

    topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
    messages = []
    base_ns = 1_700_000_000_000_000_000  # ~2023 epoch

    for i in range(100):
        ts = base_ns + i * 100_000_000  # 100ms intervals = 10Hz
        status = 0 if i < 90 else -1  # 90% fix
        hdop_sq = (1.0 + random.random() * 2.0) ** 2
        data = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=status,
            latitude=35.6812 + random.gauss(0, 0.0001),
            longitude=139.7671 + random.gauss(0, 0.0001),
            altitude=40.0 + random.gauss(0, 2.0),
            hdop_squared=hdop_sq,
        )
        messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

    bag_path = tmp_path / "gnss.db3"
    return _create_db3(bag_path, topics, messages)


@pytest.fixture
def imu_bag(tmp_path: Path) -> Path:
    """Create a bag with 1000 IMU messages at ~200Hz."""
    import random

    random.seed(42)

    topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    for i in range(1000):
        ts = base_ns + i * 5_000_000  # 5ms intervals = 200Hz
        data = build_imu_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            accel=(
                random.gauss(0, 0.05),
                random.gauss(0, 0.05),
                9.81 + random.gauss(0, 0.05),
            ),
            gyro=(
                random.gauss(0, 0.002),
                random.gauss(0, 0.002),
                random.gauss(0, 0.002),
            ),
        )
        messages.append({"topic": "/imu", "timestamp_ns": ts, "data": data})

    bag_path = tmp_path / "imu.db3"
    return _create_db3(bag_path, topics, messages)


@pytest.fixture
def multi_bag(tmp_path: Path) -> Path:
    """Create a bag with GNSS + IMU + a generic topic for sync testing."""
    import random

    random.seed(42)

    topics = [
        {"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        {"name": "/lidar", "type": "sensor_msgs/msg/PointCloud2", "format": "cdr"},
    ]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    # GNSS at 10Hz
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

    # IMU at 100Hz
    for i in range(500):
        ts = base_ns + i * 10_000_000
        data = build_imu_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            accel=(0.01, -0.02, 9.81),
            gyro=(0.001, -0.001, 0.0),
        )
        messages.append({"topic": "/imu", "timestamp_ns": ts, "data": data})

    # LiDAR at 10Hz with slight offset (5ms jitter)
    for i in range(50):
        jitter = random.randint(-5_000_000, 5_000_000)
        ts = base_ns + i * 100_000_000 + jitter
        # Minimal fake PointCloud2 data
        data = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)  # CDR header
        data += struct.pack("<II", ts // 1_000_000_000, ts % 1_000_000_000)  # stamp
        frame_id = b"lidar\x00\x00\x00"  # padded to 4-byte align
        data += struct.pack("<I", len(frame_id))
        data += frame_id
        data += struct.pack("<II", 1, 1000)  # height=1, width=1000
        data += b"\x00" * 100  # padding
        messages.append({"topic": "/lidar", "timestamp_ns": ts, "data": data})

    # Sort by timestamp
    messages.sort(key=lambda m: m["timestamp_ns"])

    bag_path = tmp_path / "multi.db3"
    return _create_db3(bag_path, topics, messages)


@pytest.fixture
def gnss_bag_degraded(tmp_path: Path) -> Path:
    """Create a lower-quality GNSS bag for comparison tests (50% fix, high HDOP)."""
    import random

    random.seed(99)

    topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    for i in range(100):
        ts = base_ns + i * 100_000_000
        status = 0 if i < 50 else -1  # 50% fix
        hdop_sq = (3.0 + random.random() * 5.0) ** 2  # high HDOP
        data = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=status,
            latitude=35.6812 + random.gauss(0, 0.001),
            longitude=139.7671 + random.gauss(0, 0.001),
            altitude=40.0 + random.gauss(0, 10.0),
            hdop_squared=hdop_sq,
        )
        messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})

    bag_path = tmp_path / "gnss_degraded.db3"
    return _create_db3(bag_path, topics, messages)
