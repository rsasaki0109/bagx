"""Shared test helper functions.

These are extracted from conftest.py so they can be imported by test modules directly.
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path

import numpy as np

try:
    from rosbags.rosbag1 import Writer as Rosbag1Writer
    from rosbags.typesys import Stores, get_typestore

    HAS_ROSBAGS = True
except ImportError:
    HAS_ROSBAGS = False


def create_db3(path: Path, topics: list[dict], messages: list[dict]) -> Path:
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


def payload_align(buf: bytes, alignment: int, cdr_header_size: int = 4) -> bytes:
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
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"gps\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    buf += struct.pack("<b", status)
    buf = payload_align(buf, 2)
    buf += struct.pack("<H", 1)
    buf = payload_align(buf, 8)
    buf += struct.pack("<ddd", latitude, longitude, altitude)
    cov = [hdop_squared] + [0.0] * 8
    buf += struct.pack("<9d", *cov)
    buf += struct.pack("<B", 2)
    return buf


def build_imu_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    accel: tuple[float, float, float] = (0.0, 0.0, 9.81),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    """Build a fake CDR-encoded Imu message."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"imu\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    buf = payload_align(buf, 8)
    buf += struct.pack("<4d", *orientation)
    buf += struct.pack("<9d", *([0.0] * 9))
    buf += struct.pack("<3d", *gyro)
    buf += struct.pack("<9d", *([0.0] * 9))
    buf += struct.pack("<3d", *accel)
    buf += struct.pack("<9d", *([0.0] * 9))
    return buf


def _ros1_typestore():
    if not HAS_ROSBAGS:
        raise RuntimeError("rosbags is required for ROS1 test helpers")
    return get_typestore(Stores.ROS1_NOETIC)


def create_ros1_bag(
    path: Path,
    topics: list[dict],
    messages: list[dict],
) -> Path:
    """Create a minimal ROS1 .bag file using rosbags.

    Args:
        path: Output .bag path.
        topics: List of {"name": str, "type": str}.
        messages: List of {"topic": str, "timestamp_ns": int, "fields": dict}.
            ``fields`` holds message field values for the supported types below.
    """
    if not HAS_ROSBAGS:
        raise RuntimeError("rosbags is required for ROS1 test helpers")

    typestore = _ros1_typestore()
    types = typestore.types
    Time = types["builtin_interfaces/msg/Time"]
    Header = types["std_msgs/msg/Header"]
    Vector3 = types["geometry_msgs/msg/Vector3"]
    Quaternion = types["geometry_msgs/msg/Quaternion"]
    NavSatStatus = types["sensor_msgs/msg/NavSatStatus"]
    NavSatFix = types["sensor_msgs/msg/NavSatFix"]
    Imu = types["sensor_msgs/msg/Imu"]
    zero_cov = np.zeros(9, dtype=np.float64)

    connections: dict[str, object] = {}
    with Rosbag1Writer(path) as writer:
        for topic_info in topics:
            connections[topic_info["name"]] = writer.add_connection(
                topic_info["name"],
                topic_info["type"],
                typestore=typestore,
            )

        for msg in messages:
            topic = msg["topic"]
            ts_ns = msg["timestamp_ns"]
            fields = msg.get("fields", {})
            msg_type = next(t["type"] for t in topics if t["name"] == topic)
            stamp_sec = fields.get("stamp_sec", ts_ns // 1_000_000_000)
            stamp_nanosec = fields.get("stamp_nanosec", ts_ns % 1_000_000_000)
            stamp = Time(sec=stamp_sec, nanosec=stamp_nanosec)

            if msg_type == "sensor_msgs/msg/NavSatFix":
                header = Header(
                    seq=fields.get("seq", 0),
                    stamp=stamp,
                    frame_id=fields.get("frame_id", "gps"),
                )
                status = NavSatStatus(
                    status=fields.get("status", 0),
                    service=fields.get("service", 1),
                )
                ros_msg = NavSatFix(
                    header=header,
                    status=status,
                    latitude=fields.get("latitude", 0.0),
                    longitude=fields.get("longitude", 0.0),
                    altitude=fields.get("altitude", 0.0),
                    position_covariance=fields.get("position_covariance", zero_cov),
                    position_covariance_type=fields.get("position_covariance_type", 0),
                )
            elif msg_type == "sensor_msgs/msg/Imu":
                header = Header(
                    seq=fields.get("seq", 0),
                    stamp=stamp,
                    frame_id=fields.get("frame_id", "imu"),
                )
                accel = fields.get("linear_acceleration", (0.0, 0.0, 9.81))
                gyro = fields.get("angular_velocity", (0.0, 0.0, 0.0))
                orientation = fields.get("orientation", (0.0, 0.0, 0.0, 1.0))
                ros_msg = Imu(
                    header=header,
                    orientation=Quaternion(
                        x=orientation[0],
                        y=orientation[1],
                        z=orientation[2],
                        w=orientation[3],
                    ),
                    orientation_covariance=zero_cov,
                    angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                    angular_velocity_covariance=zero_cov,
                    linear_acceleration=Vector3(x=accel[0], y=accel[1], z=accel[2]),
                    linear_acceleration_covariance=zero_cov,
                )
            else:
                raise ValueError(f"Unsupported ROS1 test message type: {msg_type}")

            raw = typestore.serialize_ros1(ros_msg, msg_type)
            writer.write(connections[topic], ts_ns, raw)

    return path
