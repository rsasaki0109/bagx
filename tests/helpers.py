"""Shared test helper functions.

These are extracted from conftest.py so they can be imported by test modules directly.
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path


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
