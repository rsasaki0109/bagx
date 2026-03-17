"""Tests for CDR endianness handling in reader.py."""

from __future__ import annotations

import struct

import pytest

from bagx.reader import _parse_imu, _parse_navsatfix, _read_cdr_header
from helpers import payload_align


def _build_navsatfix_cdr_big_endian(
    stamp_sec: int,
    stamp_nanosec: int,
    status: int,
    latitude: float,
    longitude: float,
    altitude: float,
    hdop_squared: float = 1.0,
) -> bytes:
    """Build a big-endian CDR-encoded NavSatFix message."""
    # CDR header (big-endian: byte 1 = 0x00)
    buf = struct.pack(">4B", 0x00, 0x00, 0x00, 0x00)
    # Header stamp
    buf += struct.pack(">II", stamp_sec, stamp_nanosec)
    # frame_id
    frame_id = b"gps\x00"
    buf += struct.pack(">I", len(frame_id))
    buf += frame_id
    # NavSatStatus: status (i8) + padding + service (u16)
    buf += struct.pack(">b", status)
    buf = payload_align(buf, 2)
    buf += struct.pack(">H", 1)  # SERVICE_GPS
    buf = payload_align(buf, 8)
    # lat, lon, alt
    buf += struct.pack(">ddd", latitude, longitude, altitude)
    # position_covariance: 9 doubles
    cov = [hdop_squared] + [0.0] * 8
    buf += struct.pack(">9d", *cov)
    # covariance_type
    buf += struct.pack(">B", 2)
    return buf


def _build_imu_cdr_big_endian(
    stamp_sec: int,
    stamp_nanosec: int,
    accel: tuple[float, float, float] = (0.0, 0.0, 9.81),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    """Build a big-endian CDR-encoded Imu message."""
    buf = struct.pack(">4B", 0x00, 0x00, 0x00, 0x00)
    buf += struct.pack(">II", stamp_sec, stamp_nanosec)
    frame_id = b"imu\x00"
    buf += struct.pack(">I", len(frame_id))
    buf += frame_id
    buf = payload_align(buf, 8)
    # orientation (x, y, z, w)
    buf += struct.pack(">4d", *orientation)
    # orientation_covariance (9 doubles)
    buf += struct.pack(">9d", *([0.0] * 9))
    # angular_velocity (x, y, z)
    buf += struct.pack(">3d", *gyro)
    # angular_velocity_covariance (9 doubles)
    buf += struct.pack(">9d", *([0.0] * 9))
    # linear_acceleration (x, y, z)
    buf += struct.pack(">3d", *accel)
    # linear_acceleration_covariance (9 doubles)
    buf += struct.pack(">9d", *([0.0] * 9))
    return buf


class TestReadCdrHeader:
    """Test _read_cdr_header endianness detection."""

    def test_little_endian_header(self):
        data = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        offset, endian = _read_cdr_header(data)
        assert offset == 4
        assert endian == "<"

    def test_big_endian_header(self):
        data = struct.pack(">4B", 0x00, 0x00, 0x00, 0x00)
        offset, endian = _read_cdr_header(data)
        assert offset == 4
        assert endian == ">"

    def test_minimal_data_returns_default(self):
        # Less than 4 bytes should return default little-endian
        data = b"\x00\x01"
        offset, endian = _read_cdr_header(data)
        assert offset == 0
        assert endian == "<"

    def test_empty_data_returns_default(self):
        offset, endian = _read_cdr_header(b"")
        assert offset == 0
        assert endian == "<"

    def test_exactly_three_bytes_returns_default(self):
        offset, endian = _read_cdr_header(b"\x00\x01\x00")
        assert offset == 0
        assert endian == "<"


class TestNavSatFixEndianness:
    """Test NavSatFix parsing with both endiannesses."""

    def test_little_endian_parsing(self):
        from helpers import build_navsatfix_cdr

        data = build_navsatfix_cdr(
            stamp_sec=100, stamp_nanosec=500,
            status=0, latitude=35.68, longitude=139.77, altitude=42.0,
            hdop_squared=4.0,
        )
        result = _parse_navsatfix(data)
        assert result["stamp_sec"] == 100
        assert result["stamp_nanosec"] == 500
        assert result["status"] == 0
        assert result["latitude"] == pytest.approx(35.68)
        assert result["longitude"] == pytest.approx(139.77)
        assert result["altitude"] == pytest.approx(42.0)

    def test_big_endian_parsing(self):
        data = _build_navsatfix_cdr_big_endian(
            stamp_sec=200, stamp_nanosec=999,
            status=0, latitude=48.85, longitude=2.35, altitude=100.0,
            hdop_squared=9.0,
        )
        result = _parse_navsatfix(data)
        assert result["stamp_sec"] == 200
        assert result["stamp_nanosec"] == 999
        assert result["status"] == 0
        assert result["latitude"] == pytest.approx(48.85)
        assert result["longitude"] == pytest.approx(2.35)
        assert result["altitude"] == pytest.approx(100.0)
        assert result["position_covariance"][0] == pytest.approx(9.0)


class TestImuEndianness:
    """Test IMU parsing with big-endian CDR."""

    def test_big_endian_imu_parsing(self):
        data = _build_imu_cdr_big_endian(
            stamp_sec=300, stamp_nanosec=123,
            accel=(0.1, -0.2, 9.81),
            gyro=(0.01, -0.01, 0.005),
            orientation=(0.0, 0.0, 0.707, 0.707),
        )
        result = _parse_imu(data)
        assert result["stamp_sec"] == 300
        assert result["stamp_nanosec"] == 123
        assert result["linear_acceleration"]["x"] == pytest.approx(0.1)
        assert result["linear_acceleration"]["y"] == pytest.approx(-0.2)
        assert result["linear_acceleration"]["z"] == pytest.approx(9.81)
        assert result["angular_velocity"]["x"] == pytest.approx(0.01)
        assert result["angular_velocity"]["y"] == pytest.approx(-0.01)
        assert result["angular_velocity"]["z"] == pytest.approx(0.005)
        assert result["orientation"]["z"] == pytest.approx(0.707)
        assert result["orientation"]["w"] == pytest.approx(0.707)

    def test_little_endian_imu_parsing(self):
        from helpers import build_imu_cdr

        data = build_imu_cdr(
            stamp_sec=400, stamp_nanosec=0,
            accel=(1.0, 2.0, 9.81),
            gyro=(0.0, 0.0, 0.1),
        )
        result = _parse_imu(data)
        assert result["stamp_sec"] == 400
        assert result["linear_acceleration"]["x"] == pytest.approx(1.0)
        assert result["linear_acceleration"]["z"] == pytest.approx(9.81)
        assert result["angular_velocity"]["z"] == pytest.approx(0.1)
