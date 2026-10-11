"""Tests for CDR parsers in reader.py."""

from __future__ import annotations

import struct

import pytest

from bagx.reader import (
    _parse_cdr_basic,
    _parse_header,
    _parse_imu,
    _parse_navsatfix,
    _parse_odometry,
    _parse_pointcloud2_meta,
    _parse_pose_with_covariance_stamped,
    _parse_pose_stamped,
    _parse_tf_message,
    _parse_twist_stamped,
)
from helpers import payload_align
from tests.conftest import (
    build_imu_cdr,
    build_navsatfix_cdr,
    build_odometry_cdr,
    build_pose_with_covariance_stamped_cdr,
    build_tf_message_cdr,
    build_twist_stamped_cdr,
)


def _build_pose_stamped_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> bytes:
    """Build a fake CDR-encoded PoseStamped message."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"map\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    buf = payload_align(buf, 8)
    # Pose: position (3 doubles) + orientation (4 doubles)
    buf += struct.pack("<3d", *position)
    buf += struct.pack("<4d", *orientation)
    return buf


def _build_pointcloud2_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    height: int,
    width: int,
) -> bytes:
    """Build minimal CDR-encoded PointCloud2 metadata."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"lidar\x00\x00\x00"  # padded to 4-byte align
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    # Already 4-byte aligned after frame_id
    buf += struct.pack("<II", height, width)
    buf += b"\x00" * 100  # padding for the rest
    return buf


class TestParseNavSatFix:
    """Test _parse_navsatfix with known values."""

    def test_valid_message(self):
        from helpers import build_navsatfix_cdr

        data = build_navsatfix_cdr(
            stamp_sec=1700000000, stamp_nanosec=500000000,
            status=0, latitude=35.6812, longitude=139.7671, altitude=40.0,
            hdop_squared=4.0,
        )
        result = _parse_navsatfix(data)

        assert result["stamp_sec"] == 1700000000
        assert result["stamp_nanosec"] == 500000000
        assert result["status"] == 0
        assert result["latitude"] == pytest.approx(35.6812)
        assert result["longitude"] == pytest.approx(139.7671)
        assert result["altitude"] == pytest.approx(40.0)
        assert result["position_covariance"][0] == pytest.approx(4.0)
        assert result["position_covariance_type"] == 2

    def test_truncated_data_header_only(self):
        # Only CDR header + partial stamp
        data = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00) + struct.pack("<I", 100)
        result = _parse_navsatfix(data)
        assert "_parse_error" in result or "_raw_size" in result

    def test_empty_data(self):
        result = _parse_cdr_basic(b"", "sensor_msgs/msg/NavSatFix")
        assert result["_raw_size"] == 0

    def test_field_names_for_scene(self):
        """Verify field names match what scene.py expects."""
        from helpers import build_navsatfix_cdr

        data = build_navsatfix_cdr(
            stamp_sec=100, stamp_nanosec=0,
            status=0, latitude=1.0, longitude=2.0, altitude=3.0,
        )
        result = _parse_navsatfix(data)
        # scene.py's _state_from_navsatfix expects these keys
        assert "latitude" in result
        assert "longitude" in result
        assert "altitude" in result
        assert "status" in result


class TestParseImu:
    """Test _parse_imu with known values."""

    def test_valid_message(self):
        from helpers import build_imu_cdr

        data = build_imu_cdr(
            stamp_sec=1700000000, stamp_nanosec=0,
            accel=(0.1, -0.2, 9.81),
            gyro=(0.001, -0.001, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        result = _parse_imu(data)

        assert result["stamp_sec"] == 1700000000
        assert result["orientation"]["x"] == pytest.approx(0.0)
        assert result["orientation"]["w"] == pytest.approx(1.0)
        assert result["angular_velocity"]["x"] == pytest.approx(0.001)
        assert result["angular_velocity"]["y"] == pytest.approx(-0.001)
        assert result["linear_acceleration"]["x"] == pytest.approx(0.1)
        assert result["linear_acceleration"]["y"] == pytest.approx(-0.2)
        assert result["linear_acceleration"]["z"] == pytest.approx(9.81)

    def test_truncated_imu(self):
        # Only header + stamp + frame_id (no orientation/velocity data)
        buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        buf += struct.pack("<II", 100, 0)
        frame_id = b"imu\x00"
        buf += struct.pack("<I", len(frame_id))
        buf += frame_id
        result = _parse_imu(buf)
        assert result["stamp_sec"] == 100
        # Should not have orientation/velocity since data is truncated
        assert "linear_acceleration" not in result

    def test_field_names_for_scene(self):
        """Verify field names match what scene.py expects."""
        from helpers import build_imu_cdr

        data = build_imu_cdr(stamp_sec=100, stamp_nanosec=0)
        result = _parse_imu(data)
        # scene.py's _state_from_imu expects these keys
        assert "orientation" in result
        assert "x" in result["orientation"]
        assert "angular_velocity" in result
        assert "linear_acceleration" in result


class TestParseHeader:
    """Test _parse_header."""

    def test_valid_header(self):
        buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        buf += struct.pack("<II", 1700000000, 500000000)
        result = _parse_header(buf)
        assert result["stamp_sec"] == 1700000000
        assert result["stamp_nanosec"] == 500000000

    def test_truncated_header(self):
        buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        buf += struct.pack("<I", 100)  # only 4 bytes instead of 8
        result = _parse_header(buf)
        assert "_raw_size" in result


class TestParsePoseStamped:
    """Test _parse_pose_stamped."""

    def test_valid_pose(self):
        data = _build_pose_stamped_cdr(
            stamp_sec=100, stamp_nanosec=500,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.707, 0.707),
        )
        result = _parse_pose_stamped(data)
        assert result["stamp_sec"] == 100
        assert result["stamp_nanosec"] == 500
        assert result["position"]["x"] == pytest.approx(1.0)
        assert result["position"]["y"] == pytest.approx(2.0)
        assert result["position"]["z"] == pytest.approx(3.0)
        assert result["orientation"]["z"] == pytest.approx(0.707)
        assert result["orientation"]["w"] == pytest.approx(0.707)

    def test_truncated_pose(self):
        # Only header + stamp, no pose data
        buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        buf += struct.pack("<II", 100, 0)
        buf += struct.pack("<I", 4)  # frame_id len
        buf += b"map\x00"
        result = _parse_pose_stamped(buf)
        assert result["stamp_sec"] == 100
        assert "position" not in result

    def test_field_names_for_scene(self):
        """Verify field names match what scene.py expects."""
        data = _build_pose_stamped_cdr(
            stamp_sec=100, stamp_nanosec=0,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        result = _parse_pose_stamped(data)
        # scene.py's _state_from_pose_stamped expects these keys
        assert "position" in result
        assert "x" in result["position"]
        assert "orientation" in result
        assert "w" in result["orientation"]


class TestParsePoseWithCovarianceStamped:
    """Test _parse_pose_with_covariance_stamped."""

    def test_valid_pose_with_covariance(self):
        data = build_pose_with_covariance_stamped_cdr(
            stamp_sec=100,
            stamp_nanosec=500,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.707, 0.707),
        )
        result = _parse_pose_with_covariance_stamped(data)

        assert result["stamp_sec"] == 100
        assert result["pose"]["pose"]["position"]["x"] == pytest.approx(1.0)
        assert result["pose"]["pose"]["orientation"]["w"] == pytest.approx(0.707)
        assert len(result["pose"]["covariance"]) == 36

    def test_field_names_for_scene(self):
        data = build_pose_with_covariance_stamped_cdr(
            stamp_sec=100,
            stamp_nanosec=0,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        result = _parse_pose_with_covariance_stamped(data)

        assert "pose" in result
        assert "pose" in result["pose"]
        assert "position" in result["pose"]["pose"]
        assert "orientation" in result["pose"]["pose"]


class TestParseTwistStamped:
    """Test _parse_twist_stamped."""

    def test_valid_twist(self):
        data = build_twist_stamped_cdr(
            stamp_sec=200,
            stamp_nanosec=0,
            linear=(1.0, 2.0, 3.0),
            angular=(0.1, 0.2, 0.3),
        )
        result = _parse_twist_stamped(data)

        assert result["stamp_sec"] == 200
        assert result["twist"]["linear"]["x"] == pytest.approx(1.0)
        assert result["twist"]["angular"]["z"] == pytest.approx(0.3)


class TestParseOdometry:
    """Test _parse_odometry."""

    def test_valid_odometry(self):
        data = build_odometry_cdr(
            stamp_sec=300,
            stamp_nanosec=123,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.707, 0.707),
            linear=(0.5, 0.0, 0.0),
            angular=(0.0, 0.0, 0.1),
        )
        result = _parse_odometry(data)

        assert result["stamp_sec"] == 300
        assert result["child_frame_id"] == "base_link"
        assert result["pose"]["pose"]["position"]["y"] == pytest.approx(2.0)
        assert result["twist"]["twist"]["linear"]["x"] == pytest.approx(0.5)
        assert result["twist"]["twist"]["angular"]["z"] == pytest.approx(0.1)

    def test_field_names_for_scene(self):
        data = build_odometry_cdr(
            stamp_sec=300,
            stamp_nanosec=0,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear=(0.5, 0.0, 0.0),
            angular=(0.0, 0.0, 0.1),
        )
        result = _parse_odometry(data)

        assert "pose" in result
        assert "twist" in result
        assert "pose" in result["pose"]
        assert "twist" in result["twist"]


class TestParseTfMessage:
    """Test _parse_tf_message."""

    def test_valid_tf_message(self):
        data = build_tf_message_cdr(
            stamp_sec=400,
            stamp_nanosec=0,
            translation=(1.0, 2.0, 3.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )
        result = _parse_tf_message(data)

        assert len(result["transforms"]) == 1
        transform = result["transforms"][0]
        assert transform["frame_id"] == "map"
        assert transform["child_frame_id"] == "base_link"
        assert transform["transform"]["translation"]["x"] == pytest.approx(1.0)
        assert transform["transform"]["rotation"]["w"] == pytest.approx(1.0)


class TestParsePointCloud2:
    """Test _parse_pointcloud2_meta."""

    def test_valid_pointcloud2(self):
        data = _build_pointcloud2_cdr(
            stamp_sec=100, stamp_nanosec=0,
            height=1, width=1000,
        )
        result = _parse_pointcloud2_meta(data)
        assert result["stamp_sec"] == 100
        assert result["height"] == 1
        assert result["width"] == 1000
        assert result["num_points"] == 1000

    def test_truncated_pointcloud2(self):
        buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        buf += struct.pack("<II", 100, 0)
        # No frame_id length field - too short
        result = _parse_pointcloud2_meta(buf)
        assert result["stamp_sec"] == 100
        assert "_parse_error" in result


class TestParseCdrBasicDispatch:
    """Test _parse_cdr_basic dispatch and fallback."""

    def test_unknown_type_returns_raw_size(self):
        data = b"\x00\x01\x00\x00" + b"\x00" * 50
        result = _parse_cdr_basic(data, "custom_msgs/msg/Unknown")
        assert result["_raw_size"] == 54
        assert result["_msg_type"] == "custom_msgs/msg/Unknown"

    def test_none_data(self):
        result = _parse_cdr_basic(None, "sensor_msgs/msg/NavSatFix")
        assert result["_raw_size"] == 0

    def test_known_type_dispatches(self):
        from helpers import build_navsatfix_cdr

        data = build_navsatfix_cdr(
            stamp_sec=100, stamp_nanosec=0,
            status=0, latitude=1.0, longitude=2.0, altitude=3.0,
        )
        result = _parse_cdr_basic(data, "sensor_msgs/msg/NavSatFix")
        assert "latitude" in result
        assert result["latitude"] == pytest.approx(1.0)

    def test_odometry_dispatches(self):
        data = build_odometry_cdr(
            stamp_sec=100,
            stamp_nanosec=0,
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear=(0.5, 0.0, 0.0),
            angular=(0.0, 0.0, 0.1),
        )
        result = _parse_cdr_basic(data, "nav_msgs/msg/Odometry")

        assert "pose" in result
        assert result["pose"]["pose"]["position"]["x"] == pytest.approx(1.0)


def test_header_first_parsers_expose_frame_id() -> None:
    from bagx.reader import _parse_imu, _parse_navsatfix

    imu = _parse_imu(build_imu_cdr(1, 2, frame_id="imu_link"))
    gps = _parse_navsatfix(build_navsatfix_cdr(1, 2, 0, 35.0, 139.0, 10.0))
    assert imu["frame_id"] == "imu_link"
    assert gps["frame_id"] == "gps"


def test_parse_laser_scan_counts_valid_ranges() -> None:
    from bagx.reader import _parse_cdr_basic
    from tests.conftest import build_laser_scan_cdr

    ranges = [1.0, 2.0, float("inf"), float("nan"), 0.05, 50.0, 3.0]
    msg = _parse_cdr_basic(build_laser_scan_cdr(5, 6, ranges, frame_id="base_scan"),
                           "sensor_msgs/msg/LaserScan")
    assert msg["frame_id"] == "base_scan"
    assert msg["stamp_sec"] == 5
    assert msg["num_ranges"] == 7
    assert msg["num_valid_ranges"] == 3
    assert msg["range_max"] == pytest.approx(30.0)
    assert "_parse_error" not in msg


def test_parse_camera_info() -> None:
    from bagx.reader import _parse_cdr_basic
    from tests.conftest import build_camera_info_cdr

    msg = _parse_cdr_basic(build_camera_info_cdr(1, 2, width=1280, height=720, d=[0.1, 0.2]),
                           "sensor_msgs/msg/CameraInfo")
    assert (msg["width"], msg["height"]) == (1280, 720)
    assert msg["distortion_model"] == "plumb_bob"
    assert msg["d"] == pytest.approx([0.1, 0.2])
    assert msg["k"][0] == pytest.approx(500.0)
    assert len(msg["p"]) == 12
    assert "_parse_error" not in msg


def test_parse_image_meta_skips_pixels() -> None:
    from bagx.reader import _parse_cdr_basic
    from tests.conftest import build_image_cdr

    msg = _parse_cdr_basic(build_image_cdr(1, 2, width=4, height=2, encoding="bgr8"),
                           "sensor_msgs/msg/Image")
    assert (msg["width"], msg["height"], msg["encoding"]) == (4, 2, "bgr8")
    assert msg["step"] == 12
    assert msg["data_size"] == 24
    assert "data" not in msg


def test_parse_joint_state() -> None:
    from bagx.reader import _parse_cdr_basic
    from tests.conftest import build_joint_state_cdr

    msg = _parse_cdr_basic(build_joint_state_cdr(1, 2, ["j1", "joint_two"], [0.5, -1.0]),
                           "sensor_msgs/msg/JointState")
    assert msg["name"] == ["j1", "joint_two"]
    assert msg["position"] == pytest.approx([0.5, -1.0])
    assert msg["velocity"] == [] and msg["effort"] == []


@pytest.mark.parametrize("msg_type", [
    "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/Image", "sensor_msgs/msg/JointState",
])
def test_new_parsers_flag_truncated_payloads(msg_type: str) -> None:
    from bagx.reader import _parse_cdr_basic
    from tests.conftest import (
        build_camera_info_cdr, build_image_cdr, build_joint_state_cdr, build_laser_scan_cdr,
    )

    full = {
        "sensor_msgs/msg/LaserScan": build_laser_scan_cdr(1, 2, [1.0] * 10),
        "sensor_msgs/msg/CameraInfo": build_camera_info_cdr(1, 2),
        "sensor_msgs/msg/Image": build_image_cdr(1, 2, width=2, height=2),
        "sensor_msgs/msg/JointState": build_joint_state_cdr(1, 2, ["a", "b"]),
    }[msg_type]
    msg = _parse_cdr_basic(full[: len(full) // 2 + 10], msg_type)
    assert msg.get("_parse_error") is True
