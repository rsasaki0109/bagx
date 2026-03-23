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


def build_pose_with_covariance_stamped_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> bytes:
    """Build a fake CDR-encoded PoseWithCovarianceStamped message."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"map\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    buf = _payload_align(buf, 8)
    buf += struct.pack("<3d", *position)
    buf += struct.pack("<4d", *orientation)
    buf += struct.pack("<36d", *([0.0] * 36))
    return buf


def build_twist_stamped_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
    angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bytes:
    """Build a fake CDR-encoded TwistStamped message."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"base_link\x00\x00\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    buf = _payload_align(buf, 8)
    buf += struct.pack("<3d", *linear)
    buf += struct.pack("<3d", *angular)
    return buf


def build_odometry_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    linear: tuple[float, float, float],
    angular: tuple[float, float, float],
    child_frame_id: str = "base_link",
) -> bytes:
    """Build a fake CDR-encoded Odometry message."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame_id = b"odom\x00"
    buf += struct.pack("<I", len(frame_id))
    buf += frame_id
    child = child_frame_id.encode() + b"\x00"
    buf = _payload_align(buf, 4)
    buf += struct.pack("<I", len(child))
    buf += child
    buf = _payload_align(buf, 8)
    buf += struct.pack("<3d", *position)
    buf += struct.pack("<4d", *orientation)
    buf += struct.pack("<36d", *([0.0] * 36))
    buf += struct.pack("<3d", *linear)
    buf += struct.pack("<3d", *angular)
    buf += struct.pack("<36d", *([0.0] * 36))
    return buf


def build_tf_message_cdr(
    stamp_sec: int,
    stamp_nanosec: int,
    translation: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
    frame_id: str = "map",
    child_frame_id: str = "base_link",
) -> bytes:
    """Build a fake CDR-encoded TFMessage with a single transform."""
    buf = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
    buf += struct.pack("<I", 1)  # transforms length
    buf += struct.pack("<II", stamp_sec, stamp_nanosec)
    frame = frame_id.encode() + b"\x00"
    buf += struct.pack("<I", len(frame))
    buf += frame
    child = child_frame_id.encode() + b"\x00"
    buf = _payload_align(buf, 4)
    buf += struct.pack("<I", len(child))
    buf += child
    buf = _payload_align(buf, 8)
    buf += struct.pack("<3d", *translation)
    buf += struct.pack("<4d", *rotation)
    return buf


def build_stub_cdr() -> bytes:
    """Build a minimal generic CDR payload for unknown message types."""
    return struct.pack("<4B", 0x00, 0x01, 0x00, 0x00) + b"\x00" * 32


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


@pytest.fixture
def nav2_bag(tmp_path: Path) -> Path:
    """Create a namespaced Nav2-style bag with odom/scan/costmap/plan/cmd_vel."""
    topics = [
        {"name": "/robot/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"},
        {"name": "/robot/scan", "type": "sensor_msgs/msg/LaserScan", "format": "cdr"},
        {"name": "/robot/amcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "format": "cdr"},
        {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "format": "cdr"},
        {"name": "/global_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "format": "cdr"},
        {"name": "/plan", "type": "nav_msgs/msg/Path", "format": "cdr"},
        {"name": "/navigate_to_pose/_action/status", "type": "action_msgs/msg/GoalStatusArray", "format": "cdr"},
        {"name": "/controller_server/cmd_vel", "type": "geometry_msgs/msg/TwistStamped", "format": "cdr"},
    ]
    messages = []
    base_ns = 1_700_000_000_000_000_000

    for i in range(100):
        ts = base_ns + i * 20_000_000  # 50Hz
        data = build_odometry_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            position=(i * 0.02, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear=(0.5, 0.0, 0.0),
            angular=(0.0, 0.0, 0.05),
        )
        messages.append({"topic": "/robot/odom", "timestamp_ns": ts, "data": data})
        amcl_ts = ts + 5_000_000
        amcl = build_pose_with_covariance_stamped_cdr(
            stamp_sec=amcl_ts // 1_000_000_000,
            stamp_nanosec=amcl_ts % 1_000_000_000,
            position=(i * 0.02, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        messages.append({"topic": "/robot/amcl_pose", "timestamp_ns": amcl_ts, "data": amcl})

    for i in range(48):
        scan_ts = base_ns + i * 83_333_333  # ~12Hz
        messages.append({"topic": "/robot/scan", "timestamp_ns": scan_ts, "data": build_stub_cdr()})
        costmap_ts = scan_ts + 30_000_000
        messages.append({"topic": "/local_costmap/costmap", "timestamp_ns": costmap_ts, "data": build_stub_cdr()})

    for i in range(4):
        global_costmap_ts = base_ns + i * 1_000_000_000
        messages.append({"topic": "/global_costmap/costmap", "timestamp_ns": global_costmap_ts, "data": build_stub_cdr()})
        plan_ts = global_costmap_ts + 40_000_000
        messages.append({"topic": "/plan", "timestamp_ns": plan_ts, "data": build_stub_cdr()})
        messages.append({
            "topic": "/navigate_to_pose/_action/status",
            "timestamp_ns": plan_ts + 20_000_000,
            "data": build_stub_cdr(),
        })

    for i in range(40):
        cmd_ts = base_ns + i * 50_000_000  # 20Hz
        cmd = build_twist_stamped_cdr(
            stamp_sec=cmd_ts // 1_000_000_000,
            stamp_nanosec=cmd_ts % 1_000_000_000,
            linear=(0.4, 0.0, 0.0),
            angular=(0.0, 0.0, 0.1),
        )
        messages.append({"topic": "/controller_server/cmd_vel", "timestamp_ns": cmd_ts, "data": cmd})

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(tmp_path / "nav2.db3", topics, messages)


@pytest.fixture
def moveit_sparse_bag(tmp_path: Path) -> Path:
    """Create a MoveIt-style bag with a single planned path output."""
    topics = [
        {"name": "/joint_states", "type": "sensor_msgs/msg/JointState", "format": "cdr"},
        {"name": "/display_planned_path", "type": "moveit_msgs/msg/DisplayTrajectory", "format": "cdr"},
    ]
    messages = []
    base_ns = 1_700_000_250_000_000_000

    for i in range(60):
        ts = base_ns + i * 10_000_000  # 100Hz
        messages.append({"topic": "/joint_states", "timestamp_ns": ts, "data": build_stub_cdr()})

    messages.append({
        "topic": "/display_planned_path",
        "timestamp_ns": base_ns + 450_000_000,
        "data": build_stub_cdr(),
    })

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(tmp_path / "moveit_sparse.db3", topics, messages)


@pytest.fixture
def autoware_bag(tmp_path: Path) -> Path:
    """Create an Autoware-style bag with sensing/perception/planning/control."""
    topics = [
        {"name": "/sensing/lidar/top/pointcloud_raw_ex", "type": "sensor_msgs/msg/PointCloud2", "format": "cdr"},
        {"name": "/sensing/camera/front/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "format": "cdr"},
        {"name": "/sensing/gnss/ublox/nav_sat_fix", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/perception/object_recognition/objects", "type": "autoware_auto_perception_msgs/msg/TrackedObjects", "format": "cdr"},
        {"name": "/planning/scenario_planning/lane_driving/trajectory", "type": "autoware_auto_planning_msgs/msg/Trajectory", "format": "cdr"},
        {"name": "/control/command/control_cmd", "type": "autoware_auto_control_msgs/msg/AckermannControlCommand", "format": "cdr"},
    ]
    messages = []
    base_ns = 1_700_000_100_000_000_000

    for i in range(30):
        ts = base_ns + i * 33_333_333  # ~30Hz
        messages.append({
            "topic": "/sensing/camera/front/image_raw/compressed",
            "timestamp_ns": ts,
            "data": build_stub_cdr(),
        })

    for i in range(20):
        lidar_ts = base_ns + i * 100_000_000  # 10Hz
        messages.append({
            "topic": "/sensing/lidar/top/pointcloud_raw_ex",
            "timestamp_ns": lidar_ts,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/perception/object_recognition/objects",
            "timestamp_ns": lidar_ts + 30_000_000,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/planning/scenario_planning/lane_driving/trajectory",
            "timestamp_ns": lidar_ts + 70_000_000,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/control/command/control_cmd",
            "timestamp_ns": lidar_ts + 110_000_000,
            "data": build_stub_cdr(),
        })
        gnss = build_navsatfix_cdr(
            stamp_sec=lidar_ts // 1_000_000_000,
            stamp_nanosec=lidar_ts % 1_000_000_000,
            status=0,
            latitude=35.6812,
            longitude=139.7671,
            altitude=40.0,
        )
        messages.append({
            "topic": "/sensing/gnss/ublox/nav_sat_fix",
            "timestamp_ns": lidar_ts,
            "data": gnss,
        })

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(tmp_path / "autoware.db3", topics, messages)


@pytest.fixture
def autoware_sensing_only_bag(tmp_path: Path) -> Path:
    """Create an Autoware-style sensing bag without planning/control topics."""
    topics = [
        {"name": "/sensing/lidar/top/pointcloud_raw_ex", "type": "sensor_msgs/msg/PointCloud2", "format": "cdr"},
        {"name": "/sensing/camera/front/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage", "format": "cdr"},
        {"name": "/sensing/gnss/ublox/nav_sat_fix", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/localization/kinematic_state", "type": "nav_msgs/msg/Odometry", "format": "cdr"},
    ]
    messages = []
    base_ns = 1_700_000_150_000_000_000

    for i in range(30):
        cam_ts = base_ns + i * 50_000_000  # 20Hz
        messages.append({
            "topic": "/sensing/camera/front/image_raw/compressed",
            "timestamp_ns": cam_ts,
            "data": build_stub_cdr(),
        })

    for i in range(20):
        lidar_ts = base_ns + i * 100_000_000  # 10Hz
        messages.append({
            "topic": "/sensing/lidar/top/pointcloud_raw_ex",
            "timestamp_ns": lidar_ts,
            "data": build_stub_cdr(),
        })
        gnss = build_navsatfix_cdr(
            stamp_sec=lidar_ts // 1_000_000_000,
            stamp_nanosec=lidar_ts % 1_000_000_000,
            status=0,
            latitude=35.6812,
            longitude=139.7671,
            altitude=40.0,
        )
        messages.append({
            "topic": "/sensing/gnss/ublox/nav_sat_fix",
            "timestamp_ns": lidar_ts,
            "data": gnss,
        })
        odom = build_odometry_cdr(
            stamp_sec=lidar_ts // 1_000_000_000,
            stamp_nanosec=lidar_ts % 1_000_000_000,
            position=(i * 0.2, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear=(2.0, 0.0, 0.0),
            angular=(0.0, 0.0, 0.0),
        )
        messages.append({
            "topic": "/localization/kinematic_state",
            "timestamp_ns": lidar_ts + 20_000_000,
            "data": odom,
        })

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(tmp_path / "autoware_sensing_only.db3", topics, messages)


@pytest.fixture
def moveit_bag(tmp_path: Path) -> Path:
    """Create a MoveIt-style bag with namespaced joint_states, planning, and execution topics."""
    topics = [
        {"name": "/fr3/joint_states", "type": "sensor_msgs/msg/JointState", "format": "cdr"},
        {"name": "/move_group/display_planned_path", "type": "moveit_msgs/msg/DisplayTrajectory", "format": "cdr"},
        {"name": "/move_group/monitored_planning_scene", "type": "moveit_msgs/msg/PlanningScene", "format": "cdr"},
        {"name": "/move_action/_action/status", "type": "action_msgs/msg/GoalStatusArray", "format": "cdr"},
        {"name": "/execute_trajectory/_action/status", "type": "action_msgs/msg/GoalStatusArray", "format": "cdr"},
        {"name": "/panda_arm_controller/follow_joint_trajectory/_action/status", "type": "action_msgs/msg/GoalStatusArray", "format": "cdr"},
    ]
    messages = []
    base_ns = 1_700_000_200_000_000_000

    for i in range(200):
        ts = base_ns + i * 5_000_000  # 200Hz
        messages.append({"topic": "/fr3/joint_states", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(20):
        ts = base_ns + i * 100_000_000
        messages.append({
            "topic": "/move_group/display_planned_path",
            "timestamp_ns": ts + 40_000_000,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/move_group/monitored_planning_scene",
            "timestamp_ns": ts,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/move_action/_action/status",
            "timestamp_ns": ts + 20_000_000,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/execute_trajectory/_action/status",
            "timestamp_ns": ts + 60_000_000,
            "data": build_stub_cdr(),
        })
        messages.append({
            "topic": "/panda_arm_controller/follow_joint_trajectory/_action/status",
            "timestamp_ns": ts + 90_000_000,
            "data": build_stub_cdr(),
        })

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(tmp_path / "moveit.db3", topics, messages)


@pytest.fixture
def tf_bag(tmp_path: Path) -> Path:
    """Create a bag with TFMessage for scene extraction tests."""
    topics = [{"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "format": "cdr"}]
    messages = []
    base_ns = 1_700_000_300_000_000_000

    for i in range(10):
        ts = base_ns + i * 100_000_000
        data = build_tf_message_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            translation=(i * 0.1, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )
        messages.append({"topic": "/tf", "timestamp_ns": ts, "data": data})

    return _create_db3(tmp_path / "tf.db3", topics, messages)
