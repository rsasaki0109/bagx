"""The rosbag2_py backend must produce the same eval result as the SQLite fallback.

Runs only where ROS 2 (rosbag2_py + rclpy) is importable, i.e. in the ROS CI job.
"""

from __future__ import annotations

from pathlib import Path

import pytest

import bagx.reader as reader_mod
from bagx.eval import evaluate_bag

pytestmark = pytest.mark.skipif(not reader_mod.HAS_ROS, reason="ROS 2 (rosbag2_py) not available")

FIXTURES = [
    "gnss_bag", "imu_bag", "multi_bag", "gnss_bag_degraded", "nav2_bag", "moveit_bag",
    "moveit_sparse_bag", "autoware_bag", "autoware_sensing_only_bag",
    "autoware_vehicle_status_bag", "robotarm_bag", "perception_bag",
    "generic_control_bag", "tf_bag",
]


def _comparable(path: str) -> dict:
    data = evaluate_bag(path).to_dict()
    return {
        "overall_score": round(data["overall_score"], 6),
        "domain_score": None if data["domain_score"] is None else round(data["domain_score"], 6),
        "topic_count": data["topic_count"],
        "total_messages": data["total_messages"],
        "finding_ids": sorted(f["id"] for f in data["findings"]),
        "recommendations": data["recommendations"],
        "tf": data.get("tf"),
    }


@pytest.mark.parametrize("fixture_name", FIXTURES)
def test_ros_backend_matches_sqlite_fallback(
    fixture_name: str, request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch
) -> None:
    bag = str(request.getfixturevalue(fixture_name))

    calls = {"ros": 0}
    original = reader_mod.BagReader._read_ros

    def counting_read_ros(self, topics):
        calls["ros"] += 1
        yield from original(self, topics)

    monkeypatch.setattr(reader_mod.BagReader, "_read_ros", counting_read_ros)
    via_ros = _comparable(bag)
    assert calls["ros"] > 0, "rosbag2_py backend was not used"

    monkeypatch.setattr(reader_mod, "HAS_ROS", False)
    via_sqlite = _comparable(bag)

    assert via_ros == via_sqlite


def test_ros_messages_are_decoded_not_fallback_parsed(gnss_bag: Path) -> None:
    """rclpy-decoded dicts carry the nested header; the CDR fallback does not."""
    msg = next(reader_mod.BagReader(gnss_bag).read_messages())
    assert isinstance(msg.data.get("header"), dict)
    assert msg.data["header"]["frame_id"] == "gps"
    # Normalized to the same flat fields the evaluators read.
    assert isinstance(msg.data["status"], int)
    assert "stamp_sec" in msg.data


def _write_real_bag(path: Path) -> None:
    """Write a bag with rosbag2_py + rclpy serialization (not the test CDR builders)."""
    import math

    import rosbag2_py
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, NavSatFix
    from tf2_msgs.msg import TFMessage

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {
        "/imu": "sensor_msgs/msg/Imu", "/gnss/fix": "sensor_msgs/msg/NavSatFix",
        "/scan": "sensor_msgs/msg/LaserScan", "/joint_states": "sensor_msgs/msg/JointState",
        "/camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
        "/camera/color/image_raw": "sensor_msgs/msg/Image", "/odom": "nav_msgs/msg/Odometry",
        "/amcl_pose": "geometry_msgs/msg/PoseWithCovarianceStamped", "/tf": "tf2_msgs/msg/TFMessage",
    }
    for i, (name, msg_type) in enumerate(types.items()):
        try:  # Jazzy and later take a leading topic id
            metadata = rosbag2_py.TopicMetadata(i, name, msg_type, "cdr")
        except TypeError:  # Humble: TopicMetadata(name, type, serialization_format)
            metadata = rosbag2_py.TopicMetadata(name, msg_type, "cdr")
        writer.create_topic(metadata)

    base = 1_700_000_000_000_000_000
    for i in range(200):
        t = base + i * 10_000_000
        stamp = Time(sec=t // 1_000_000_000, nanosec=t % 1_000_000_000)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "imu_link"
        imu.linear_acceleration.z = 9.81 + 0.01 * math.sin(i)
        imu.angular_velocity.x = 0.001 * math.cos(i)
        imu.orientation.w = 1.0
        writer.write("/imu", serialize_message(imu), t)
        if i % 2 == 0:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "odom", "base_link"
            odom.pose.pose.orientation.w = 1.0
            writer.write("/odom", serialize_message(odom), t)
            tf = TFMessage(transforms=[TransformStamped()])
            tf.transforms[0].header.stamp, tf.transforms[0].header.frame_id = stamp, "odom"
            tf.transforms[0].child_frame_id = "base_link"
            tf.transforms[0].transform.rotation.w = 1.0
            writer.write("/tf", serialize_message(tf), t)
        if i % 10 == 0:
            fix = NavSatFix(latitude=35.0 + i * 1e-6, longitude=139.0, altitude=10.0)
            fix.header.stamp, fix.header.frame_id = stamp, "gnss_link"
            fix.status.status = 0 if i % 30 else -1
            fix.position_covariance = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 4.0]
            writer.write("/gnss/fix", serialize_message(fix), t)
            scan = LaserScan(angle_min=-1.5, angle_max=1.5, angle_increment=0.01,
                             range_min=0.1, range_max=10.0,
                             ranges=[1.0 if k % 3 else float("inf") for k in range(300)])
            scan.header.stamp, scan.header.frame_id = stamp, "laser"
            writer.write("/scan", serialize_message(scan), t + 1_000_000)
            pose = PoseWithCovarianceStamped()
            pose.header.stamp, pose.header.frame_id = stamp, "map"
            pose.pose.pose.orientation.w = 1.0
            writer.write("/amcl_pose", serialize_message(pose), t + 30_000_000)
            joints = JointState(name=["a", "b"], position=[0.1 * i, -0.1 * i])
            joints.header.stamp = stamp
            writer.write("/joint_states", serialize_message(joints), t)
            info = CameraInfo(width=4, height=2, distortion_model="plumb_bob",
                              d=[0.0] * 5, k=[500.0, 0, 2, 0, 500.0, 1, 0, 0, 1])
            info.header.stamp, info.header.frame_id = stamp, "camera_optical"
            writer.write("/camera/color/camera_info", serialize_message(info), t)
            image = Image(width=4, height=2, encoding="rgb8", step=12, data=bytes(24))
            image.header.stamp, image.header.frame_id = stamp, "camera_optical"
            writer.write("/camera/color/image_raw", serialize_message(image), t)
    del writer


def test_rclpy_written_bag_matches_across_backends(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    bag_dir = tmp_path / "real_bag"
    _write_real_bag(bag_dir)

    via_ros = _comparable(str(bag_dir))
    monkeypatch.setattr(reader_mod, "HAS_ROS", False)
    via_sqlite = _comparable(str(bag_dir))

    assert via_ros == via_sqlite
    assert via_ros["total_messages"] > 0


FRAME_LENGTHS = range(0, 12)  # every residue of the string length modulo 4 and 8


def _cdr_cases():
    """(msg_type, message factory, expected fields) for every CDR fallback parser."""
    from geometry_msgs.msg import (
        PoseStamped, PoseWithCovarianceStamped, TransformStamped, TwistStamped,
    )
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import (
        CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, NavSatFix, PointCloud2,
    )
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage

    def with_header(msg, frame):
        header = msg if isinstance(msg, Header) else msg.header
        header.stamp.sec, header.stamp.nanosec, header.frame_id = 7, 8, frame
        return msg

    def navsat(f):
        m = with_header(NavSatFix(latitude=35.5, longitude=139.25, altitude=10.0), f)
        m.status.status, m.status.service = -1, 4
        return m

    def imu(f):
        m = with_header(Imu(), f)
        m.linear_acceleration.z, m.angular_velocity.x, m.orientation.w = 9.5, 0.25, 1.0
        return m

    def pose(f):
        m = with_header(PoseStamped(), f)
        m.pose.position.x, m.pose.orientation.w = 1.5, 1.0
        return m

    def pose_cov(f):
        m = with_header(PoseWithCovarianceStamped(), f)
        m.pose.pose.position.y = 2.5
        return m

    def twist(f):
        m = with_header(TwistStamped(), f)
        m.twist.linear.x = 0.75
        return m

    def odom(f):
        m = with_header(Odometry(), f)
        m.child_frame_id = "c" * (len(f) % 5)
        m.pose.pose.position.x, m.twist.twist.linear.x = 3.5, 0.5
        return m

    def tf(f):
        t = TransformStamped()
        with_header(t, f)
        t.child_frame_id = "child" + "x" * (len(f) % 3)
        t.transform.translation.x, t.transform.rotation.w = 1.25, 1.0
        return TFMessage(transforms=[t, t])

    def cloud(f):
        return with_header(PointCloud2(height=1, width=17, point_step=16, row_step=272,
                                       data=bytes(272)), f)

    def scan(f):
        return with_header(LaserScan(angle_min=-1.0, angle_max=1.0, angle_increment=0.5,
                                     range_min=0.5, range_max=5.0,
                                     ranges=[1.0, 9.0, float("inf"), 2.0]), f)

    def info(f):
        return with_header(CameraInfo(width=640, height=480, distortion_model="m" * (len(f) % 7),
                                      d=[0.1] * (len(f) % 6), k=[float(i) for i in range(9)]), f)

    def image(f):
        return with_header(Image(width=3, height=2, encoding="e" * (1 + len(f) % 5),
                                 is_bigendian=1, step=9, data=bytes(18)), f)

    def compressed(f):
        return with_header(CompressedImage(format="j" * (1 + len(f) % 4), data=bytes(33)), f)

    def joints(f):
        return with_header(JointState(name=["j" * (k + 1) for k in range(len(f) % 4)],
                                      position=[0.5] * (len(f) % 4)), f)

    return [
        ("sensor_msgs/msg/NavSatFix", navsat,
         lambda p, f: (p["status"], p["service"], p["latitude"], p["altitude"])
         == (-1, 4, 35.5, 10.0)),
        ("sensor_msgs/msg/Imu", imu,
         lambda p, f: p["linear_acceleration"]["z"] == 9.5 and p["angular_velocity"]["x"] == 0.25),
        ("geometry_msgs/msg/PoseStamped", pose, lambda p, f: p["position"]["x"] == 1.5),
        ("geometry_msgs/msg/PoseWithCovarianceStamped", pose_cov,
         lambda p, f: p["pose"]["pose"]["position"]["y"] == 2.5),
        ("geometry_msgs/msg/TwistStamped", twist, lambda p, f: p["twist"]["linear"]["x"] == 0.75),
        ("nav_msgs/msg/Odometry", odom,
         lambda p, f: p["child_frame_id"] == "c" * (len(f) % 5)
         and p["pose"]["pose"]["position"]["x"] == 3.5),
        ("tf2_msgs/msg/TFMessage", tf,
         lambda p, f: len(p["transforms"]) == 2
         and p["transforms"][1]["child_frame_id"] == "child" + "x" * (len(f) % 3)
         and p["transforms"][1]["transform"]["translation"]["x"] == 1.25),
        ("sensor_msgs/msg/PointCloud2", cloud, lambda p, f: p["num_points"] == 17),
        ("sensor_msgs/msg/LaserScan", scan,
         lambda p, f: (p["num_ranges"], p["num_valid_ranges"]) == (4, 2)),
        ("sensor_msgs/msg/CameraInfo", info,
         lambda p, f: p["width"] == 640 and p["d"] == [0.1] * (len(f) % 6) and p["k"][8] == 8.0),
        ("sensor_msgs/msg/Image", image,
         lambda p, f: (p["encoding"], p["is_bigendian"], p["step"], p["data_size"])
         == ("e" * (1 + len(f) % 5), 1, 9, 18)),
        ("sensor_msgs/msg/CompressedImage", compressed,
         lambda p, f: (p["format"], p["data_size"]) == ("j" * (1 + len(f) % 4), 33)),
        ("sensor_msgs/msg/JointState", joints,
         lambda p, f: p["name"] == ["j" * (k + 1) for k in range(len(f) % 4)]),
    ]


def test_cdr_parsers_match_rclpy_serialization_for_any_frame_length() -> None:
    """Guards field alignment: a misaligned field only shows for some string lengths."""
    from rclpy.serialization import serialize_message

    failures = []
    for msg_type, make, check in _cdr_cases():
        for n in FRAME_LENGTHS:
            frame = "f" * n
            parsed = reader_mod._parse_cdr_basic(bytes(serialize_message(make(frame))), msg_type)
            ok = not parsed.get("_parse_error")
            if ok and msg_type != "tf2_msgs/msg/TFMessage":
                ok = parsed.get("frame_id") == frame and parsed.get("stamp_sec") == 7
            try:
                ok = ok and check(parsed, frame)
            except (KeyError, IndexError, TypeError):
                ok = False
            if not ok:
                failures.append((msg_type, n))
    assert failures == []
