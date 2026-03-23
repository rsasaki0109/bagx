"""Generate tiny benchmark bags and run the benchmark CLI as a smoke test."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _create_nav2_bag(path: Path) -> Path:
    from tests.conftest import (
        _create_db3,
        build_odometry_cdr,
        build_pose_with_covariance_stamped_cdr,
        build_stub_cdr,
        build_twist_stamped_cdr,
    )

    topics = [
        {"name": "/robot/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"},
        {"name": "/robot/scan", "type": "sensor_msgs/msg/LaserScan", "format": "cdr"},
        {"name": "/robot/amcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "format": "cdr"},
        {"name": "/local_costmap/costmap", "type": "nav_msgs/msg/OccupancyGrid", "format": "cdr"},
        {"name": "/plan", "type": "nav_msgs/msg/Path", "format": "cdr"},
        {"name": "/navigate_to_pose/_action/status", "type": "action_msgs/msg/GoalStatusArray", "format": "cdr"},
        {"name": "/controller_server/cmd_vel", "type": "geometry_msgs/msg/TwistStamped", "format": "cdr"},
    ]
    messages: list[dict] = []
    base_ns = 1_700_001_000_000_000_000

    for i in range(60):
        ts = base_ns + i * 20_000_000  # 50Hz
        messages.append({
            "topic": "/robot/odom",
            "timestamp_ns": ts,
            "data": build_odometry_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                position=(i * 0.05, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
                linear=(0.5, 0.0, 0.0),
                angular=(0.0, 0.0, 0.0),
            ),
        })
        messages.append({
            "topic": "/robot/amcl_pose",
            "timestamp_ns": ts + 5_000_000,
            "data": build_pose_with_covariance_stamped_cdr(
                stamp_sec=(ts + 5_000_000) // 1_000_000_000,
                stamp_nanosec=(ts + 5_000_000) % 1_000_000_000,
                position=(i * 0.05, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
        })

    for i in range(15):
        ts = base_ns + i * 100_000_000  # 10Hz
        messages.append({"topic": "/robot/scan", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/local_costmap/costmap", "timestamp_ns": ts + 30_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/plan", "timestamp_ns": ts + 45_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/navigate_to_pose/_action/status", "timestamp_ns": ts + 50_000_000, "data": build_stub_cdr()})
        messages.append({
            "topic": "/controller_server/cmd_vel",
            "timestamp_ns": ts + 60_000_000,
            "data": build_twist_stamped_cdr(
                stamp_sec=(ts + 60_000_000) // 1_000_000_000,
                stamp_nanosec=(ts + 60_000_000) % 1_000_000_000,
                linear=(0.1, 0.0, 0.0),
                angular=(0.0, 0.0, 0.0),
            ),
        })

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(path, topics, messages)


def _create_robotarm_bag(path: Path) -> Path:
    from tests.conftest import _create_db3, build_stub_cdr

    topics = [
        {"name": "/camera_1/color/image_raw", "type": "sensor_msgs/msg/Image", "format": "cdr"},
        {"name": "/camera_1/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
        {"name": "/camera_1/aligned_depth_to_color/image_raw", "type": "sensor_msgs/msg/Image", "format": "cdr"},
        {"name": "/camera_1/aligned_depth_to_color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
        {"name": "/joint_states", "type": "sensor_msgs/msg/JointState", "format": "cdr"},
    ]
    messages: list[dict] = []
    base_ns = 1_700_001_100_000_000_000

    for i in range(250):
        ts = base_ns + i * 5_000_000  # 200Hz
        messages.append({"topic": "/joint_states", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(30):
        ts = base_ns + i * 33_333_333  # ~30Hz
        messages.append({"topic": "/camera_1/color/image_raw", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera_1/color/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(16):
        ts = base_ns + i * 66_666_666 + 10_000_000  # ~15Hz
        messages.append({"topic": "/camera_1/aligned_depth_to_color/image_raw", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera_1/aligned_depth_to_color/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(path, topics, messages)


def _create_perception_bag(path: Path) -> Path:
    from tests.conftest import _create_db3, build_stub_cdr

    topics = [
        {"name": "/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
        {"name": "/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "format": "cdr"},
        {"name": "/camera/depth/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
        {"name": "/camera/infra1/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
        {"name": "/camera/infra2/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
        {"name": "/camera/realsense_splitter_node/output/depth", "type": "sensor_msgs/msg/Image", "format": "cdr"},
        {"name": "/camera/realsense_splitter_node/output/infra1", "type": "sensor_msgs/msg/Image", "format": "cdr"},
        {"name": "/camera/realsense_splitter_node/output/infra2", "type": "sensor_msgs/msg/Image", "format": "cdr"},
        {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "format": "cdr"},
    ]
    messages: list[dict] = []
    base_ns = 1_700_001_200_000_000_000

    for i in range(18):
        ts = base_ns + i * 66_666_666  # ~15Hz
        messages.append({"topic": "/camera/color/image_raw", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/color/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(36):
        ts = base_ns + i * 33_333_333 + 8_000_000  # ~30Hz
        messages.append({"topic": "/camera/realsense_splitter_node/output/depth", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/depth/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/realsense_splitter_node/output/infra1", "timestamp_ns": ts + 4_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/infra1/camera_info", "timestamp_ns": ts + 4_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/realsense_splitter_node/output/infra2", "timestamp_ns": ts + 5_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/infra2/camera_info", "timestamp_ns": ts + 5_000_000, "data": build_stub_cdr()})

    messages.append({"topic": "/tf_static", "timestamp_ns": base_ns, "data": build_stub_cdr()})

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(path, topics, messages)


def main() -> None:
    from bagx.contracts import REPORT_SCHEMA_VERSION

    with tempfile.TemporaryDirectory(prefix="bagx-benchmark-smoke-") as tmp_dir:
        tmp_path = Path(tmp_dir)
        nav2_bag = _create_nav2_bag(tmp_path / "nav2.db3")
        robotarm_bag = _create_robotarm_bag(tmp_path / "robotarm.db3")
        perception_bag = _create_perception_bag(tmp_path / "perception.db3")

        manifest_path = tmp_path / "smoke_suite.json"
        report_path = tmp_path / "benchmark-report.json"
        manifest = {
            "suite_name": "ci-smoke-suite",
            "cases": [
                {
                    "name": "nav2-smoke",
                    "bag_path": str(nav2_bag),
                    "expect": {
                        "required_domains": ["Nav2"],
                        "required_recommendations": ["Nav2 topics detected"],
                        "min_overall_score": 90,
                    },
                },
                {
                    "name": "robotarm-smoke",
                    "bag_path": str(robotarm_bag),
                    "expect": {
                        "required_domains": ["RobotArm"],
                        "required_recommendations": ["Robot arm perception/manipulation topics detected"],
                        "forbidden_recommendations": ["No GNSS data"],
                        "min_domain_score": 90,
                    },
                },
                {
                    "name": "perception-smoke",
                    "bag_path": str(perception_bag),
                    "expect": {
                        "required_domains": ["Perception"],
                        "required_recommendations": ["Perception topics detected"],
                        "forbidden_recommendations": ["No GNSS data", "No IMU data"],
                        "min_domain_score": 90,
                    },
                },
            ],
        }
        manifest_path.write_text(json.dumps(manifest, indent=2))

        subprocess.run(
            [
                sys.executable,
                "-m",
                "bagx.cli",
                "benchmark",
                str(manifest_path),
                "--json",
                str(report_path),
            ],
            check=True,
        )

        with open(report_path) as f:
            data = json.load(f)

        if data["schema_version"] != REPORT_SCHEMA_VERSION:
            raise SystemExit(f"unexpected schema_version: {data['schema_version']}")
        if data["report_type"] != "benchmark_suite":
            raise SystemExit(f"unexpected report_type: {data['report_type']}")
        if data["failed_cases"] != 0 or data["passed_cases"] != 3:
            raise SystemExit(f"unexpected benchmark result: {data}")


if __name__ == "__main__":
    main()
