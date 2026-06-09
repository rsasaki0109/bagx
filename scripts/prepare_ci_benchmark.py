"""Generate tiny benchmark bags and a manifest for bagx-action CI dogfooding."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

DEFAULT_OUT_DIR = REPO_ROOT / ".ci-benchmark"


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
        ts = base_ns + i * 20_000_000
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
        ts = base_ns + i * 100_000_000
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
        ts = base_ns + i * 5_000_000
        messages.append({"topic": "/joint_states", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(30):
        ts = base_ns + i * 33_333_333
        messages.append({"topic": "/camera_1/color/image_raw", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera_1/color/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(16):
        ts = base_ns + i * 66_666_666 + 10_000_000
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
        ts = base_ns + i * 66_666_666
        messages.append({"topic": "/camera/color/image_raw", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/color/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})

    for i in range(36):
        ts = base_ns + i * 33_333_333 + 8_000_000
        messages.append({"topic": "/camera/realsense_splitter_node/output/depth", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/depth/camera_info", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/realsense_splitter_node/output/infra1", "timestamp_ns": ts + 4_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/infra1/camera_info", "timestamp_ns": ts + 4_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/realsense_splitter_node/output/infra2", "timestamp_ns": ts + 5_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/camera/infra2/camera_info", "timestamp_ns": ts + 5_000_000, "data": build_stub_cdr()})

    messages.append({"topic": "/tf_static", "timestamp_ns": base_ns, "data": build_stub_cdr()})

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(path, topics, messages)


def _create_control_bag(path: Path) -> Path:
    from tests.conftest import _create_db3, build_odometry_cdr, build_stub_cdr, build_twist_stamped_cdr

    topics = [
        {"name": "/base/state/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"},
        {"name": "/drive/cmd_vel", "type": "geometry_msgs/msg/TwistStamped", "format": "cdr"},
        {"name": "/planner/path", "type": "nav_msgs/msg/Path", "format": "cdr"},
        {"name": "/mission/_action/status", "type": "action_msgs/msg/GoalStatusArray", "format": "cdr"},
        {"name": "/mission/result", "type": "example_interfaces/action/Fibonacci_Result", "format": "cdr"},
        {"name": "/planner/compute_path/_service_event", "type": "nav_msgs/srv/GetPlan_Event", "format": "cdr"},
    ]
    messages: list[dict] = []
    base_ns = 1_700_001_300_000_000_000

    for i in range(80):
        ts = base_ns + i * 40_000_000
        messages.append({
            "topic": "/base/state/odom",
            "timestamp_ns": ts,
            "data": build_odometry_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                position=(i * 0.03, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
                linear=(0.4, 0.0, 0.0),
                angular=(0.0, 0.0, 0.0),
            ),
        })

    for i in range(64):
        ts = base_ns + i * 50_000_000 + 15_000_000
        messages.append({
            "topic": "/drive/cmd_vel",
            "timestamp_ns": ts,
            "data": build_twist_stamped_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                linear=(0.1, 0.0, 0.0),
                angular=(0.0, 0.0, 0.0),
            ),
        })

    for i in range(6):
        ts = base_ns + i * 1_000_000_000
        messages.append({"topic": "/planner/compute_path/_service_event", "timestamp_ns": ts - 5_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/planner/path", "timestamp_ns": ts, "data": build_stub_cdr()})
        messages.append({"topic": "/mission/_action/status", "timestamp_ns": ts + 10_000_000, "data": build_stub_cdr()})
        messages.append({"topic": "/mission/result", "timestamp_ns": ts + 180_000_000, "data": build_stub_cdr()})

    messages.sort(key=lambda m: m["timestamp_ns"])
    return _create_db3(path, topics, messages)


def prepare_ci_benchmark(out_dir: Path = DEFAULT_OUT_DIR) -> Path:
    """Create synthetic bags and smoke_suite.json under out_dir."""
    out_dir.mkdir(parents=True, exist_ok=True)
    nav2_bag = _create_nav2_bag(out_dir / "nav2.db3")
    robotarm_bag = _create_robotarm_bag(out_dir / "robotarm.db3")
    perception_bag = _create_perception_bag(out_dir / "perception.db3")
    control_bag = _create_control_bag(out_dir / "control.db3")

    manifest_path = out_dir / "smoke_suite.json"
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
            {
                "name": "control-smoke",
                "bag_path": str(control_bag),
                "expect": {
                    "required_domains": ["Control"],
                    "required_recommendations": [
                        "Planning/control topics detected",
                        "Action result (/mission/result) recorded",
                        "Service event (/planner/compute_path/_service_event) recorded",
                    ],
                    "forbidden_recommendations": ["Nav2 topics detected"],
                    "min_domain_score": 90,
                },
            },
        ],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2))
    return manifest_path


def main() -> None:
    manifest = prepare_ci_benchmark()
    print(f"Prepared CI benchmark manifest: {manifest}")


if __name__ == "__main__":
    main()
