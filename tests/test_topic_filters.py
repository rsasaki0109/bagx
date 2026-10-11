"""Tests for shared topic filter helpers."""

import numpy as np

from bagx.topic_filters import (
    is_rate_anomaly_candidate,
    is_sync_candidate,
    nearest_neighbor_delays_ns,
)


class TestSyncCandidate:
    def test_sensor_topics_are_included(self):
        assert is_sync_candidate("/robot/scan", "sensor_msgs/msg/LaserScan")
        assert is_sync_candidate("/front_camera/image_raw", "sensor_msgs/msg/Image")

    def test_control_and_meta_topics_are_excluded(self):
        assert not is_sync_candidate("/clock", "rosgraph_msgs/msg/Clock")
        assert not is_sync_candidate("/controller_server/cmd_vel", "geometry_msgs/msg/TwistStamped")
        assert not is_sync_candidate("/camera/color/camera_info", "sensor_msgs/msg/CameraInfo")
        assert not is_sync_candidate(
            "/navigate_to_pose/_action/status",
            "action_msgs/msg/GoalStatusArray",
        )

    def test_auxiliary_readings_under_sensor_namespaces_are_excluded(self):
        # NTU VIRAL publishes these next to /imu/imu
        assert not is_sync_candidate("/imu/temperature", "sensor_msgs/msg/Temperature")
        assert not is_sync_candidate("/imu/magnetic_field", "sensor_msgs/msg/MagneticField")
        assert is_sync_candidate("/imu/imu", "sensor_msgs/msg/Imu")


class TestNearestNeighborDelays:
    def test_walks_the_sparser_stream(self):
        imu = np.arange(0, 1_000_000_000, 5_000_000, dtype=np.int64)  # 200 Hz
        lidar = np.arange(1_000_000, 1_000_000_000, 100_000_000, dtype=np.int64)  # 10 Hz, +1ms
        delays = nearest_neighbor_delays_ns(imu, lidar)
        assert len(delays) == len(lidar)
        assert np.all(delays == 1_000_000)

    def test_symmetric_in_argument_order(self):
        a = np.array([0, 40, 100, 140], dtype=np.int64)
        b = np.array([10, 30, 50, 70, 90, 110, 130, 150], dtype=np.int64)
        assert list(nearest_neighbor_delays_ns(a, b)) == list(nearest_neighbor_delays_ns(b, a))
        assert list(nearest_neighbor_delays_ns(a, b)) == [10, 10, 10, 10]

    def test_empty_input(self):
        empty = np.array([], dtype=np.int64)
        assert len(nearest_neighbor_delays_ns(empty, np.array([1, 2], dtype=np.int64))) == 0


class TestRateAnomalyCandidate:
    def test_regular_sensor_topics_are_included(self):
        assert is_rate_anomaly_candidate("/imu", "sensor_msgs/msg/Imu")
        assert is_rate_anomaly_candidate("/odom", "nav_msgs/msg/Odometry")

    def test_internal_planning_and_action_topics_are_excluded(self):
        assert not is_rate_anomaly_candidate(
            "/move_group/monitored_planning_scene",
            "moveit_msgs/msg/PlanningScene",
        )
        assert not is_rate_anomaly_candidate(
            "/panda_arm_controller/follow_joint_trajectory/_action/status",
            "action_msgs/msg/GoalStatusArray",
        )
        assert not is_rate_anomaly_candidate(
            "/planner/compute_path/_service_event",
            "nav_msgs/srv/GetPlan_Event",
        )
        assert not is_rate_anomaly_candidate(
            "/mission/result",
            "example_interfaces/action/Fibonacci_Result",
        )
        assert not is_rate_anomaly_candidate("/tf_static", "tf2_msgs/msg/TFMessage")
