"""Tests for shared topic filter helpers."""

from bagx.topic_filters import is_rate_anomaly_candidate, is_sync_candidate


class TestSyncCandidate:
    def test_sensor_topics_are_included(self):
        assert is_sync_candidate("/robot/scan", "sensor_msgs/msg/LaserScan")
        assert is_sync_candidate("/front_camera/image_raw", "sensor_msgs/msg/Image")

    def test_control_and_meta_topics_are_excluded(self):
        assert not is_sync_candidate("/clock", "rosgraph_msgs/msg/Clock")
        assert not is_sync_candidate("/controller_server/cmd_vel", "geometry_msgs/msg/TwistStamped")
        assert not is_sync_candidate(
            "/navigate_to_pose/_action/status",
            "action_msgs/msg/GoalStatusArray",
        )


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
        assert not is_rate_anomaly_candidate("/tf_static", "tf2_msgs/msg/TFMessage")
