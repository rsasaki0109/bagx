"""Tests for bagx.eval module."""

import math
from pathlib import Path

import pytest

from bagx.eval import EvalConfig, evaluate_bag


class TestEvalGnss:
    def test_gnss_fix_rate(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        assert report.gnss is not None
        assert report.gnss.fix_rate == pytest.approx(0.9, abs=0.01)
        assert report.gnss.fix_count == 90
        assert report.gnss.no_fix_count == 10

    def test_gnss_hdop(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        g = report.gnss
        assert not math.isnan(g.hdop_mean)
        assert g.hdop_mean > 0
        assert g.hdop_max >= g.hdop_mean

    def test_gnss_coordinates(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        g = report.gnss
        lat_min, lat_max = g.latitude_range
        assert abs(lat_min - 35.6812) < 0.01
        assert abs(lat_max - 35.6812) < 0.01

    def test_gnss_score_range(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        assert 0 <= report.gnss.score <= 100


class TestEvalImu:
    def test_imu_metrics(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        assert report.imu is not None
        assert report.imu.total_messages == 1000

    def test_imu_noise(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        m = report.imu
        # Noise should be small (generated with std=0.05 for accel)
        assert not math.isnan(m.accel_noise_x)
        assert m.accel_noise_x < 1.0
        assert not math.isnan(m.gyro_noise_x)
        assert m.gyro_noise_x < 0.1

    def test_imu_frequency(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        # Generated at 200Hz (5ms interval)
        assert report.imu.frequency_hz == pytest.approx(200.0, rel=0.05)

    def test_imu_bias_stability(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        assert not math.isnan(report.imu.accel_bias_stability)
        assert not math.isnan(report.imu.gyro_bias_stability)

    def test_imu_score_range(self, imu_bag: Path):
        report = evaluate_bag(str(imu_bag))
        assert 0 <= report.imu.score <= 100


class TestEvalMulti:
    def test_overall_score(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.overall_score > 0
        assert report.overall_score <= 100

    def test_sync_detected(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.sync is not None
        assert len(report.sync.topic_pairs) > 0

    def test_topic_count(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.topic_count == 3
        assert report.total_messages == 600

    def test_duration(self, multi_bag: Path):
        report = evaluate_bag(str(multi_bag))
        assert report.duration_sec > 0


class TestEvalReport:
    def test_to_dict_no_nan(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        d = report.to_dict()
        # NaN values should be replaced with None
        _assert_no_nan(d)

    def test_json_output(self, gnss_bag: Path, tmp_path: Path):
        import json

        json_path = tmp_path / "report.json"
        with open(json_path, "w") as f:
            evaluate_bag(str(gnss_bag), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "gnss" in data
        assert data["gnss"]["fix_rate"] == pytest.approx(0.9, abs=0.01)


class TestFrameworkDetection:
    """Test domain-specific recommendations for supported frameworks."""

    def test_nav2_sync_excludes_control_topics(self, nav2_bag: Path):
        report = evaluate_bag(str(nav2_bag))
        if report.sync is None:
            return

        sync_pairs = {topic for pair in report.sync.topic_pairs for topic in pair}
        assert "/controller_server/cmd_vel" not in sync_pairs
        assert "/local_costmap/costmap" not in sync_pairs

    def test_nav2_namespaced_topics_detected(self, nav2_bag: Path):
        report = evaluate_bag(str(nav2_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Nav2 topics detected" in recommendations
        assert "Odometry (/robot/odom) at 50Hz" in recommendations
        assert "LaserScan (/robot/scan) at 12Hz" in recommendations
        assert "Global plan (/plan) recorded 4 times" in recommendations
        assert "NavigateToPose status (/navigate_to_pose/_action/status) recorded" in recommendations
        assert "scan → costmap" in recommendations
        assert "plan → cmd_vel onset" in recommendations

    def test_autoware_topics_detected(self, autoware_bag: Path):
        report = evaluate_bag(str(autoware_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Autoware topics detected" in recommendations
        assert "LiDAR (/sensing/lidar/top/pointcloud_raw_ex) at 10Hz" in recommendations
        assert "Camera (/sensing/camera/front/image_raw/compressed) at 30Hz" in recommendations
        assert "GNSS (/sensing/gnss/ublox/nav_sat_fix)" in recommendations
        assert "Control command (/control/command/control_cmd) at 10Hz" in recommendations
        assert "sensing → perception" in recommendations
        assert "end-to-end (sensing → control): 110ms median" in recommendations

    def test_autoware_sensing_only_bag_skips_planning_checks(self, autoware_sensing_only_bag: Path):
        report = evaluate_bag(str(autoware_sensing_only_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Autoware topics detected" in recommendations
        assert "Sensing/localization-only Autoware bag" in recommendations

    def test_autoware_vehicle_status_and_packet_lidar_detected(self, autoware_vehicle_status_bag: Path):
        report = evaluate_bag(str(autoware_vehicle_status_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Autoware topics detected" in recommendations
        assert "LiDAR (/sensing/lidar/front/velodyne_packets) at 10Hz" in recommendations
        assert "Vehicle status (/vehicle/status/velocity_status) at 100Hz" in recommendations
        assert "Sensing/localization-only Autoware bag" in recommendations

    def test_moveit_topics_detected(self, moveit_bag: Path):
        report = evaluate_bag(str(moveit_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "MoveIt topics detected" in recommendations
        assert "JointState (/fr3/joint_states) at 200Hz" in recommendations
        assert "MoveGroup action activity recorded on /move_action/_action/status" in recommendations
        assert "Joint trajectory controller activity recorded on /panda_arm_controller/follow_joint_trajectory/_action/status" in recommendations
        assert "joint_states → planned_path" in recommendations
        assert "planned_path → arm execution" in recommendations

    def test_moveit_sparse_planned_path_still_reports_pipeline(self, moveit_sparse_bag: Path):
        report = evaluate_bag(str(moveit_sparse_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "MoveIt topics detected" in recommendations
        assert "joint_states → planned_path" in recommendations
        assert "1 sample" in recommendations

    def test_robot_arm_perception_bag_uses_domain_specific_recommendations(self, robotarm_bag: Path):
        report = evaluate_bag(str(robotarm_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Robot arm perception/manipulation topics detected" in recommendations
        assert "JointState (/joint_states) at 200Hz" in recommendations
        assert "RGB image (/camera_1/color/image_raw) at 30Hz" in recommendations
        assert "Depth image (/camera_1/aligned_depth_to_color/image_raw) at 15Hz" in recommendations
        assert "Camera calibration topics are recorded" in recommendations
        assert "No GNSS data" not in recommendations
        assert "No IMU data" not in recommendations

    def test_robot_arm_sync_excludes_camera_info_cross_pairs(self, robotarm_bag: Path):
        report = evaluate_bag(str(robotarm_bag))
        assert report.sync is not None

        sync_pairs = {topic for pair in report.sync.topic_pairs for topic in pair}
        assert "/camera_1/color/camera_info" not in sync_pairs
        assert "/camera_1/aligned_depth_to_color/camera_info" not in sync_pairs

    def test_perception_bag_uses_domain_specific_recommendations(self, perception_bag: Path):
        report = evaluate_bag(str(perception_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Perception topics detected" in recommendations
        assert "RGB image (/camera/color/image_raw)" in recommendations
        assert "Depth image (/camera/realsense_splitter_node/output/depth)" in recommendations
        assert "Infra image (/camera/realsense_splitter_node/output/infra1)" in recommendations
        assert "Infra stereo streams are both recorded" in recommendations
        assert "Camera calibration topics are recorded" in recommendations
        assert "No GNSS data" not in recommendations
        assert "No IMU data" not in recommendations

    def test_perception_bag_has_domain_score(self, perception_bag: Path):
        report = evaluate_bag(str(perception_bag))

        assert report.domain_score is not None
        assert report.domain_score >= 90

    def test_generic_control_bag_uses_generic_control_recommendations(self, generic_control_bag: Path):
        report = evaluate_bag(str(generic_control_bag))
        recommendations = "\n".join(report.to_dict()["recommendations"])

        assert "Planning/control topics detected" in recommendations
        assert "State feedback (/base/state/odom) at 25Hz" in recommendations
        assert "Control command (/drive/cmd_vel) at 20Hz" in recommendations
        assert "Planner output (/planner/path) recorded 8 times" in recommendations
        assert "Action status (/mission/_action/status) recorded" in recommendations
        assert "planner → command onset" in recommendations
        assert "command → state feedback" in recommendations
        assert "Nav2 topics detected" not in recommendations
        assert "No GNSS data" not in recommendations
        assert "No IMU data" not in recommendations

    def test_generic_control_bag_has_domain_score(self, generic_control_bag: Path):
        report = evaluate_bag(str(generic_control_bag))

        assert report.domain_score is not None
        assert report.domain_score >= 90


class TestEvalConfig:
    """Test EvalConfig customization."""

    def test_default_config_same_as_before(self, gnss_bag: Path):
        """Default EvalConfig should produce the same results as no config."""
        report_default = evaluate_bag(str(gnss_bag))
        report_explicit = evaluate_bag(str(gnss_bag), config=EvalConfig())
        assert report_default.gnss.score == pytest.approx(report_explicit.gnss.score)
        assert report_default.overall_score == pytest.approx(report_explicit.overall_score)

    def test_custom_config_gnss_fix_only(self, gnss_bag: Path):
        """With gnss_hdop_weight=0, score should depend only on fix rate."""
        config = EvalConfig(gnss_fix_weight=1.0, gnss_hdop_weight=0.0)
        report = evaluate_bag(str(gnss_bag), config=config)
        # fix_rate is 0.9, so fix_score = min(0.9 * 100, 100) = 90
        # With fix_weight=1.0 and hdop_weight=0.0, score = 90 * 1.0 + 0 = 90
        assert report.gnss.score == pytest.approx(90.0, abs=1.0)

    def test_custom_config_gnss_hdop_only(self, gnss_bag: Path):
        """With gnss_fix_weight=0, score should depend only on HDOP."""
        config = EvalConfig(gnss_fix_weight=0.0, gnss_hdop_weight=1.0)
        report = evaluate_bag(str(gnss_bag), config=config)
        # Score should be purely HDOP-based, different from fix-only
        assert 0 <= report.gnss.score <= 100
        # Verify it's different from fix-only score
        config_fix = EvalConfig(gnss_fix_weight=1.0, gnss_hdop_weight=0.0)
        report_fix = evaluate_bag(str(gnss_bag), config=config_fix)
        assert report.gnss.score != pytest.approx(report_fix.gnss.score, abs=0.1)

    def test_evaluate_bag_accepts_config_parameter(self, gnss_bag: Path):
        """Verify evaluate_bag actually accepts and uses the config parameter."""
        config = EvalConfig(gnss_hdop_scale=0.0)  # HDOP has no effect
        report = evaluate_bag(str(gnss_bag), config=config)
        assert report.gnss is not None
        # With hdop_scale=0, hdop_score = max(0, 100 - 0) = 100
        # So overall gnss score should be higher than default
        default_report = evaluate_bag(str(gnss_bag))
        assert report.gnss.score >= default_report.gnss.score


def _assert_no_nan(obj):
    if isinstance(obj, dict):
        for v in obj.values():
            _assert_no_nan(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            _assert_no_nan(v)
    elif isinstance(obj, float):
        assert not math.isnan(obj), "Found NaN in report dict"
