"""Framework readiness must expose low rates without changing legacy scores."""

import json

import jsonschema
import pytest
from typer.testing import CliRunner

from bagx.cli import app
from bagx.contracts import findings_schema
from bagx.domain_findings import generate_domain_readiness_findings
from bagx.eval import EvalReport, evaluate_bag
from bagx.eval_findings import generate_findings


def report(topics):
    return EvalReport(
        "fixture",
        10,
        100,
        len(topics),
        topic_info={
            name: {"type": type_name, "count": 10, "rate_hz": rate}
            for name, type_name, rate in topics
        },
    )


@pytest.mark.parametrize(
    "domain,topic,type_name,goal",
    [
        ("Autoware", "/sensing/camera/image", "sensor_msgs/msg/Image", 15),
        ("Autoware", "/sensing/lidar/points", "sensor_msgs/msg/PointCloud2", 10),
        ("Autoware", "/vehicle/status/velocity_status", "custom/msg/VelocityReport", 20),
        ("Autoware", "/planning/trajectory", "custom/msg/Trajectory", 5),
        ("Autoware", "/control/command/control_cmd", "custom/msg/Control", 10),
        ("MoveIt", "/joint_states", "sensor_msgs/msg/JointState", 100),
        ("RobotArm", "/joint_states", "sensor_msgs/msg/JointState", 100),
        ("RobotArm", "/camera/color/image", "sensor_msgs/msg/Image", 15),
        ("Perception", "/camera/depth/image", "sensor_msgs/msg/Image", 15),
        ("Control", "/odom", "nav_msgs/msg/Odometry", 20),
        ("Control", "/controller/cmd_vel", "geometry_msgs/msg/Twist", 10),
    ],
)
def test_rate_boundary_and_evidence(domain, topic, type_name, goal):
    for rate, state, severity in [(goal - 0.1, "rate_low", "warning"), (goal, "rate_good", "info")]:
        findings = generate_domain_readiness_findings(
            report([(topic, type_name, rate)]), domain=domain
        )
        rate_finding = next(f for f in findings if f.category == "rate_quality")
        assert rate_finding.id.endswith("." + state)
        assert rate_finding.severity == severity
        assert rate_finding.affected_topics == [topic]
        assert rate_finding.evidence[0].observed == rate
        assert rate_finding.evidence[0].expected == f">={goal}"
        jsonschema.validate(rate_finding.to_dict(), findings_schema())


def test_secondary_camera_cannot_be_hidden_by_fast_primary():
    data = report(
        [
            ("/camera/fast", "sensor_msgs/msg/CompressedImage", 30),
            ("/camera/slow", "sensor_msgs/msg/Image", 5),
            ("/camera/camera_info", "sensor_msgs/msg/CameraInfo", 30),
        ]
    )
    findings = generate_findings(data, set())
    rates = [f for f in findings if f.category == "rate_quality"]
    assert {tuple(f.affected_topics): f.severity for f in rates} == {
        ("/camera/fast",): "info",
        ("/camera/slow",): "warning",
    }


@pytest.mark.parametrize("rate", [float("nan"), float("inf"), -1, 100001])
def test_implausible_rates_are_unavailable(rate):
    findings = generate_domain_readiness_findings(
        report([("/joint_states", "sensor_msgs/msg/JointState", rate)]), domain="MoveIt"
    )
    finding = next(f for f in findings if f.category == "rate_quality")
    assert finding.id.endswith(".rate_unavailable")
    assert finding.severity == "warning"
    json.dumps(finding.to_dict(), allow_nan=False)


def test_autoware_sensing_only_does_not_require_planning_or_control():
    data = report([("/sensing/lidar/points", "sensor_msgs/msg/PointCloud2", 10)])
    findings = generate_findings(data, set())
    assert not any("missing_planning" in f.id or "missing_control" in f.id for f in findings)


@pytest.mark.parametrize("stage,missing", [("planning", "control"), ("control", "planning")])
def test_autoware_recorded_stage_requires_its_counterpart(stage, missing):
    data = report([(f"/{stage}/trajectory", "custom/msg/Trajectory", 10)])
    findings = generate_findings(data, set())
    assert any(f.id == f"autoware.missing_{missing}" and f.severity == "warning" for f in findings)


def test_moveit_feedback_can_make_execution_observable():
    data = report(
        [
            ("/joint_states", "sensor_msgs/msg/JointState", 100),
            ("/display_planned_path", "moveit_msgs/msg/DisplayTrajectory", 0.1),
        ]
    )
    assert "moveit.missing_execution" in {f.id for f in generate_findings(data, set())}
    data.topic_info["/execute_trajectory/_action/feedback"] = {
        "type": "custom/msg/Feedback",
        "count": 10,
        "rate_hz": 1,
    }
    assert "moveit.missing_execution" not in {f.id for f in generate_findings(data, set())}


def test_moveit_planning_scene_without_path_is_reported():
    data = report(
        [
            ("/joint_states", "sensor_msgs/msg/JointState", 100),
            ("/planning_scene", "moveit_msgs/msg/PlanningScene", 1),
        ]
    )
    assert "moveit.missing_planned_path" in {f.id for f in generate_findings(data, set())}


def test_robotarm_camera_info_absence_is_explicit():
    data = report(
        [
            ("/joint_states", "sensor_msgs/msg/JointState", 100),
            ("/camera/color", "sensor_msgs/msg/Image", 15),
        ]
    )
    findings = generate_findings(data, set())
    assert any(f.id == "robotarm.missing_camera_info" for f in findings)
    assert not any(f.id.startswith("perception.") for f in findings)


@pytest.mark.parametrize(
    "fixture,overall,domain",
    [
        ("nav2_bag", 100, 100),
        ("moveit_bag", 100, 100),
        ("moveit_sparse_bag", 66.66666666666667, 66.66666666666667),
        ("autoware_bag", 98.66666666666667, 100),
        ("autoware_sensing_only_bag", 98.66666666666667, 100),
        ("autoware_vehicle_status_bag", 96.44736842105263, 100),
        ("robotarm_bag", 94.43181819318181, 100),
        ("perception_bag", 100, 100),
        ("generic_control_bag", 100, 100),
    ],
)
def test_framework_scores_remain_at_pre_readiness_baseline(request, fixture, overall, domain):
    result = evaluate_bag(str(request.getfixturevalue(fixture)))
    assert result.overall_score == pytest.approx(overall, rel=1e-12)
    assert result.domain_score == pytest.approx(domain, rel=1e-12)
    assert not any(f.id.startswith("plugin.") for f in result.findings)
    for finding in result.findings:
        jsonschema.validate(finding.to_dict(), findings_schema())


def test_new_framework_findings_work_in_benchmark_contracts(moveit_sparse_bag, tmp_path):
    manifest = tmp_path / "suite.json"
    manifest.write_text(
        json.dumps(
            {
                "cases": [
                    {
                        "name": "moveit",
                        "bag_path": str(moveit_sparse_bag),
                        "expect": {"expected_findings": ["moveit.missing_execution"]},
                    }
                ]
            }
        )
    )
    result = CliRunner().invoke(app, ["benchmark", str(manifest), "--exit-on", "warning"])
    assert result.exit_code == 1, result.output
    assert "worst finding severity" in result.output
