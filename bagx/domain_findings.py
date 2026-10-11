"""Structured readiness checks for the built-in domain plugin hooks.

These findings expose the existing domain rate targets; they do not rescore.
Nav2 keeps its existing finding ids in eval_findings for compatibility.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from bagx.eval_topics import (
    _select_control_command_topics,
    _select_generic_planning_topics,
    _select_topics,
)
from bagx.findings import Evidence, Finding, clean_topic_token, finding_id

if TYPE_CHECKING:
    from bagx.eval import EvalReport


def _rates(
    report: EvalReport, domain: str, label: str, names: list[str], goal: float
) -> list[Finding]:
    findings = []
    for name in names:
        rate = float(report.topic_info[name].get("rate_hz", 0.0) or 0.0)
        unavailable = not math.isfinite(rate) or rate < 0 or rate > 100_000
        state = "rate_unavailable" if unavailable else "rate_low" if rate < goal else "rate_good"
        findings.append(
            Finding(
                id=finding_id(domain, label, clean_topic_token(name), state),
                title=f"{domain} {label} ({name}): {state.replace('_', ' ')}",
                severity="info" if state == "rate_good" else "warning",
                category="rate_quality",
                domain=domain.lower(),
                affected_topics=[name],
                evidence=[
                    Evidence(
                        metric="rate_hz",
                        observed=round(rate, 3) if math.isfinite(rate) else None,
                        expected=f">={goal:g}",
                        unit="Hz",
                        topic=name,
                    )
                ],
                suggested_action=None
                if state == "rate_good"
                else (
                    "Check timestamp validity and recording duration before interpreting the rate."
                    if unavailable
                    else f"Check the publisher and record {label} near {goal:g}Hz or higher."
                ),
                confidence="medium",
            )
        )
    return findings


def _missing(domain: str, label: str, topic: str, *, workflow: bool = False) -> Finding:
    return Finding(
        id=finding_id(domain, f"missing_{label}"),
        title=f"{domain}: {label.replace('_', ' ')} is not recorded",
        severity="warning",
        category="workflow_observability" if workflow else "topic_presence",
        domain=domain.lower(),
        affected_topics=[topic],
        evidence=[Evidence(metric="topic_present", observed=False, expected=True, topic=topic)],
        suggested_action=f"Record {label.replace('_', ' ')} to assess this part of the pipeline.",
        confidence="medium",
    )


def generate_domain_readiness_findings(report: EvalReport, *, domain: str) -> list[Finding]:
    topics = report.topic_info
    findings: list[Finding] = []
    joints = _select_topics(
        topics,
        type_markers=("jointstate",),
        suffixes=("/joint_states",),
        contains=("joint_states",),
    )

    if domain == "Autoware":
        camera = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
            prefixes=("/sensing/",),
            contains=("/camera/", "image"),
        )
        lidar = _select_topics(
            topics,
            type_markers=("pointcloud2", "velodynescan"),
            prefixes=("/sensing/",),
            contains=("lidar", "pointcloud", "velodyne_packets"),
        )
        vehicle = _select_topics(
            topics,
            prefixes=("/vehicle/status/",),
            contains=("velocity_status", "steering_status", "gear_status"),
        )
        vehicle = vehicle or _select_topics(topics, prefixes=("/vehicle/status/",))
        planning = _select_topics(topics, prefixes=("/planning/",), contains=("trajectory", "path"))
        planning = planning or _select_topics(topics, prefixes=("/planning/",))
        planning = [name for name in planning if name.startswith("/planning/")]
        control = _select_topics(
            topics,
            prefixes=("/control/",),
            contains=("control_cmd", "command", "trajectory_follower"),
        )
        control = control or _select_topics(topics, prefixes=("/control/",))
        control = [name for name in control if name.startswith("/control/")]
        for names, goal, label in [
            (camera, 15, "camera"),
            (lidar, 10, "lidar"),
            (vehicle, 20, "vehicle_status"),
            (planning, 5, "planning"),
            (control, 10, "control"),
        ]:
            findings.extend(_rates(report, domain, label, names, goal))
        # Sensing-only bags are legitimate. Require the counterpart only when
        # one of the planning/control stages was actually recorded.
        if planning and not control:
            findings.append(_missing(domain, "control", "/control/command/control_cmd"))
        if control and not planning:
            findings.append(_missing(domain, "planning", "/planning/scenario_planning/trajectory"))

    elif domain in {"Perception", "RobotArm"}:
        images = _select_topics(topics, type_markers=("image", "compressedimage"))
        if domain == "RobotArm":
            findings.extend(_rates(report, domain, "joint_states", joints[:1], 100))
            # Match RobotArm's color/depth selection; plain infra is not scored.
            images = [
                name for name in images if "infra" not in name.lower() or "depth" in name.lower()
            ]
        findings.extend(_rates(report, domain, "image", images, 15))
        camera_info = _select_topics(
            topics, type_markers=("camerainfo",), contains=("camera_info",)
        )
        if not camera_info:
            findings.append(_missing(domain, "camera_info", "camera_info"))

    elif domain == "MoveIt":
        findings.extend(_rates(report, domain, "joint_states", joints[:1], 100))
        planned = _select_topics(
            topics,
            type_markers=("displaytrajectory",),
            suffixes=("/display_planned_path",),
            contains=("display_planned_path", "planned_path"),
        )
        execution = _select_topics(
            topics,
            contains=(
                "execute_trajectory/_action/status",
                "execute_trajectory/_action/feedback",
                "follow_joint_trajectory/_action/status",
                "follow_joint_trajectory/_action/feedback",
            ),
        )
        if not planned:
            findings.append(_missing(domain, "planned_path", "/display_planned_path"))
        elif not execution:
            findings.append(
                _missing(domain, "execution", "/execute_trajectory/_action/status", workflow=True)
            )

    elif domain == "Control":
        odom = _select_topics(
            topics,
            type_markers=("odometry",),
            suffixes=("/odom",),
            contains=("odometry/filtered", "/odometry/"),
        )
        commands = _select_control_command_topics(topics)
        for names, goal, label in [
            (odom[:1], 20, "odometry"),
            (joints[:1], 100, "joint_states"),
            (commands[:1], 10, "command"),
        ]:
            findings.extend(_rates(report, domain, label, names, goal))
        if commands and not _select_generic_planning_topics(topics):
            findings.append(_missing(domain, "planning", "/plan"))
    return findings
