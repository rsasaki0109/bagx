"""Framework-specific (Nav2 / Autoware / MoveIt / ...) recommendations and scores."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bagx.custom_rules import CustomDomainResult
from bagx.eval_latency import _add_event_response_latency_recs, _add_pipeline_latency_recs
from bagx.eval_topics import (
    _format_rate_hz,
    _has_nav2_signature,
    _rate_goal_score,
    _select_action_feedback_topics,
    _select_action_result_topics,
    _select_action_status_topics,
    _select_control_command_topics,
    _select_generic_planning_topics,
    _select_service_event_topics,
    _select_topics,
)
from bagx.eval_workflow import _matching_workflow_metrics

if TYPE_CHECKING:
    from bagx.eval import EvalReport

def _detect_domain_recommendations(report: EvalReport) -> list[str]:
    """Detect ROS2 framework and generate domain-specific recommendations."""
    recs: list[str] = []
    topics = report.topic_info
    if not topics:
        return recs

    domains = _detect_domain_names(topics)
    topic_types = {t: info["type"] for t, info in topics.items()}
    image_topics = _select_topics(
        topics,
        type_markers=("image", "compressedimage"),
    )
    color_image_topics = [
        name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
    ]
    depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
    infra_image_topics = [name for name in image_topics if "infra" in name.lower()]
    camera_info_topics = _select_topics(
        topics,
        type_markers=("camerainfo",),
        contains=("camera_info",),
    )
    color_info_topics = [
        name for name in camera_info_topics if "depth" not in name.lower() and "infra" not in name.lower()
    ]
    depth_info_topics = [name for name in camera_info_topics if "depth" in name.lower()]
    infra_info_topics = [name for name in camera_info_topics if "infra" in name.lower()]

    odom_topics = _select_topics(
        topics,
        type_markers=("odometry",),
        suffixes=("/odom",),
        contains=("odometry/filtered", "/odometry/"),
    )
    scan_topics = _select_topics(
        topics,
        type_markers=("laserscan",),
        suffixes=("/scan",),
        contains=("/scan_",),
    )
    cmd_vel_topics = _select_topics(
        topics,
        type_markers=("twist", "twiststamped"),
        suffixes=("/cmd_vel", "/cmd_vel_smoothed"),
        contains=("cmd_vel",),
    )
    local_costmap_topics = _select_topics(
        topics,
        suffixes=("/local_costmap", "/local_costmap/costmap"),
        contains=("local_costmap",),
    )
    global_costmap_topics = _select_topics(
        topics,
        suffixes=("/global_costmap", "/global_costmap/costmap"),
        contains=("global_costmap",),
    )
    plan_topics = _select_topics(
        topics,
        suffixes=("/plan", "/global_plan"),
        contains=("planner_server/plan", "/plan", "trajectory"),
    )
    amcl_topics = _select_topics(
        topics,
        type_markers=("posewithcovariancestamped",),
        suffixes=("/amcl_pose",),
        contains=("amcl_pose",),
    )
    navigate_status_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/status",),
    )
    navigate_feedback_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/feedback",),
    )

    # --- Nav2 detection ---
    if _has_nav2_signature(topics):
        recs.append("[bold cyan]Nav2 topics detected[/bold cyan]")

        # Odom check
        for name in odom_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 20:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Odometry ({name}) at {rate:.0f}Hz — good for Nav2")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Odometry ({name}) at {rate:.0f}Hz — Nav2 works best at 20Hz+")

        # LaserScan check
        for name in scan_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 10:
                    recs.append(f"  [green]:heavy_check_mark:[/green] LaserScan ({name}) at {rate:.0f}Hz — good for costmap")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] LaserScan ({name}) at {rate:.0f}Hz — 10Hz+ recommended for costmap updates")

        # Costmap publication checks
        for name in local_costmap_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 1:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Local costmap ({name}) at {rate:.1f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Local costmap ({name}) at {rate:.1f}Hz — low refresh for local obstacle updates")
        for name in global_costmap_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 0.2:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Global costmap ({name}) at {rate:.1f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Global costmap ({name}) at {rate:.1f}Hz — planner map refresh is very sparse")

        # cmd_vel check
        for name in cmd_vel_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 10:
                recs.append(f"  [yellow]:warning:[/yellow] cmd_vel at {rate:.0f}Hz — control loop may be too slow")

        if plan_topics:
            for name in plan_topics:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Global plan ({name}) recorded {topics[name]['count']} times — planner output is visible"
                )
        else:
            recs.append(
                "  [yellow]:warning:[/yellow] No global plan topic recorded — planner activity is hard to inspect from this bag"
            )

        if navigate_status_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] NavigateToPose status ({navigate_status_topics[0]}) recorded — goal lifecycle is observable"
            )
        elif navigate_feedback_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] NavigateToPose feedback ({navigate_feedback_topics[0]}) recorded"
            )
        else:
            recs.append(
                "  [yellow]:warning:[/yellow] No NavigateToPose action topic recorded — goal-level debugging is limited"
            )

        # Nav2 pipeline latency
        ts = report._topic_timestamps
        nav2_pipeline = []
        if scan_topics and local_costmap_topics:
            nav2_pipeline.append((scan_topics[0], local_costmap_topics[0], "scan → costmap"))
        if odom_topics and amcl_topics:
            nav2_pipeline.append((odom_topics[0], amcl_topics[0], "odom → localization"))
        if scan_topics and cmd_vel_topics:
            nav2_pipeline.append((scan_topics[0], cmd_vel_topics[0], "scan → cmd_vel (full loop)"))
        _add_pipeline_latency_recs(recs, ts, nav2_pipeline, stamps=report._topic_stamps)
        if plan_topics and cmd_vel_topics:
            _add_event_response_latency_recs(
                recs,
                ts,
                [(plan_topics[0], cmd_vel_topics[0], "plan → cmd_vel onset")],
                min_input_samples=1,
                max_response_ms=1000.0,
            )

    generic_planning_topics = _select_generic_planning_topics(topics)
    generic_control_topics = _select_control_command_topics(topics)
    generic_action_status_topics = _select_action_status_topics(topics)
    generic_action_feedback_topics = _select_action_feedback_topics(topics)
    generic_action_result_topics = _select_action_result_topics(topics)
    generic_service_event_topics = _select_service_event_topics(topics)

    # --- Autoware detection ---
    autoware_prefixes = ("/sensing/", "/perception/", "/planning/", "/control/", "/localization/", "/vehicle/")
    autoware_topics = [
        name for name in topics if any(name.startswith(prefix) for prefix in autoware_prefixes)
    ]

    if len(autoware_topics) >= 1:
        recs.append("[bold cyan]Autoware topics detected[/bold cyan]")

        # Camera check
        camera_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
            prefixes=("/sensing/",),
            contains=("/camera/", "image"),
        )
        for name in camera_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 15:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Camera ({name}) at {rate:.0f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Camera ({name}) at {rate:.0f}Hz — 15Hz+ recommended for perception")

        # LiDAR check under /sensing
        lidar_topics = _select_topics(
            topics,
            type_markers=("pointcloud2", "velodynescan"),
            prefixes=("/sensing/",),
            contains=("lidar", "pointcloud", "velodyne_packets"),
        )
        for name in lidar_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                recs.append(f"  [green]:heavy_check_mark:[/green] LiDAR ({name}) at {rate:.0f}Hz")

        # GNSS under /sensing
        gnss_topics = _select_topics(
            topics,
            type_markers=("navsatfix",),
            prefixes=("/sensing/",),
            contains=("gnss",),
        )
        for name in gnss_topics:
            if "navsatfix" in topic_types[name].lower():
                recs.append(f"  [green]:heavy_check_mark:[/green] GNSS ({name})")

        vehicle_status_topics = _select_topics(
            topics,
            prefixes=("/vehicle/status/",),
            contains=("velocity_status", "steering_status", "gear_status"),
        ) or _select_topics(topics, prefixes=("/vehicle/status/",))
        for name in vehicle_status_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Vehicle status ({name}) at {rate:.0f}Hz — control/vehicle telemetry recorded"
                )

        # Autoware pipeline latency
        ts = report._topic_timestamps
        # Find actual topic names matching patterns
        sensing_lidar = lidar_topics
        perception = _select_topics(topics, prefixes=("/perception/",))
        planning = _select_topics(
            topics,
            prefixes=("/planning/",),
            contains=("trajectory", "path"),
        ) or _select_topics(topics, prefixes=("/planning/",))
        control = _select_topics(
            topics,
            prefixes=("/control/",),
            contains=("control_cmd", "command", "trajectory_follower"),
        ) or _select_topics(topics, prefixes=("/control/",))

        for name in planning:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                recs.append(f"  [green]:heavy_check_mark:[/green] Planning output ({name}) at {rate:.0f}Hz")

        for name in control:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 10:
                    recs.append(f"  [green]:heavy_check_mark:[/green] Control command ({name}) at {rate:.0f}Hz")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] Control command ({name}) at {rate:.0f}Hz — low for closed-loop vehicle control")

        if not planning and not control:
            recs.append(
                "  [dim]:information_source:[/dim] Sensing/localization-only Autoware bag — skipping planning/control checks"
            )
        elif not planning:
            recs.append(
                "  [yellow]:warning:[/yellow] Control topics are present but no planning topic was recorded"
            )
        elif not control:
            recs.append(
                "  [yellow]:warning:[/yellow] Planning topics are present but no control topic was recorded"
            )

        aw_pipeline = []
        if sensing_lidar and perception:
            aw_pipeline.append((sensing_lidar[0], perception[0], "sensing → perception"))
        if perception and planning:
            aw_pipeline.append((perception[0], planning[0], "perception → planning"))
        if planning and control:
            aw_pipeline.append((planning[0], control[0], "planning → control"))
        if sensing_lidar and control:
            aw_pipeline.append((sensing_lidar[0], control[0], "end-to-end (sensing → control)"))
        _add_pipeline_latency_recs(recs, ts, aw_pipeline, stamps=report._topic_stamps)

    # --- MoveIt detection ---
    joint_state_topics = _select_topics(
        topics,
        type_markers=("jointstate",),
        suffixes=("/joint_states",),
        contains=("joint_states",),
    )
    planned_path_topics = _select_topics(
        topics,
        type_markers=("displaytrajectory",),
        suffixes=("/display_planned_path",),
        contains=("display_planned_path", "planned_path"),
    )
    planning_scene_topics = _select_topics(
        topics,
        type_markers=("planningscene",),
        suffixes=("/planning_scene", "/monitored_planning_scene"),
        contains=("planning_scene", "monitored_planning_scene"),
    )
    move_group_result_topics = _select_topics(
        topics,
        suffixes=("/move_group/result",),
        contains=("move_group/result",),
    )
    move_action_status_topics = _select_topics(
        topics,
        contains=("move_action/_action/status",),
    )
    move_action_feedback_topics = _select_topics(
        topics,
        contains=("move_action/_action/feedback",),
    )
    execute_action_status_topics = _select_topics(
        topics,
        contains=("execute_trajectory/_action/status",),
    )
    execute_action_feedback_topics = _select_topics(
        topics,
        contains=("execute_trajectory/_action/feedback",),
    )
    controller_action_status_topics = _select_topics(
        topics,
        contains=("follow_joint_trajectory/_action/status",),
    )
    controller_action_feedback_topics = _select_topics(
        topics,
        contains=("follow_joint_trajectory/_action/feedback",),
    )

    moveit_evidence = sum(
        bool(matches)
        for matches in [
            joint_state_topics,
            planned_path_topics,
            planning_scene_topics,
            move_group_result_topics,
            move_action_status_topics,
            move_action_feedback_topics,
            execute_action_status_topics,
            execute_action_feedback_topics,
            controller_action_status_topics,
            controller_action_feedback_topics,
        ]
    )

    if moveit_evidence >= 2 or (
        joint_state_topics and (
            planned_path_topics
            or planning_scene_topics
            or move_group_result_topics
            or move_action_status_topics
            or execute_action_status_topics
            or controller_action_status_topics
        )
    ):
        recs.append("[bold cyan]MoveIt topics detected[/bold cyan]")

        for name in joint_state_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 100_000:  # sanity check for broken timestamps
                if rate >= 100:
                    recs.append(f"  [green]:heavy_check_mark:[/green] JointState ({name}) at {rate:.0f}Hz — good for motion planning")
                else:
                    recs.append(f"  [yellow]:warning:[/yellow] JointState ({name}) at {rate:.0f}Hz — 100Hz+ recommended for smooth trajectories")

        if move_action_status_topics or move_action_feedback_topics:
            name = (move_action_status_topics or move_action_feedback_topics)[0]
            recs.append(
                f"  [green]:heavy_check_mark:[/green] MoveGroup action activity recorded on {name}"
            )

        if execute_action_status_topics or execute_action_feedback_topics:
            name = (execute_action_status_topics or execute_action_feedback_topics)[0]
            recs.append(
                f"  [green]:heavy_check_mark:[/green] execute_trajectory action activity recorded on {name}"
            )

        if controller_action_status_topics or controller_action_feedback_topics:
            name = (controller_action_status_topics or controller_action_feedback_topics)[0]
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Joint trajectory controller activity recorded on {name}"
            )
        elif planned_path_topics:
            recs.append(
                "  [yellow]:warning:[/yellow] Planned path is present but trajectory execution was not recorded"
            )

        # MoveIt pipeline latency
        ts = report._topic_timestamps
        moveit_pipeline = []
        if joint_state_topics and planned_path_topics:
            moveit_pipeline.append((joint_state_topics[0], planned_path_topics[0], "joint_states → planned_path"))
        _add_pipeline_latency_recs(
            recs, ts, moveit_pipeline, min_output_samples=1, stamps=report._topic_stamps
        )
        if planned_path_topics and controller_action_status_topics:
            _add_event_response_latency_recs(
                recs,
                ts,
                [(planned_path_topics[0], controller_action_status_topics[0], "planned_path → arm execution")],
                min_input_samples=1,
                max_response_ms=5000.0,
            )
        elif planned_path_topics and execute_action_status_topics:
            _add_event_response_latency_recs(
                recs,
                ts,
                [(planned_path_topics[0], execute_action_status_topics[0], "planned_path → execute_trajectory")],
                min_input_samples=1,
                max_response_ms=5000.0,
            )

    if "Control" in domains and not ({"Nav2", "Autoware", "MoveIt"} & domains):
        recs.append("[bold cyan]Planning/control topics detected[/bold cyan]")

        for name in odom_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 20:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] State feedback ({name}) at {rate:.0f}Hz — good for closed-loop control"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] State feedback ({name}) at {rate:.0f}Hz — 20Hz+ recommended for closed-loop control"
                    )

        for name in joint_state_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 100_000:
                if rate >= 100:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] JointState ({name}) at {rate:.0f}Hz — good for actuator/state observability"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] JointState ({name}) at {rate:.0f}Hz — 100Hz+ recommended for responsive joint control"
                    )

        for name in generic_control_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                if rate >= 10:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Control command ({name}) at {rate:.0f}Hz — good for control-loop observability"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Control command ({name}) at {rate:.0f}Hz — 10Hz+ recommended for closed-loop control"
                    )

        if generic_planning_topics:
            for name in generic_planning_topics:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Planner output ({name}) recorded {topics[name]['count']} times — upstream planning is visible"
                )
        else:
            recs.append(
                "  [yellow]:warning:[/yellow] No planner output topic recorded — upstream planning is hard to inspect from this bag"
            )

        if generic_action_status_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Action status ({generic_action_status_topics[0]}) recorded — goal lifecycle is observable"
            )
        elif generic_action_feedback_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Action feedback ({generic_action_feedback_topics[0]}) recorded — goal progress is observable"
            )
        if generic_action_result_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Action result ({generic_action_result_topics[0]}) recorded — completion outcome is observable"
            )
        elif generic_action_status_topics or generic_action_feedback_topics:
            recs.append(
                "  [yellow]:warning:[/yellow] Action activity is present but no result topic was recorded — success/failure completion is hard to inspect"
            )

        if generic_service_event_topics:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Service event ({generic_service_event_topics[0]}) recorded — request/response auditing is possible"
            )

        control_pipelines = []
        if generic_planning_topics and generic_control_topics:
            control_pipelines.append((generic_planning_topics[0], generic_control_topics[0], "planner → command onset"))
        if generic_planning_topics and generic_action_result_topics:
            control_pipelines.append((generic_planning_topics[0], generic_action_result_topics[0], "planner → action result"))
        if generic_service_event_topics and generic_planning_topics:
            control_pipelines.append((generic_service_event_topics[0], generic_planning_topics[0], "service call → planner output"))
        _add_event_response_latency_recs(
            recs,
            report._topic_timestamps,
            control_pipelines,
            min_input_samples=1,
            max_response_ms=2000.0,
        )
        if generic_control_topics and odom_topics:
            _add_event_response_latency_recs(
                recs,
                report._topic_timestamps,
                [(generic_control_topics[0], odom_topics[0], "command → state feedback")],
                min_input_samples=3,
                max_response_ms=1000.0,
            )

    robot_arm_evidence = bool(joint_state_topics and (color_image_topics or depth_image_topics))
    if robot_arm_evidence and "MoveIt" not in domains:
        recs.append("[bold cyan]Robot arm perception/manipulation topics detected[/bold cyan]")

        for name in joint_state_topics:
            rate = topics[name]["rate_hz"]
            if 0 < rate < 100_000:
                if rate >= 100:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] JointState ({name}) at {rate:.0f}Hz — good for manipulation state tracking"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] JointState ({name}) at {rate:.0f}Hz — 100Hz+ recommended for responsive arm control"
                    )

        for name in color_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] RGB image ({name}) at {rate_label}Hz — good for manipulation perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] RGB image ({name}) at {rate_label}Hz — 15Hz+ recommended for manipulation perception"
                    )

        for name in depth_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Depth image ({name}) at {rate_label}Hz — good for RGB-D grasp perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Depth image ({name}) at {rate_label}Hz — 15Hz+ recommended for RGB-D grasp perception"
                    )

        if color_image_topics and depth_image_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] RGB and depth streams are both recorded — suitable for open-loop manipulation perception benchmarking"
            )
        if color_info_topics or depth_info_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] Camera calibration topics are recorded — exported perception data is reusable"
            )

    perception_evidence = bool(camera_info_topics and (color_image_topics or depth_image_topics or infra_image_topics))
    if perception_evidence and not ({"Autoware", "MoveIt", "RobotArm"} & domains):
        recs.append("[bold cyan]Perception topics detected[/bold cyan]")

        for name in color_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] RGB image ({name}) at {rate_label}Hz — good for camera-based perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] RGB image ({name}) at {rate_label}Hz — 15Hz+ recommended for camera-based perception"
                    )

        for name in depth_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Depth image ({name}) at {rate_label}Hz — good for RGB-D perception"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Depth image ({name}) at {rate_label}Hz — 15Hz+ recommended for RGB-D perception"
                    )

        for name in infra_image_topics:
            rate = topics[name]["rate_hz"]
            if rate > 0:
                rate_label = _format_rate_hz(rate)
                if rate >= 15:
                    recs.append(
                        f"  [green]:heavy_check_mark:[/green] Infra image ({name}) at {rate_label}Hz — good for depth/perception debugging"
                    )
                else:
                    recs.append(
                        f"  [yellow]:warning:[/yellow] Infra image ({name}) at {rate_label}Hz — 15Hz+ recommended for stereo/depth debugging"
                    )

        if color_image_topics and depth_image_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] RGB and depth streams are both recorded — suitable for RGB-D perception benchmarking"
            )
        if len(infra_image_topics) >= 2:
            recs.append(
                "  [green]:heavy_check_mark:[/green] Infra stereo streams are both recorded — depth debugging is possible"
            )
        if color_info_topics or depth_info_topics or infra_info_topics:
            recs.append(
                "  [green]:heavy_check_mark:[/green] Camera calibration topics are recorded — exported perception data is reusable"
            )

    return recs


def detect_domain_names(
    topics: dict[str, dict],
    custom_domains: list[CustomDomainResult] | None = None,
) -> set[str]:
    """Return built-in plus custom domain names inferred from a bag report."""
    names = _detect_domain_names(topics)
    if custom_domains:
        names.update(domain.name for domain in custom_domains)
    return names


def _detect_domain_names(topics: dict[str, dict]) -> set[str]:
    """Return high-level framework names inferred from topic names/types."""
    if not topics:
        return set()

    from bagx.domain_plugins import detect_domains

    return {plugin.name for plugin in detect_domains(topics)}


def _compute_domain_score(report: EvalReport) -> float | None:
    """Compute framework-aware score when generic SLAM metrics are not representative."""
    topics = report.topic_info
    if not topics:
        return None

    scores: list[float] = []
    domains = detect_domain_names(topics, report.custom_domains)

    if "Nav2" in domains:
        odom_topics = _select_topics(
            topics,
            type_markers=("odometry",),
            suffixes=("/odom",),
            contains=("odometry/filtered", "/odometry/"),
        )
        scan_topics = _select_topics(
            topics,
            type_markers=("laserscan",),
            suffixes=("/scan",),
            contains=("/scan_",),
        )
        local_costmap_topics = _select_topics(
            topics,
            suffixes=("/local_costmap", "/local_costmap/costmap"),
            contains=("local_costmap",),
        )
        global_costmap_topics = _select_topics(
            topics,
            suffixes=("/global_costmap", "/global_costmap/costmap"),
            contains=("global_costmap",),
        )
        plan_topics = _select_topics(
            topics,
            suffixes=("/plan", "/global_plan"),
            contains=("planner_server/plan", "/plan", "trajectory"),
        )
        navigate_topics = _select_topics(
            topics,
            contains=("navigate_to_pose/_action/status", "navigate_to_pose/_action/feedback"),
        )

        nav2_scores = []
        if odom_topics:
            nav2_scores.append(_rate_goal_score(topics[odom_topics[0]]["rate_hz"], 20.0))
        if scan_topics:
            nav2_scores.append(_rate_goal_score(topics[scan_topics[0]]["rate_hz"], 10.0))
        if local_costmap_topics:
            nav2_scores.append(_rate_goal_score(topics[local_costmap_topics[0]]["rate_hz"], 1.0))
        if global_costmap_topics:
            nav2_scores.append(_rate_goal_score(topics[global_costmap_topics[0]]["rate_hz"], 0.2))
        nav2_scores.append(100.0 if plan_topics else 0.0)
        nav2_scores.append(100.0 if navigate_topics else 0.0)
        nav2_scores.extend(
            metric.score
            for metric in _matching_workflow_metrics(report.workflow_metrics, contains=("navigate_to_pose",))
            if metric.score is not None
        )
        scores.append(float(np.mean(nav2_scores)))

    if "Autoware" in domains:
        camera_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
            prefixes=("/sensing/",),
            contains=("/camera/", "image"),
        )
        lidar_topics = _select_topics(
            topics,
            type_markers=("pointcloud2", "velodynescan"),
            prefixes=("/sensing/",),
            contains=("lidar", "pointcloud", "velodyne_packets"),
        )
        gnss_topics = _select_topics(
            topics,
            type_markers=("navsatfix",),
            prefixes=("/sensing/",),
            contains=("gnss",),
        )
        vehicle_status_topics = _select_topics(
            topics,
            prefixes=("/vehicle/status/",),
            contains=("velocity_status", "steering_status", "gear_status"),
        ) or _select_topics(topics, prefixes=("/vehicle/status/",))
        planning_topics = _select_topics(
            topics,
            prefixes=("/planning/",),
            contains=("trajectory", "path"),
        ) or _select_topics(topics, prefixes=("/planning/",))
        control_topics = _select_topics(
            topics,
            prefixes=("/control/",),
            contains=("control_cmd", "command", "trajectory_follower"),
        ) or _select_topics(topics, prefixes=("/control/",))

        autoware_scores = []
        autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in camera_topics)
        autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 10.0) for name in lidar_topics)
        autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 20.0) for name in vehicle_status_topics)
        if gnss_topics:
            autoware_scores.append(100.0)
        if planning_topics or control_topics:
            autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 5.0) for name in planning_topics)
            autoware_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 10.0) for name in control_topics)
            if planning_topics and not control_topics:
                autoware_scores.append(0.0)
            if control_topics and not planning_topics:
                autoware_scores.append(0.0)
        if autoware_scores:
            scores.append(float(np.mean(autoware_scores)))

    if "RobotArm" in domains:
        joint_state_topics = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        image_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
        )
        color_image_topics = [
            name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
        ]
        depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
        camera_info_topics = _select_topics(
            topics,
            type_markers=("camerainfo",),
            contains=("camera_info",),
        )

        robot_arm_scores = []
        if joint_state_topics:
            robot_arm_scores.append(_rate_goal_score(topics[joint_state_topics[0]]["rate_hz"], 100.0))
        robot_arm_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in color_image_topics)
        robot_arm_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in depth_image_topics)
        if color_image_topics and depth_image_topics:
            robot_arm_scores.append(100.0)
        if camera_info_topics:
            robot_arm_scores.append(100.0)
        if robot_arm_scores:
            scores.append(float(np.mean(robot_arm_scores)))

    if "Perception" in domains:
        image_topics = _select_topics(
            topics,
            type_markers=("image", "compressedimage"),
        )
        color_image_topics = [
            name for name in image_topics if "depth" not in name.lower() and "infra" not in name.lower()
        ]
        depth_image_topics = [name for name in image_topics if "depth" in name.lower()]
        infra_image_topics = [name for name in image_topics if "infra" in name.lower()]
        camera_info_topics = _select_topics(
            topics,
            type_markers=("camerainfo",),
            contains=("camera_info",),
        )

        perception_scores = []
        perception_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in color_image_topics)
        perception_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in depth_image_topics)
        perception_scores.extend(_rate_goal_score(topics[name]["rate_hz"], 15.0) for name in infra_image_topics)
        if color_image_topics and depth_image_topics:
            perception_scores.append(100.0)
        if len(infra_image_topics) >= 2:
            perception_scores.append(100.0)
        if camera_info_topics:
            perception_scores.append(100.0)
        if perception_scores:
            scores.append(float(np.mean(perception_scores)))

    if "Control" in domains:
        odom_topics = _select_topics(
            topics,
            type_markers=("odometry",),
            suffixes=("/odom",),
            contains=("odometry/filtered", "/odometry/"),
        )
        joint_state_topics = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        control_command_topics = _select_control_command_topics(topics)
        planning_topics = _select_generic_planning_topics(topics)
        action_status_topics = _select_action_status_topics(topics)
        action_feedback_topics = _select_action_feedback_topics(topics)
        action_result_topics = _select_action_result_topics(topics)
        service_event_topics = _select_service_event_topics(topics)

        control_scores = []
        if odom_topics:
            control_scores.append(_rate_goal_score(topics[odom_topics[0]]["rate_hz"], 20.0))
        if joint_state_topics:
            control_scores.append(_rate_goal_score(topics[joint_state_topics[0]]["rate_hz"], 100.0))
        if control_command_topics:
            control_scores.append(_rate_goal_score(topics[control_command_topics[0]]["rate_hz"], 10.0))
        if planning_topics:
            control_scores.append(100.0)
        workflow_visibility = 0.0
        if action_status_topics or action_feedback_topics:
            workflow_visibility += 40.0
        if action_result_topics:
            workflow_visibility += 40.0
        if service_event_topics:
            workflow_visibility += 20.0
        if workflow_visibility > 0:
            control_scores.append(min(100.0, workflow_visibility))
        control_scores.extend(
            metric.score for metric in report.workflow_metrics if metric.score is not None
        )
        if control_scores:
            scores.append(float(np.mean(control_scores)))

    if "MoveIt" in domains:
        joint_state_topics = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        planned_path_topics = _select_topics(
            topics,
            type_markers=("displaytrajectory",),
            suffixes=("/display_planned_path",),
            contains=("display_planned_path", "planned_path"),
        )
        move_action_topics = _select_topics(
            topics,
            contains=("move_action/_action/status", "move_action/_action/feedback"),
        )
        execute_action_topics = _select_topics(
            topics,
            contains=("execute_trajectory/_action/status", "execute_trajectory/_action/feedback"),
        )
        controller_action_topics = _select_topics(
            topics,
            contains=("follow_joint_trajectory/_action/status", "follow_joint_trajectory/_action/feedback"),
        )

        moveit_scores = []
        if joint_state_topics:
            moveit_scores.append(_rate_goal_score(topics[joint_state_topics[0]]["rate_hz"], 100.0))
        moveit_scores.append(100.0 if planned_path_topics else 0.0)
        execution_observable = bool(controller_action_topics or execute_action_topics)
        if planned_path_topics:
            moveit_scores.append(100.0 if execution_observable else 0.0)
        if move_action_topics:
            moveit_scores.append(100.0)
        if controller_action_topics:
            moveit_scores.append(100.0)
        elif execute_action_topics:
            moveit_scores.append(85.0)
        moveit_scores.extend(
            metric.score
            for metric in _matching_workflow_metrics(
                report.workflow_metrics,
                contains=("move_action", "execute_trajectory", "follow_joint_trajectory", "move_group"),
            )
            if metric.score is not None
        )
        scores.append(float(np.mean(moveit_scores)))

    for domain in report.custom_domains:
        scores.append(domain.score)

    return max(scores) if scores else None
