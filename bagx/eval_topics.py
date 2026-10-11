"""Topic selection helpers shared by eval, domain detection and plugins."""

from __future__ import annotations

def _rate_goal_score(rate_hz: float, target_hz: float) -> float:
    """Normalize a topic rate against a practical target."""
    if rate_hz <= 0 or target_hz <= 0:
        return 0.0
    return float(max(0.0, min(100.0, (rate_hz / target_hz) * 100.0)))


def _format_rate_hz(rate_hz: float) -> str:
    """Format a rate without hiding threshold-relevant decimals."""
    if rate_hz < 20 and abs(rate_hz - round(rate_hz)) >= 0.05:
        return f"{rate_hz:.1f}"
    return f"{rate_hz:.0f}"


def _select_topics(
    topics: dict[str, dict],
    *,
    type_markers: tuple[str, ...] = (),
    prefixes: tuple[str, ...] = (),
    suffixes: tuple[str, ...] = (),
    contains: tuple[str, ...] = (),
) -> list[str]:
    """Return topic names matching type/name heuristics, sorted by rate desc."""
    matches = []
    normalized_type_markers = tuple(marker.lower() for marker in type_markers)
    normalized_prefixes = tuple(prefix.lower() for prefix in prefixes)
    normalized_suffixes = tuple(suffix.lower() for suffix in suffixes)
    normalized_contains = tuple(fragment.lower() for fragment in contains)

    for name, info in topics.items():
        if int(info.get("count", 0) or 0) <= 0:
            continue
        lower_name = name.lower()
        lower_type = str(info.get("type", "")).lower()

        if normalized_type_markers and not any(marker in lower_type for marker in normalized_type_markers):
            continue

        has_name_filters = bool(normalized_prefixes or normalized_suffixes or normalized_contains)
        if has_name_filters:
            name_match = (
                any(lower_name.startswith(prefix) for prefix in normalized_prefixes)
                or any(lower_name == suffix or lower_name.endswith(suffix) for suffix in normalized_suffixes)
                or any(fragment in lower_name for fragment in normalized_contains)
            )
            if not name_match:
                continue

        matches.append(name)

    return sorted(matches, key=lambda name: (-topics[name]["rate_hz"], name))


def _select_control_command_topics(topics: dict[str, dict]) -> list[str]:
    """Select generic control command topics based on standard ROS messages."""
    return _select_topics(
        topics,
        type_markers=("twist", "twiststamped", "ackermann", "jointtrajectory"),
        suffixes=("/cmd_vel", "/cmd_vel_smoothed", "/command", "/trajectory_cmd"),
        contains=("cmd_vel", "control_cmd", "/command", "_cmd", "ackermann", "trajectory_cmd"),
    )


def _select_generic_planning_topics(topics: dict[str, dict]) -> list[str]:
    """Select generic planning output topics without assuming a framework."""
    matches = _select_topics(
        topics,
        type_markers=("path", "trajectory"),
        suffixes=("/path", "/trajectory"),
        contains=("/path", "_path", "trajectory"),
    )
    return [
        name
        for name in matches
        if "_service_event" not in name.lower() and "_action/" not in name.lower()
    ]


def _select_action_status_topics(topics: dict[str, dict]) -> list[str]:
    """Select generic action status topics."""
    return _select_topics(
        topics,
        contains=("_action/status",),
    )


def _select_action_feedback_topics(topics: dict[str, dict]) -> list[str]:
    """Select generic action feedback topics."""
    return _select_topics(
        topics,
        contains=("_action/feedback",),
    )


def _select_action_result_topics(topics: dict[str, dict]) -> list[str]:
    """Select generic action result topics or hidden get_result channels."""
    return _select_topics(
        topics,
        suffixes=("/result",),
        contains=("_action/get_result", "_action/result"),
    )


def _select_service_event_topics(topics: dict[str, dict]) -> list[str]:
    """Select generic service event topics."""
    return _select_topics(
        topics,
        suffixes=("/_service_event",),
        contains=("_service_event",),
    )


def _has_nav2_signature(topics: dict[str, dict]) -> bool:
    """Return whether topics look like Nav2 rather than generic control."""
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
        contains=("planner_server/plan", "/plan"),
    )
    amcl_topics = _select_topics(
        topics,
        type_markers=("posewithcovariancestamped",),
        suffixes=("/amcl_pose",),
        contains=("amcl_pose",),
    )
    map_topics = _select_topics(
        topics,
        type_markers=("occupancygrid",),
        suffixes=("/map",),
        contains=("/map",),
    )
    behavior_tree_topics = _select_topics(
        topics,
        suffixes=("/behavior_tree_log",),
        contains=("behavior_tree_log",),
    )
    navigate_status_topics = _select_action_status_topics({
        name: info for name, info in topics.items() if "navigate_to_pose" in name.lower()
    })
    navigate_feedback_topics = _select_action_feedback_topics({
        name: info for name, info in topics.items() if "navigate_to_pose" in name.lower()
    })

    nav2_specific = sum(
        bool(matches)
        for matches in [
            local_costmap_topics,
            global_costmap_topics,
            amcl_topics,
            map_topics,
            behavior_tree_topics,
            navigate_status_topics,
            navigate_feedback_topics,
        ]
    )
    nav2_support = sum(
        bool(matches)
        for matches in [
            odom_topics,
            scan_topics,
            cmd_vel_topics,
            plan_topics,
        ]
    )
    return (nav2_specific >= 1 and nav2_support >= 1) or bool(scan_topics and odom_topics and plan_topics)


def _has_control_signature(topics: dict[str, dict]) -> bool:
    """Return whether topics look like a generic planning/control loop."""
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

    state_feedback = bool(odom_topics or joint_state_topics)
    action_activity = bool(action_status_topics or action_feedback_topics)
    return (
        (state_feedback and bool(control_command_topics))
        or (bool(planning_topics) and bool(control_command_topics))
        or (state_feedback and action_activity)
        or (bool(planning_topics) and bool(action_result_topics))
        or (bool(planning_topics) and bool(service_event_topics))
    )
