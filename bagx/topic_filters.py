"""Shared topic classification helpers."""

from __future__ import annotations


def is_sync_candidate(topic_name: str, topic_type: str) -> bool:
    """Return whether a topic is meaningful for sensor/state sync analysis."""
    name = topic_name.lower()
    type_name = topic_type.lower()

    excluded_name_parts = (
        "/clock",
        "clock",
        "cmd_vel",
        "costmap",
        "parameter_events",
        "rosout",
        "diagnostics",
        "feedback",
        "status",
        "goal",
        "event",
    )
    if any(part in name for part in excluded_name_parts):
        return False

    included_type_parts = (
        "navsatfix",
        "imu",
        "laserscan",
        "pointcloud",
        "image",
    )
    if any(part in type_name for part in included_type_parts):
        return True

    included_name_parts = (
        "gnss",
        "imu",
        "scan",
        "lidar",
        "pointcloud",
        "camera",
        "image",
    )
    return any(part in name for part in included_name_parts)


def is_rate_anomaly_candidate(topic_name: str, topic_type: str) -> bool:
    """Return whether a topic is meaningful for generic rate-gap anomaly checks."""
    name = topic_name.lower()
    type_name = topic_type.lower()

    if name in {"/tf", "/tf_static"}:
        return False

    excluded_name_parts = (
        "/clock",
        "/tf_static",
        "planning_scene",
        "parameter_events",
        "rosout",
        "statistics",
        "transition_event",
        "_action/status",
        "_action/feedback",
        "_action/get_result",
        "_action/send_goal",
        "_action/cancel_goal",
    )
    if any(part in name for part in excluded_name_parts):
        return False

    excluded_type_parts = (
        "goalstatusarray",
        "clock",
        "log",
        "parameterevent",
        "planningscene",
    )
    return not any(part in type_name for part in excluded_type_parts)
