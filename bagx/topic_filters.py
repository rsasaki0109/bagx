"""Shared topic classification helpers."""

from __future__ import annotations

import numpy as np


def is_sync_candidate(topic_name: str, topic_type: str) -> bool:
    """Return whether a topic is meaningful for sensor/state sync analysis."""
    name = topic_name.lower()
    type_name = topic_type.lower()

    # Auxiliary readings that often live under an "imu" or "camera" namespace
    # (e.g. /imu/temperature) but are not sensor streams to align.
    excluded_type_parts = (
        "camerainfo",
        "magneticfield",
        "temperature",
        "fluidpressure",
        "relativehumidity",
        "illuminance",
        "batterystate",
    )
    if any(part in type_name for part in excluded_type_parts):
        return False

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
        "_service_event",
        "/result",
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


def nearest_neighbor_delays_ns(ts_a: np.ndarray, ts_b: np.ndarray) -> np.ndarray:
    """Distance from each message of the sparser stream to the nearest message of the denser one.

    Walking the denser stream instead would measure the sparser stream's period
    (a 200 Hz IMU against a 10 Hz LiDAR averages ~25 ms however well the two are
    aligned), and the result would depend on which topic name sorts first.
    Both inputs must be sorted int64 timestamp arrays.
    """
    sparse, dense = (ts_a, ts_b) if len(ts_a) <= len(ts_b) else (ts_b, ts_a)
    if len(sparse) == 0 or len(dense) == 0:
        return np.array([], dtype=np.int64)
    idx = np.searchsorted(dense, sparse)
    after = dense[np.minimum(idx, len(dense) - 1)]
    before = dense[np.maximum(idx - 1, 0)]
    return np.minimum(np.abs(after - sparse), np.abs(sparse - before))
