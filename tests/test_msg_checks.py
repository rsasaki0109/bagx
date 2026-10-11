"""Tests for bagx.msg_checks (message content checks)."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from bagx.eval import evaluate_bag
from bagx.msg_checks import MessageCheckAccumulator
from bagx.reader import Message
from tests.conftest import (
    _create_db3,
    build_camera_info_cdr,
    build_image_cdr,
    build_joint_state_cdr,
    build_laser_scan_cdr,
)

BASE_NS = 1_700_000_000_000_000_000
TOPICS = [
    {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "format": "cdr"},
    {"name": "/camera/color/camera_info", "type": "sensor_msgs/msg/CameraInfo", "format": "cdr"},
    {"name": "/camera/color/image_raw", "type": "sensor_msgs/msg/Image", "format": "cdr"},
    {"name": "/joint_states", "type": "sensor_msgs/msg/JointState", "format": "cdr"},
]


def _bag(tmp_path: Path, messages: list[dict], name: str = "msg.db3") -> str:
    return str(_create_db3(tmp_path / name, TOPICS, messages))


def _msg(topic: str, i: int, data: bytes) -> dict:
    return {"topic": topic, "timestamp_ns": BASE_NS + i * 50_000_000, "data": data}


def _healthy(n: int = 20) -> list[dict]:
    messages = []
    for i in range(n):
        sec = BASE_NS // 1_000_000_000
        messages.append(_msg("/scan", i, build_laser_scan_cdr(sec, i, [1.0 + i % 3] * 90)))
        messages.append(_msg("/camera/color/camera_info", i, build_camera_info_cdr(sec, i, 4, 2)))
        messages.append(_msg("/camera/color/image_raw", i, build_image_cdr(sec, i, 4, 2)))
        messages.append(_msg("/joint_states", i, build_joint_state_cdr(sec, i, ["a", "b"])))
    return messages


def _ids(path: str) -> set[str]:
    return {f.id for f in evaluate_bag(path).findings if f.category == "message_content"}


def test_healthy_bag_has_no_content_findings(tmp_path: Path) -> None:
    assert _ids(_bag(tmp_path, _healthy())) == set()


def test_scan_with_mostly_invalid_ranges(tmp_path: Path) -> None:
    messages = [m for m in _healthy() if m["topic"] != "/scan"]
    for i in range(20):
        ranges = [float("inf")] * 70 + [2.0] * 20
        messages.append(_msg("/scan", i, build_laser_scan_cdr(1, i, ranges)))
    assert "msg.scan_low_valid_ranges.scan" in _ids(_bag(tmp_path, messages))


def test_scan_with_no_valid_ranges_and_bad_config(tmp_path: Path) -> None:
    messages = [m for m in _healthy() if m["topic"] != "/scan"]
    for i in range(5):
        messages.append(_msg("/scan", i, build_laser_scan_cdr(
            1, i, [float("nan")] * 10, range_min=5.0, range_max=1.0)))
    ids = _ids(_bag(tmp_path, messages))
    assert {"msg.scan_no_valid_ranges.scan", "msg.scan_config.scan"} <= ids


def test_scan_beam_count_change_and_empty_frame(tmp_path: Path) -> None:
    messages = [m for m in _healthy() if m["topic"] != "/scan"]
    for i in range(10):
        messages.append(_msg("/scan", i, build_laser_scan_cdr(
            1, i, [1.0] * (90 if i < 5 else 45), frame_id="" if i == 3 else "laser")))
    ids = _ids(_bag(tmp_path, messages))
    assert {"msg.scan_beam_count_changes.scan", "msg.empty_frame_id.scan"} <= ids


def test_uncalibrated_and_mismatched_camera_info(tmp_path: Path) -> None:
    messages = [m for m in _healthy() if "camera_info" not in m["topic"]]
    for i in range(5):
        messages.append(_msg("/camera/color/camera_info", i,
                             build_camera_info_cdr(1, i, 1280, 720, k=[0.0] * 9)))
    ids = _ids(_bag(tmp_path, messages))
    assert "msg.camera_uncalibrated.camera.color.camera_info" in ids
    assert "msg.camera_info_size_mismatch.camera.color.camera_info" in ids


def test_truncated_and_reconfigured_images(tmp_path: Path) -> None:
    messages = [m for m in _healthy() if m["topic"] != "/camera/color/image_raw"]
    for i in range(6):
        if i < 3:
            data = build_image_cdr(1, i, 4, 2)
        else:
            data = build_image_cdr(1, i, 4, 2, encoding="mono8", data_size=10)
        messages.append(_msg("/camera/color/image_raw", i, data))
    ids = _ids(_bag(tmp_path, messages))
    assert {"msg.image_size_mismatch.camera.color.image_raw",
            "msg.image_format_changes.camera.color.image_raw"} <= ids


def test_joint_states_from_two_publishers(tmp_path: Path) -> None:
    messages = [m for m in _healthy() if m["topic"] != "/joint_states"]
    for i in range(10):
        names = ["a", "b"] if i % 2 else ["gripper"]
        messages.append(_msg("/joint_states", i, build_joint_state_cdr(1, i, names)))
    messages.append(_msg("/joint_states", 11, build_joint_state_cdr(1, 11, ["a", "b"], [0.0])))
    messages.append(_msg("/joint_states", 12, build_joint_state_cdr(1, 12, ["a", "b"],
                                                                    [math.nan, 0.0])))
    ids = _ids(_bag(tmp_path, messages))
    assert {"msg.joint_state_name_set_changes.joint_states",
            "msg.joint_state_length_mismatch.joint_states",
            "msg.joint_state_nan.joint_states"} <= ids


def test_decoded_message_shapes() -> None:
    """mcap / rosbag2_py / rosbags dicts carry full arrays and nested headers."""
    acc = MessageCheckAccumulator({
        "/scan": "sensor_msgs/msg/LaserScan",
        "/img": "sensor_msgs/msg/Image",
        "/info": "sensor_msgs/msg/CameraInfo",
    })
    header = {"stamp": {"sec": 1, "nanosec": 0}, "frame_id": "laser"}
    acc.add(Message("/scan", BASE_NS, {
        "header": header, "angle_min": -1.0, "angle_max": 1.0, "angle_increment": 0.1,
        "range_min": 0.1, "range_max": 10.0, "ranges": [float("inf")] * 9 + [1.0],
    }))
    acc.add(Message("/img", BASE_NS, {
        "header": header, "width": 2, "height": 2, "step": 6, "encoding": "rgb8",
        "data": b"\x00" * 5,
    }))
    acc.add(Message("/info", BASE_NS, {"header": header, "width": 2, "height": 2,
                                       "k": [0.0] * 9}))
    ids = {f.id for f in acc.build()}
    assert ids == {"msg.scan_low_valid_ranges.scan", "msg.image_size_mismatch.img",
                   "msg.camera_uncalibrated.info"}


def test_content_findings_do_not_change_score_and_show_in_recs(tmp_path: Path) -> None:
    healthy = evaluate_bag(_bag(tmp_path, _healthy(), "a.db3"))
    messages = [m for m in _healthy() if "camera_info" not in m["topic"]]
    messages += [_msg("/camera/color/camera_info", i, build_camera_info_cdr(1, i, 4, 2, k=[0.0] * 9))
                 for i in range(20)]
    broken = evaluate_bag(_bag(tmp_path, messages, "b.db3"))
    assert broken.overall_score == pytest.approx(healthy.overall_score)
    recs = broken.to_dict()["recommendations"]
    assert any(r.startswith("WARNING Data: /camera/color/camera_info has an empty intrinsic")
               for r in recs)
