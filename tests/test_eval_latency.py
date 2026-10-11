"""Tests for header-stamp matched pipeline latency."""

from __future__ import annotations

from pathlib import Path

import pytest

from bagx.eval import evaluate_bag
from bagx.eval_latency import _add_pipeline_latency_recs, _stamp_matched_latencies
from tests.conftest import (
    _create_db3,
    build_laser_scan_cdr,
    build_odometry_cdr,
    build_pose_with_covariance_stamped_cdr,
)

BASE_NS = 1_700_000_000_000_000_000
IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _split(ns: int) -> tuple[int, int]:
    return ns // 1_000_000_000, ns % 1_000_000_000


def test_stamp_matched_latencies_exact() -> None:
    inputs = [(BASE_NS + i * 20_000_000, BASE_NS + i * 20_000_000) for i in range(10)]
    outputs = [(rec + 45_000_000, stamp) for rec, stamp in inputs]
    assert _stamp_matched_latencies(inputs, outputs, 5) == pytest.approx([45.0] * 10)


def test_stamp_matched_latencies_needs_most_outputs_to_match() -> None:
    inputs = [(BASE_NS + i, BASE_NS + i) for i in range(10)]
    outputs = [(BASE_NS + 100 + i, BASE_NS + 1_000 + i) for i in range(10)]  # never equal
    outputs[:3] = [(BASE_NS + 100 + i, BASE_NS + i) for i in range(3)]
    assert _stamp_matched_latencies(inputs, outputs, 1) is None
    assert _stamp_matched_latencies(None, outputs, 1) is None


def test_record_time_approximation_is_used_without_matching_stamps() -> None:
    ts = {
        "/in": [BASE_NS + i * 20_000_000 for i in range(20)],
        "/out": [BASE_NS + i * 20_000_000 + 5_000_000 for i in range(20)],
    }
    stamps = {"/in": [(t, t) for t in ts["/in"]], "/out": [(t, t) for t in ts["/out"]]}
    recs: list[str] = []
    _add_pipeline_latency_recs(recs, ts, [("/in", "/out", "in → out")], stamps=stamps)
    assert len(recs) == 1 and "5ms median" in recs[0]
    assert "header-stamp matched" not in recs[0]


def test_eval_reports_exact_latency_when_stamps_propagate(tmp_path: Path) -> None:
    """amcl_pose carries the odom stamp it was computed from, 45ms after odom.

    odom runs at 50Hz, so the record-time approximation ("latest odom before
    the output") would report ~5ms; the stamp match recovers the true 45ms.
    """
    topics = [
        {"name": "/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"},
        {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "format": "cdr"},
        {"name": "/amcl_pose", "type": "geometry_msgs/msg/PoseWithCovarianceStamped",
         "format": "cdr"},
    ]
    messages = []
    for i in range(100):
        t = BASE_NS + i * 20_000_000
        messages.append({"topic": "/odom", "timestamp_ns": t,
                         "data": build_odometry_cdr(*_split(t), *IDENTITY, (0, 0, 0), (0, 0, 0))})
        if i % 5 == 0:
            messages.append({"topic": "/scan", "timestamp_ns": t + 1_000_000,
                             "data": build_laser_scan_cdr(*_split(t), [1.0] * 90)})
            messages.append({"topic": "/amcl_pose", "timestamp_ns": t + 45_000_000,
                             "data": build_pose_with_covariance_stamped_cdr(*_split(t), *IDENTITY)})
    bag = _create_db3(tmp_path / "nav2.db3", topics,
                      sorted(messages, key=lambda m: m["timestamp_ns"]))
    recs = evaluate_bag(str(bag)).to_dict()["recommendations"]
    line = next(r for r in recs if "odom → localization" in r)
    assert "45ms median" in line
    assert "(header-stamp matched)" in line
