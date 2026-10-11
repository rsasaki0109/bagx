"""Tests for bagx.tf_tree (TF tree validation)."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from typer.testing import CliRunner

from bagx.cli import app
from bagx.eval import evaluate_bag
from bagx.reader import Message
from bagx.tf_tree import TfAccumulator, analyze_tf
from tests.conftest import _create_db3, build_imu_cdr, build_tf_message_cdr

BASE_NS = 1_700_000_000_000_000_000
BASE_SEC = BASE_NS // 1_000_000_000
TF_TOPICS = [
    {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "format": "cdr"},
    {"name": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "format": "cdr"},
]
IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _tf(ts_ns: int, parent: str, child: str, topic: str = "/tf", stamp_ns: int | None = None):
    stamp = ts_ns if stamp_ns is None else stamp_ns
    return {
        "topic": topic,
        "timestamp_ns": ts_ns,
        "data": build_tf_message_cdr(
            stamp // 1_000_000_000, stamp % 1_000_000_000, *IDENTITY,
            frame_id=parent, child_frame_id=child,
        ),
    }


def _imu(ts_ns: int, frame: str = "imu_link", stamp_ns: int | None = None, topic: str = "/imu"):
    stamp = ts_ns if stamp_ns is None else stamp_ns
    return {
        "topic": topic,
        "timestamp_ns": ts_ns,
        "data": build_imu_cdr(stamp // 1_000_000_000, stamp % 1_000_000_000, frame_id=frame),
    }


def _healthy_messages(n: int = 50) -> list[dict]:
    messages = [
        _tf(BASE_NS, "base_link", "imu_link", topic="/tf_static"),
    ]
    for i in range(n):
        ts = BASE_NS + i * 20_000_000
        messages.append(_tf(ts, "odom", "base_link"))
        messages.append(_tf(ts, "map", "odom"))
        messages.append(_imu(ts + 1_000_000))
    return messages


def _bag(tmp_path: Path, messages: list[dict], name: str = "tf.db3") -> str:
    topics = TF_TOPICS + [
        {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        {"name": "/imu2", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
    ]
    return str(_create_db3(tmp_path / name, topics, sorted(messages, key=lambda m: m["timestamp_ns"])))


def _ids(report) -> set[str]:
    return {finding.id for finding in report.findings}


def test_healthy_tree_has_no_findings(tmp_path: Path) -> None:
    report = analyze_tf(_bag(tmp_path, _healthy_messages()))
    assert report.findings == []
    assert report.roots == ["map"]
    assert report.frames == ["base_link", "imu_link", "map", "odom"]
    edge = next(e for e in report.edges if (e.parent, e.child) == ("odom", "base_link"))
    assert not edge.static
    assert edge.rate_hz == pytest.approx(50.0, rel=0.01)
    static = next(e for e in report.edges if e.child == "imu_link")
    assert static.static and static.rate_hz is None
    assert report.sensor_frames == {"/imu": "imu_link"}


def test_multiple_parents_is_an_error(tmp_path: Path) -> None:
    messages = _healthy_messages()
    messages.append(_tf(BASE_NS + 5_000_000, "camera_link", "imu_link", topic="/tf_static"))
    report = analyze_tf(_bag(tmp_path, messages))
    finding = next(f for f in report.findings if f.id == "tf.multiple_parents.imu_link")
    assert finding.severity == "error"
    assert finding.evidence[0].observed == ["base_link", "camera_link"]


def test_cycle_is_detected(tmp_path: Path) -> None:
    messages = _healthy_messages(10)
    messages.append(_tf(BASE_NS + 1, "base_link", "map", topic="/tf_static"))
    report = analyze_tf(_bag(tmp_path, messages))
    cycle = next(f for f in report.findings if f.id == "tf.cycle")
    assert cycle.severity == "error"
    assert set(cycle.evidence[0].observed) >= {"map", "odom", "base_link"}


def test_disconnected_trees_warn(tmp_path: Path) -> None:
    messages = [m for m in _healthy_messages() if b"map" not in m["data"]]
    messages.append(_tf(BASE_NS, "world", "map", topic="/tf_static"))
    report = analyze_tf(_bag(tmp_path, messages))
    assert "tf.disconnected" in _ids(report)
    assert report.roots == ["odom", "world"]


def test_static_dynamic_conflict(tmp_path: Path) -> None:
    messages = _healthy_messages()
    messages.append(_tf(BASE_NS, "odom", "base_link", topic="/tf_static"))
    report = analyze_tf(_bag(tmp_path, messages))
    assert "tf.static_dynamic_conflict.odom.base_link" in _ids(report)


def test_gap_on_one_publisher_is_one_finding(tmp_path: Path) -> None:
    messages = [
        m for m in _healthy_messages(100)
        if not (m["topic"] == "/tf"
                and BASE_NS + 400_000_000 < m["timestamp_ns"] < BASE_NS + 1_200_000_000)
    ]
    report = analyze_tf(_bag(tmp_path, messages))
    gaps = [f for f in report.findings if f.id.startswith("tf.edge_gap")]
    assert len(gaps) == 1
    assert len(gaps[0].evidence[1].observed) == 2  # map->odom and odom->base_link
    assert gaps[0].time_range is not None
    assert gaps[0].time_range.duration_sec == pytest.approx(0.80, abs=0.001)


def test_edge_that_stops_early(tmp_path: Path) -> None:
    messages = _healthy_messages(50)  # TF for the first second only
    messages += [_imu(BASE_NS + 1_000_000_000 + i * 20_000_000) for i in range(200)]
    report = analyze_tf(_bag(tmp_path, messages))
    stopped = [f for f in report.findings if f.id.startswith("tf.edge_stopped")]
    assert len(stopped) == 1
    assert stopped[0].time_range.start_ns == BASE_NS + 980_000_000
    assert stopped[0].time_range.end_ns == BASE_NS + 4_980_000_000


def test_zero_stamp_and_regression(tmp_path: Path) -> None:
    messages = _healthy_messages(20)
    messages.append(_tf(BASE_NS + 1_000, "odom", "wheel", stamp_ns=0))
    messages.append(_tf(BASE_NS + 2_000, "odom", "wheel", stamp_ns=BASE_NS + 10_000_000))
    messages.append(_tf(BASE_NS + 3_000, "odom", "wheel", stamp_ns=BASE_NS + 5_000_000))
    report = analyze_tf(_bag(tmp_path, messages))
    ids = _ids(report)
    assert "tf.zero_stamp.odom.wheel" in ids
    assert "tf.stamp_regression.odom.wheel" in ids


def test_sensor_frame_missing_from_tree(tmp_path: Path) -> None:
    messages = [m for m in _healthy_messages() if m["topic"] != "/imu"]
    messages += [_imu(BASE_NS + i * 20_000_000, frame="lidar_imu") for i in range(10)]
    report = analyze_tf(_bag(tmp_path, messages))
    finding = next(f for f in report.findings if f.id == "tf.frame_missing.lidar_imu")
    assert finding.affected_topics == ["/imu"]


def test_sensor_on_a_different_clock(tmp_path: Path) -> None:
    messages = [m for m in _healthy_messages() if m["topic"] != "/imu"]
    # /imu is stamped with time since boot; /imu2 and TF use system time.
    messages += [
        _imu(BASE_NS + i * 20_000_000, stamp_ns=1_000_000_000 + i * 20_000_000) for i in range(50)
    ]
    messages += [_imu(BASE_NS + i * 20_000_000, topic="/imu2") for i in range(50)]
    report = analyze_tf(_bag(tmp_path, messages))
    finding = next(f for f in report.findings if f.id == "tf.clock_mismatch.imu")
    assert finding.evidence[0].observed < -1e9


def test_tied_clock_groups_are_reported_once(tmp_path: Path) -> None:
    messages = [m for m in _healthy_messages() if m["topic"] != "/imu"]
    messages += [_imu(BASE_NS + i * 20_000_000, stamp_ns=5_000_000_000) for i in range(50)]
    report = analyze_tf(_bag(tmp_path, messages))
    ids = _ids(report)
    assert "tf.clock_split" in ids
    assert not any(i.startswith("tf.clock_mismatch") for i in ids)


def test_required_frames(tmp_path: Path) -> None:
    messages = [m for m in _healthy_messages() if b"map" not in m["data"]]
    messages.append(_tf(BASE_NS, "world", "map", topic="/tf_static"))
    report = analyze_tf(_bag(tmp_path, messages), required_frames=["map", "odom", "base_link", "gps"])
    ids = _ids(report)
    assert "tf.required_chain_broken.map.odom" in ids
    assert "tf.required_chain_broken.odom.base_link" not in ids
    assert "tf.required_frame_missing.gps" in ids


def test_no_tf_with_several_sensor_frames_is_info() -> None:
    acc = TfAccumulator({"/imu": "sensor_msgs/msg/Imu", "/gps": "sensor_msgs/msg/NavSatFix"})
    acc.add(Message("/imu", BASE_NS, {"frame_id": "imu_link", "stamp_sec": BASE_SEC}))
    acc.add(Message("/gps", BASE_NS, {"frame_id": "gps", "stamp_sec": BASE_SEC}))
    report = acc.build("mem")
    assert [(f.id, f.severity) for f in report.findings] == [("tf.not_recorded", "info")]


def test_decoded_ros1_style_messages() -> None:
    """Decoded (mcap / rosbags) dicts nest the stamp and may use '/frame' names."""
    acc = TfAccumulator({"/tf": "tf/tfMessage"})
    for i in range(5):
        acc.add(Message("/tf", BASE_NS + i * 10_000_000, {"transforms": [{
            "header": {"stamp": {"sec": BASE_SEC, "nanosec": i * 10_000_000},
                       "frame_id": "/odom"},
            "child_frame_id": "/base_link",
        }]}))
    report = acc.build("mem")
    assert report.frames == ["base_link", "odom"]
    assert report.findings == []


def test_eval_includes_tf_findings_without_changing_score(tmp_path: Path) -> None:
    healthy = evaluate_bag(_bag(tmp_path, _healthy_messages(), "a.db3"))
    messages = _healthy_messages()
    messages.append(_tf(BASE_NS + 5_000_000, "camera_link", "imu_link", topic="/tf_static"))
    broken = evaluate_bag(_bag(tmp_path, messages, "b.db3"))

    assert broken.overall_score == pytest.approx(healthy.overall_score)
    assert "tf.multiple_parents.imu_link" in {f.id for f in broken.findings}
    data = broken.to_dict()
    assert data["tf"]["roots"] == ["camera_link", "map"]
    assert any("TF: TF frame 'imu_link' has 2 parents" in r for r in data["recommendations"])


def test_eval_without_tf_has_no_tf_section(gnss_bag: Path) -> None:
    report = evaluate_bag(str(gnss_bag))
    assert report.tf is None
    assert not any(f.category == "tf_tree" for f in report.findings)


def test_cli_tf_json_and_fail_on(tmp_path: Path) -> None:
    messages = _healthy_messages()
    messages.append(_tf(BASE_NS + 5_000_000, "camera_link", "imu_link", topic="/tf_static"))
    bag = _bag(tmp_path, messages)
    out = tmp_path / "tf.json"
    runner = CliRunner()

    result = runner.invoke(app, ["tf", bag, "--json", str(out)])
    assert result.exit_code == 0, result.output
    data = json.loads(out.read_text())
    assert any(f["id"] == "tf.multiple_parents.imu_link" for f in data["findings"])

    result = runner.invoke(app, ["tf", bag, "--fail-on", "error"])
    assert result.exit_code == 1

    healthy = _bag(tmp_path, _healthy_messages(), "ok.db3")
    result = runner.invoke(app, ["tf", healthy, "--require-frames", "map,odom,base_link",
                                 "--fail-on", "warning"])
    assert result.exit_code == 0, result.output


def test_cli_tf_rejects_unknown_severity(tmp_path: Path) -> None:
    result = CliRunner().invoke(app, ["tf", _bag(tmp_path, _healthy_messages()), "--fail-on", "bad"])
    assert result.exit_code == 1


def test_laser_scan_frame_is_checked_against_tree(tmp_path: Path) -> None:
    """LaserScan frame_ids come from the CDR fallback parser, so Nav2 scans are covered."""
    from tests.conftest import build_laser_scan_cdr

    topics = TF_TOPICS + [{"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "format": "cdr"}]
    messages = [m for m in _healthy_messages() if m["topic"] != "/imu"]
    messages += [
        {"topic": "/scan", "timestamp_ns": BASE_NS + i * 100_000_000,
         "data": build_laser_scan_cdr(BASE_SEC, i * 100_000_000, [1.0] * 10, frame_id="base_scan")}
        for i in range(10)
    ]
    bag = _create_db3(tmp_path / "scan.db3", topics, sorted(messages, key=lambda m: m["timestamp_ns"]))
    report = analyze_tf(str(bag))
    assert report.sensor_frames["/scan"] == "base_scan"
    assert "tf.frame_missing.base_scan" in _ids(report)
