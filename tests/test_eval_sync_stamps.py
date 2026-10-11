"""Header diagnostics must catch period aliasing without changing readiness scores."""

from pathlib import Path

import jsonschema
import pytest
from typer.testing import CliRunner

from bagx.cli import app
from bagx.contracts import findings_schema
from bagx.eval import EvalReport, SyncMetrics, evaluate_bag
from bagx.eval_sync_stamps import generate_stamp_sync_findings
from tests.conftest import _create_db3, build_imu_cdr, build_laser_scan_cdr

BASE = 1_700_000_000_000_000_000


def _report(offset=0, common_offset=0):
    report = EvalReport("synthetic.db3", 3.0, 630, 2,
                        sync=SyncMetrics(topic_pairs=[("/imu", "/lidar")]))
    report._topic_timestamps = {
        "/imu": [BASE + i * 5_000_000 for i in range(600)],
        "/lidar": [BASE + i * 100_000_000 + 1_000_000 for i in range(30)],
    }
    report._topic_stamps = {
        topic: [(t, t + common_offset + (offset if topic == "/lidar" else 0)) for t in records]
        for topic, records in report._topic_timestamps.items()
    }
    return report


def _evidence(finding, metric):
    return next(e.observed for e in finding.evidence if e.metric == metric)


@pytest.mark.parametrize("offset", [100_000_000, -100_000_000])
def test_period_aliasing_is_reported_even_with_close_header_neighbors(offset):
    finding, = generate_stamp_sync_findings(_report(offset))
    assert finding.id == "sync.header_stamp.imu.lidar.offset"
    assert finding.severity == "warning"
    assert _evidence(finding, "relative_stamp_record_offset_ms") == offset / 1e6
    # A naive header-neighbor test would incorrectly declare this healthy.
    assert _evidence(finding, "header_nearest_neighbor_mean_ms") == pytest.approx(1)
    assert "transport/recording latency" in finding.suggested_action
    jsonschema.validate(finding.to_dict(), findings_schema())


@pytest.mark.parametrize("common_offset", [-100_000_000_000, -BASE + 10_000_000_000])
def test_common_clock_offset_and_nanosecond_precision(common_offset):
    finding, = generate_stamp_sync_findings(_report(123, common_offset=common_offset))
    assert finding.severity == "info"
    assert _evidence(finding, "relative_stamp_record_offset_ms") == pytest.approx(0.000123)
    assert _evidence(finding, "header_nearest_neighbor_mean_ms") == pytest.approx(1.000123)


def test_header_delay_can_warn_without_a_stamp_record_offset():
    report = _report()
    report._topic_timestamps = {
        "/imu": [BASE + i * 100_000_000 for i in range(30)],
        "/lidar": [BASE + i * 100_000_000 + 40_000_000 for i in range(30)],
    }
    report._topic_stamps = {t: [(r, r) for r in records]
                           for t, records in report._topic_timestamps.items()}
    finding, = generate_stamp_sync_findings(report)
    assert finding.id.endswith(".delay_high")
    assert finding.severity == "warning"
    assert _evidence(finding, "relative_stamp_record_offset_ms") == 0
    assert _evidence(finding, "header_nearest_neighbor_mean_ms") == 40


def test_regression_at_equal_record_times_is_not_sorted_away():
    report = _report()
    record, stamp = report._topic_stamps["/lidar"][10]
    report._topic_stamps["/lidar"].insert(11, (record, stamp - 1))
    report._topic_timestamps["/lidar"].insert(11, record)
    finding, = generate_stamp_sync_findings(report)
    assert finding.id.endswith(".unavailable")
    assert "regressing stamps" in finding.title


def test_reversing_topics_reverses_offset_sign_only():
    report = _report(100_000_000)
    forward, = generate_stamp_sync_findings(report)
    report.sync.topic_pairs = [("/lidar", "/imu")]
    reverse, = generate_stamp_sync_findings(report)
    assert _evidence(reverse, "relative_stamp_record_offset_ms") == -100
    assert _evidence(forward, "header_nearest_neighbor_mean_ms") == (
        _evidence(reverse, "header_nearest_neighbor_mean_ms")
    )


def test_clock_mismatch_does_not_compare_incompatible_header_times():
    finding, = generate_stamp_sync_findings(_report(-BASE + 10_000_000_000))
    assert finding.id.endswith(".clock_mismatch")
    assert finding.severity == "warning"
    assert not any(e.metric.startswith("header_nearest_neighbor") for e in finding.evidence)


@pytest.mark.parametrize("case", ["missing", "zero", "negative", "partial", "regression", "repeated"])
def test_unusable_stamps_do_not_produce_a_healthy_result(case):
    report = _report()
    samples = report._topic_stamps["/lidar"]
    if case == "missing":
        report._topic_stamps.pop("/lidar")
    elif case in {"zero", "negative"}:
        report._topic_stamps["/lidar"] = [(r, 0 if case == "zero" else -1) for r, _ in samples]
    elif case == "partial":
        report._topic_stamps["/lidar"] = samples[:20]
    elif case == "regression":
        samples[10] = (samples[10][0], samples[9][1] - 1)
    else:
        samples[10] = (samples[10][0], samples[9][1])
    finding, = generate_stamp_sync_findings(report)
    assert finding.id.endswith(".unavailable")
    assert finding.severity == "info"
    assert not any(e.metric.startswith("header_nearest_neighbor") for e in finding.evidence)


def test_offset_estimates_use_only_the_common_recording_window():
    report = _report(30_000_000)
    # An early clock offset outside the shared window must not bias the median.
    early = [(BASE - i * 5_000_000, BASE - i * 5_000_000 + 500_000_000)
             for i in range(1, 1000)]
    report._topic_stamps["/imu"] = early + report._topic_stamps["/imu"]
    report._topic_timestamps["/imu"] += [r for r, _ in early]
    finding, = generate_stamp_sync_findings(report)
    assert _evidence(finding, "relative_stamp_record_offset_ms") == 30


def test_short_header_overlap_reports_offset_without_boundary_delay():
    report = _report(900_000_000)
    report._topic_timestamps = {t: ts[:200] if t == "/imu" else ts[:10]
                                for t, ts in report._topic_timestamps.items()}
    report._topic_stamps = {t: samples[:len(report._topic_timestamps[t])]
                           for t, samples in report._topic_stamps.items()}
    finding, = generate_stamp_sync_findings(report)
    assert finding.id.endswith(".offset")
    assert "insufficient header-time overlap" in finding.title
    assert not any(e.metric.startswith("header_nearest_neighbor") for e in finding.evidence)


def _bag(path: Path, offset: int) -> Path:
    report = _report(offset)
    messages = []
    for topic, samples in report._topic_stamps.items():
        builder = build_imu_cdr if topic == "/imu" else build_laser_scan_cdr
        for record, stamp in samples:
            args = (stamp // 1_000_000_000, stamp % 1_000_000_000)
            data = builder(*args) if topic == "/imu" else builder(*args, [1.0] * 90)
            messages.append({"topic": topic, "timestamp_ns": record, "data": data})
    topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu"},
              {"name": "/lidar", "type": "sensor_msgs/msg/LaserScan"}]
    return _create_db3(path, topics, sorted(messages, key=lambda m: m["timestamp_ns"]))


def test_eval_and_cli_expose_offset_without_changing_scores(tmp_path):
    baseline = evaluate_bag(str(_bag(tmp_path / "baseline.db3", 0))).to_dict()
    bag = _bag(tmp_path / "offset.db3", 100_000_000)
    changed = evaluate_bag(str(bag)).to_dict()
    assert changed["sync"] == baseline["sync"]
    assert changed["overall_score"] == baseline["overall_score"]
    assert changed["bagx_version"] == baseline["bagx_version"] == "0.6.0"
    assert changed["schema_version"] == baseline["schema_version"]
    finding = next(f for f in changed["findings"] if f["id"].endswith(".lidar.offset"))
    jsonschema.validate(finding, findings_schema())
    assert any("Header sync:" in r and "+100.0ms" in r for r in changed["recommendations"])
    result = CliRunner().invoke(app, ["eval", str(bag), "--findings-only"])
    assert result.exit_code == 0, result.output
    assert "relative_stamp_record_offset_ms" in result.output
    assert "100.0" in result.output
