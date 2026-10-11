"""Opt-in IMU pairing must include sensors beyond the legacy five pairs."""

import numpy as np
import pytest
from typer.testing import CliRunner

from bagx.cli import app
from bagx.eval import EvalConfig, _evaluate_sync, evaluate_bag


def _streams():
    # Name order deliberately puts IMU after both cameras and many LiDARs.
    names = ["/camera/left", "/camera/right", *[f"/cloud/{i}" for i in range(7)], "/imu"]
    return {name: np.arange(0, 2_000_000_000, 5_000_000 if name == "/imu" else 100_000_000,
                            dtype=np.int64).tolist() for name in names}


def test_primary_imu_pairs_with_every_other_sensor():
    streams = _streams()
    result = _evaluate_sync(streams, list(streams), EvalConfig(sync_pairing="imu"),
                            primary_imu="/imu")
    assert set(result.topic_pairs) == {("/imu", t) for t in streams if t != "/imu"}
    assert len(result.topic_pairs) == 9
    assert result.score == 100


def test_legacy_pairing_remains_the_default():
    streams = _streams()
    result = _evaluate_sync(streams, list(streams), EvalConfig())
    names = sorted(streams)
    assert result.topic_pairs == list(zip(names[:-1], names[1:]))[:5]


def test_no_active_primary_imu_uses_all_adjacent_pairs():
    streams = _streams()
    result = _evaluate_sync(streams, list(streams), EvalConfig(sync_pairing="imu"),
                            primary_imu="/missing")
    names = sorted(streams)
    assert result.topic_pairs == list(zip(names[:-1], names[1:]))


def test_pair_selection_is_stable_when_input_order_changes():
    streams = _streams()
    cfg = EvalConfig(sync_pairing="imu")
    forward = _evaluate_sync(streams, list(streams), cfg, primary_imu="/imu")
    reverse = _evaluate_sync(streams, list(reversed(streams)), cfg, primary_imu="/imu")
    assert forward == reverse


def test_opt_in_changes_the_score_when_sensor_pairs_differ():
    # Two 10 Hz cameras alternate by 50 ms; both coincide with a 200 Hz IMU.
    # This documents mode-dependent scoring, not physical synchronization.
    streams = {
        "/camera/a": list(range(0, 2_000_000_000, 100_000_000)),
        "/camera/b": list(range(50_000_000, 2_000_000_000, 100_000_000)),
        "/imu": list(range(0, 2_000_000_000, 5_000_000)),
    }
    legacy = _evaluate_sync(streams, list(streams), EvalConfig(), primary_imu="/imu")
    imu = _evaluate_sync(streams, list(streams), EvalConfig(sync_pairing="imu"),
                         primary_imu="/imu")
    assert legacy.mean_delay_ms == [50, 0]
    assert legacy.score == pytest.approx(70.96153846153847)
    assert imu.mean_delay_ms == [0, 0]
    assert imu.score == 100


def test_pairs_without_shared_recording_time_are_omitted():
    streams = _streams()
    streams["/camera/right"] = list(range(3_000_000_000, 4_000_000_000, 100_000_000))
    result = _evaluate_sync(streams, list(streams), EvalConfig(sync_pairing="imu"),
                            primary_imu="/imu")
    assert ("/imu", "/camera/right") not in result.topic_pairs
    assert len(result.topic_pairs) == 8


def test_eval_selects_the_evaluated_primary_imu(multi_bag):
    report = evaluate_bag(str(multi_bag), config=EvalConfig(sync_pairing="imu"))
    assert report.imu_topic == "/imu"
    assert set(report.sync.topic_pairs) == {("/imu", "/gnss"), ("/imu", "/lidar")}
    assert any(f.id.startswith("sync.header_stamp.") for f in report.findings)


def test_invalid_mode_is_rejected_before_reading_a_bag():
    with pytest.raises(ValueError, match="sync_pairing"):
        evaluate_bag("/no/bag", config=EvalConfig(sync_pairing="bad"))
    result = CliRunner().invoke(app, ["eval", "/no/bag", "--sync-pairing", "bad"])
    assert result.exit_code == 2
    assert "--sync-pairing must be adjacent or imu" in result.output


def test_cli_accepts_imu_mode(multi_bag, tmp_path):
    import json

    output = tmp_path / "eval.json"
    result = CliRunner().invoke(app, ["eval", str(multi_bag), "--sync-pairing", "imu",
                                     "--json", str(output)])
    assert result.exit_code == 0, result.output
    report = json.loads(output.read_text())
    assert report["sync"]["topic_pairs"] == [["/imu", "/gnss"], ["/imu", "/lidar"]]
