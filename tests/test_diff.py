"""Tests for bagx diff."""

from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
from typer.testing import CliRunner

from bagx.cli import app
from bagx.diff import (
    FindingsDiff,
    compare_findings,
    print_diff_markdown,
    print_diff_text,
    run_diff,
)

runner = CliRunner()


def _make_finding(
    fid: str,
    severity: str,
    *,
    category: str = "sensor_quality",
    domain: str | None = "slam",
    title: str | None = None,
    evidence: list[dict] | None = None,
) -> dict:
    return {
        "id": fid,
        "title": title or f"finding {fid}",
        "severity": severity,
        "category": category,
        "domain": domain,
        "affected_topics": [],
        "evidence": evidence or [],
        "suggested_action": None,
        "confidence": "medium",
    }


class TestCompareFindings:
    def test_new_finding(self):
        diff = compare_findings(
            "/b.json", [],
            "/c.json", [_make_finding("imu.rate.low", "warning")],
        )
        assert len(diff.changes) == 1
        change = diff.changes[0]
        assert change.kind == "new"
        assert change.current_severity == "warning"
        assert change.baseline_severity is None

    def test_gone_finding(self):
        diff = compare_findings(
            "/b.json", [_make_finding("imu.rate.low", "warning")],
            "/c.json", [],
        )
        assert len(diff.changes) == 1
        assert diff.changes[0].kind == "gone"
        assert diff.changes[0].baseline_severity == "warning"

    def test_worse_severity(self):
        diff = compare_findings(
            "/b.json", [_make_finding("sync.delay", "info")],
            "/c.json", [_make_finding("sync.delay", "warning")],
        )
        assert diff.changes[0].kind == "worse"
        assert diff.changes[0].baseline_severity == "info"
        assert diff.changes[0].current_severity == "warning"

    def test_better_severity(self):
        diff = compare_findings(
            "/b.json", [_make_finding("sync.delay", "error")],
            "/c.json", [_make_finding("sync.delay", "warning")],
        )
        assert diff.changes[0].kind == "better"

    def test_same_excluded_by_default(self):
        f = _make_finding("nav2.detected", "info")
        diff = compare_findings("/b.json", [f], "/c.json", [f])
        assert diff.changes == []

    def test_same_included_when_requested(self):
        f = _make_finding("nav2.detected", "info")
        diff = compare_findings(
            "/b.json", [f], "/c.json", [f], include_same=True
        )
        assert len(diff.changes) == 1
        assert diff.changes[0].kind == "same"

    def test_evidence_drift_numeric(self):
        b = _make_finding(
            "gnss.fix_rate.good", "info",
            evidence=[{"metric": "fix_rate", "observed": 0.95, "topic": None}],
        )
        c = _make_finding(
            "gnss.fix_rate.good", "info",
            evidence=[{"metric": "fix_rate", "observed": 0.50, "topic": None}],
        )
        diff = compare_findings("/b.json", [b], "/c.json", [c], include_same=True)
        assert diff.changes[0].kind == "same"
        assert len(diff.changes[0].evidence_drift) == 1
        drift = diff.changes[0].evidence_drift[0]
        assert drift.metric == "fix_rate"
        assert drift.baseline_observed == 0.95
        assert drift.current_observed == 0.50
        assert drift.drift_ratio is not None and drift.drift_ratio > 0.4

    def test_evidence_drift_ignored_when_small(self):
        b = _make_finding(
            "gnss.fix_rate.good", "info",
            evidence=[{"metric": "fix_rate", "observed": 0.95, "topic": None}],
        )
        c = _make_finding(
            "gnss.fix_rate.good", "info",
            evidence=[{"metric": "fix_rate", "observed": 0.96, "topic": None}],
        )
        diff = compare_findings("/b.json", [b], "/c.json", [c], include_same=True)
        assert diff.changes[0].evidence_drift == []

    def test_sort_order_worse_first(self):
        diff = compare_findings(
            "/b.json", [_make_finding("a", "info"), _make_finding("c", "error")],
            "/c.json", [_make_finding("a", "warning"), _make_finding("b", "warning"), _make_finding("c", "info")],
        )
        kinds = [c.kind for c in diff.changes]
        # Order: worse > new > gone > better > same
        assert kinds == ["worse", "new", "better"]

    def test_has_regression_warning_threshold(self):
        diff = compare_findings(
            "/b.json", [],
            "/c.json", [_make_finding("imu.rate.low", "warning")],
        )
        assert diff.has_regression("warning") is True
        assert diff.has_regression("error") is False

    def test_has_regression_only_on_worse_or_new(self):
        # better+gone should not trigger regression
        diff = compare_findings(
            "/b.json", [_make_finding("x", "error")],
            "/c.json", [_make_finding("x", "info")],
        )
        assert diff.has_regression("warning") is False


class TestDiffRendering:
    def test_text_rendering_runs(self, capsys: pytest.CaptureFixture):
        diff = compare_findings(
            "/b.json", [_make_finding("x", "info")],
            "/c.json", [_make_finding("x", "warning")],
        )
        from rich.console import Console
        console = Console(file=io.StringIO(), force_terminal=False, no_color=True)
        print_diff_text(diff, console)
        output = console.file.getvalue()
        assert "WORSE" in output
        assert "info" in output and "warning" in output

    def test_markdown_rendering(self):
        diff = compare_findings(
            "/b.json", [_make_finding("x", "info")],
            "/c.json", [_make_finding("x", "warning")],
        )
        buf = io.StringIO()
        print_diff_markdown(diff, buf)
        md = buf.getvalue()
        assert "bagx findings diff" in md
        assert "| worse |" in md
        assert "`x`" in md
        assert "->" in md  # plain ASCII arrow in markdown


class TestRunDiff:
    def test_round_trip(self, tmp_path: Path):
        baseline_path = tmp_path / "baseline.json"
        current_path = tmp_path / "current.json"
        baseline_path.write_text(json.dumps({"findings": [_make_finding("a", "info")]}))
        current_path.write_text(json.dumps({"findings": [_make_finding("a", "warning")]}))

        diff = run_diff(baseline_path, current_path)
        assert isinstance(diff, FindingsDiff)
        assert len(diff.changes) == 1
        assert diff.changes[0].kind == "worse"


class TestDiffCli:
    def _write_eval(self, path: Path, findings: list[dict]) -> None:
        path.write_text(json.dumps({"findings": findings}, indent=2))

    def test_diff_cli_text(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        self._write_eval(b, [_make_finding("x", "info")])
        self._write_eval(c, [_make_finding("x", "warning"), _make_finding("y", "error")])

        result = runner.invoke(app, ["diff", str(b), str(c)])
        assert result.exit_code == 0
        assert "WORSE" in result.output
        assert "NEW" in result.output

    def test_diff_cli_json(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        out = tmp_path / "diff.json"
        self._write_eval(b, [_make_finding("x", "info")])
        self._write_eval(c, [_make_finding("x", "warning")])

        result = runner.invoke(
            app, ["diff", str(b), str(c), "--format", "json", "--output", str(out)]
        )
        assert result.exit_code == 0
        data = json.loads(out.read_text())
        assert data["summary"]["worse"] == 1
        assert data["report_type"] == "diff"

    def test_diff_cli_markdown(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        self._write_eval(b, [])
        self._write_eval(c, [_make_finding("new.id", "warning")])

        result = runner.invoke(app, ["diff", str(b), str(c), "--format", "markdown"])
        assert result.exit_code == 0
        assert "| new |" in result.output
        assert "`new.id`" in result.output

    def test_diff_cli_exit_on_warning_fails(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        self._write_eval(b, [])
        self._write_eval(c, [_make_finding("x", "warning")])

        result = runner.invoke(app, ["diff", str(b), str(c), "--exit-on", "warning"])
        assert result.exit_code == 1

    def test_diff_cli_exit_on_error_lets_warning_pass(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        self._write_eval(b, [])
        self._write_eval(c, [_make_finding("x", "warning")])

        result = runner.invoke(app, ["diff", str(b), str(c), "--exit-on", "error"])
        assert result.exit_code == 0

    def test_diff_cli_invalid_format(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        self._write_eval(b, [])
        self._write_eval(c, [])
        result = runner.invoke(app, ["diff", str(b), str(c), "--format", "yaml"])
        assert result.exit_code == 2

    def test_diff_cli_invalid_exit_on(self, tmp_path: Path):
        b = tmp_path / "b.json"
        c = tmp_path / "c.json"
        self._write_eval(b, [])
        self._write_eval(c, [])
        result = runner.invoke(app, ["diff", str(b), str(c), "--exit-on", "fatal"])
        assert result.exit_code == 2

    def test_diff_cli_missing_file(self, tmp_path: Path):
        result = runner.invoke(app, ["diff", "/nonexistent.json", "/also-nonexistent.json"])
        assert result.exit_code == 1


def _make_temporal_finding(
    fid: str,
    severity: str,
    start_ns: int,
    end_ns: int,
    *,
    title: str | None = None,
) -> dict:
    base = _make_finding(fid, severity, title=title)
    base["time_range"] = {"start_ns": start_ns, "end_ns": end_ns}
    return base


class TestSegmentAwareDiff:
    """Phase C: time_range overlap drives segment matching."""

    def test_overlapping_ranges_match_as_same_segment(self):
        # baseline 10-20s, current 12-22s → overlap, same segment
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 12_000_000_000, 22_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c], include_same=True)
        assert len(diff.changes) == 1
        assert diff.changes[0].kind == "same"
        assert diff.changes[0].baseline_time_range == {"start_ns": 10_000_000_000, "end_ns": 20_000_000_000}
        assert diff.changes[0].current_time_range == {"start_ns": 12_000_000_000, "end_ns": 22_000_000_000}

    def test_disjoint_ranges_become_separate_changes(self):
        # baseline at 10-20s, current at 50-60s → no overlap, both reported
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 50_000_000_000, 60_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        kinds = sorted(ch.kind for ch in diff.changes)
        assert kinds == ["gone", "new"]
        gone = next(ch for ch in diff.changes if ch.kind == "gone")
        new = next(ch for ch in diff.changes if ch.kind == "new")
        assert gone.baseline_time_range == {"start_ns": 10_000_000_000, "end_ns": 20_000_000_000}
        assert new.current_time_range == {"start_ns": 50_000_000_000, "end_ns": 60_000_000_000}

    def test_multiple_overlapping_segments_match_greedily(self):
        # Two baseline segments overlap two current segments respectively
        b1 = _make_temporal_finding("anomaly.imu.accel_spike.imu", "warning", 10_000_000_000, 15_000_000_000)
        b2 = _make_temporal_finding("anomaly.imu.accel_spike.imu", "warning", 100_000_000_000, 110_000_000_000)
        c1 = _make_temporal_finding("anomaly.imu.accel_spike.imu", "error", 12_000_000_000, 14_000_000_000)
        c2 = _make_temporal_finding("anomaly.imu.accel_spike.imu", "warning", 105_000_000_000, 115_000_000_000)
        diff = compare_findings("/b.json", [b1, b2], "/c.json", [c1, c2], include_same=True)
        # Two paired changes; the first should be "worse" (severity up), second "same"
        kinds = [ch.kind for ch in diff.changes]
        assert kinds.count("worse") == 1
        assert kinds.count("same") == 1
        worse = next(ch for ch in diff.changes if ch.kind == "worse")
        assert worse.baseline_time_range["start_ns"] == 10_000_000_000

    def test_temporal_severity_change_is_worse(self):
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "error", 11_000_000_000, 21_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        assert len(diff.changes) == 1
        assert diff.changes[0].kind == "worse"
        assert diff.changes[0].baseline_severity == "warning"
        assert diff.changes[0].current_severity == "error"

    def test_baseline_global_does_not_match_current_temporal(self):
        # Same id, baseline is bag-global, current is temporal → different things
        b = _make_finding("anomaly.gnss.fix_lost.gnss", "warning")
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        # The global one disappears, the temporal one appears
        kinds = sorted(ch.kind for ch in diff.changes)
        assert kinds == ["gone", "new"]

    def test_global_findings_still_join_by_id(self):
        """Pre-v0.4 (no time_range) findings continue to join by id alone."""
        b = _make_finding("nav2.detected", "info")
        c = _make_finding("nav2.detected", "info")
        diff = compare_findings("/b.json", [b], "/c.json", [c], include_same=True)
        assert len(diff.changes) == 1
        assert diff.changes[0].kind == "same"
        assert diff.changes[0].baseline_time_range is None
        assert diff.changes[0].current_time_range is None

    def test_to_dict_includes_time_range_fields(self):
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10, 20)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "error", 12, 22)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        d = diff.to_dict()
        change = d["changes"][0]
        assert change["baseline_time_range"] == {"start_ns": 10, "end_ns": 20}
        assert change["current_time_range"] == {"start_ns": 12, "end_ns": 22}

    def test_markdown_includes_segment_column_when_temporal(self):
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "error", 12_000_000_000, 22_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        stream = io.StringIO()
        print_diff_markdown(diff, stream)
        out = stream.getvalue()
        assert "segment" in out
        assert "t=12.0-22.0s" in out

    def test_text_rendering_shows_segment(self):
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "error", 12_000_000_000, 22_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        from rich.console import Console
        console = Console(file=io.StringIO(), force_terminal=False)
        print_diff_text(diff, console)
        out = console.file.getvalue()
        assert "segment" in out
        assert "t=10.0-20.0s" in out and "t=12.0-22.0s" in out

    def test_regression_detection_works_for_temporal_findings(self):
        b = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "warning", 10_000_000_000, 20_000_000_000)
        c = _make_temporal_finding("anomaly.gnss.fix_lost.gnss", "error", 12_000_000_000, 22_000_000_000)
        diff = compare_findings("/b.json", [b], "/c.json", [c])
        assert diff.has_regression("warning")
        assert diff.has_regression("error")
