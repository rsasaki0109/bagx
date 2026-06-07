"""Tests for bagx.badge readiness badge generation."""

from pathlib import Path

import pytest

from bagx.badge import DEFAULT_LABEL, eval_badge, score_color
from bagx.eval import EvalReport, evaluate_bag


class TestScoreColor:
    @pytest.mark.parametrize(
        ("score", "color"),
        [
            (100.0, "brightgreen"),
            (85.0, "brightgreen"),
            (84.9, "green"),
            (70.0, "green"),
            (60.0, "yellow"),
            (50.0, "yellow"),
            (40.0, "orange"),
            (30.0, "orange"),
            (29.9, "red"),
            (0.0, "red"),
        ],
    )
    def test_thresholds(self, score: float, color: str):
        assert score_color(score) == color


class TestEvalBadge:
    def test_endpoint_schema(self):
        report = EvalReport(
            bag_path="x.db3", duration_sec=1.0, total_messages=1, topic_count=1
        )
        report.overall_score = 76.6
        badge = eval_badge(report)
        assert badge["schemaVersion"] == 1
        assert badge["message"] == "76.6/100"
        assert badge["color"] == "green"
        # No detectable domain → default label.
        assert badge["label"] == DEFAULT_LABEL

    def test_explicit_label_wins(self):
        report = EvalReport(
            bag_path="x.db3", duration_sec=1.0, total_messages=1, topic_count=1
        )
        badge = eval_badge(report, label="my robot")
        assert badge["label"] == "my robot"

    def test_label_includes_detected_domain(self, nav2_bag: Path):
        report = evaluate_bag(str(nav2_bag))
        badge = eval_badge(report)
        assert "Nav2" in badge["label"]
        assert badge["label"].endswith("readiness")

    def test_from_real_eval(self, gnss_bag: Path):
        report = evaluate_bag(str(gnss_bag))
        badge = eval_badge(report)
        assert badge["message"].endswith("/100")
        assert badge["color"] in {"brightgreen", "green", "yellow", "orange", "red"}
