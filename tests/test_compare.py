"""Tests for bagx.compare module."""

import json
from pathlib import Path


from bagx.compare import compare_bags


class TestCompare:
    def test_compare_good_vs_degraded(self, gnss_bag: Path, gnss_bag_degraded: Path):
        report = compare_bags(str(gnss_bag), str(gnss_bag_degraded))

        # gnss_bag (A) is better: 90% fix vs 50% fix
        assert report.winner == "A"
        assert len(report.items) > 0

    def test_compare_degraded_vs_good(self, gnss_bag_degraded: Path, gnss_bag: Path):
        report = compare_bags(str(gnss_bag_degraded), str(gnss_bag))

        # B (gnss_bag) is better
        assert report.winner == "B"

    def test_compare_same_bag(self, gnss_bag: Path):
        report = compare_bags(str(gnss_bag), str(gnss_bag))

        # Should be a tie
        assert report.winner == "tie"
        for item in report.items:
            assert item.verdict == "unchanged"

    def test_comparison_items(self, gnss_bag: Path, gnss_bag_degraded: Path):
        report = compare_bags(str(gnss_bag), str(gnss_bag_degraded))

        metric_names = [i.metric for i in report.items]
        assert "Overall Score" in metric_names
        assert "GNSS Fix Rate" in metric_names
        assert "GNSS Score" in metric_names

    def test_compare_verdicts(self, gnss_bag: Path, gnss_bag_degraded: Path):
        report = compare_bags(str(gnss_bag), str(gnss_bag_degraded))

        verdicts = {i.metric: i.verdict for i in report.items}
        # Fix rate should degrade going from good to bad
        assert verdicts["GNSS Fix Rate"] == "degraded"

    def test_json_output(self, gnss_bag: Path, gnss_bag_degraded: Path, tmp_path: Path):
        json_path = tmp_path / "compare.json"
        with open(json_path, "w") as f:
            compare_bags(str(gnss_bag), str(gnss_bag_degraded), output_json=f)

        with open(json_path) as f:
            data = json.load(f)

        assert "winner" in data
        assert "items" in data
        assert len(data["items"]) > 0

    def test_to_dict(self, gnss_bag: Path, gnss_bag_degraded: Path):
        report = compare_bags(str(gnss_bag), str(gnss_bag_degraded))
        d = report.to_dict()
        assert d["winner"] in ("A", "B", "tie")
        assert isinstance(d["items"], list)
