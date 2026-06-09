"""Tests for scoreboard manifest and markdown generation."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.generate_scoreboard import (
    build_scoreboard_markdown,
    refresh_manifest_scores,
    render_scoreboard_table,
    update_scoreboard_file,
)


@pytest.fixture
def scoreboard_manifest(tmp_path: Path) -> Path:
    manifest = {
        "schema_version": "1.0",
        "suite_name": "test-scoreboard",
        "cases": [
            {
                "name": "sample-slam",
                "display_name": "Sample SLAM",
                "domain": "SLAM",
                "ros": "ros2",
                "source_url": "https://example.com/slam",
                "bag_path": "${BAGX_SCOREBOARD_BAGS}/missing.db3",
                "reproduce": "bagx eval missing.db3",
                "results": {
                    "overall_score": 88.0,
                    "imu_score": 90.0,
                    "sync_score": 85.0,
                    "gnss_score": None,
                    "domain_score": None,
                    "key_finding": "Good LiDAR+IMU pair",
                    "bagx_version": "0.6.0",
                    "evaluated_at": "2026-06-10",
                },
            }
        ],
    }
    path = tmp_path / "scoreboard.json"
    path.write_text(json.dumps(manifest, indent=2))
    return path


def test_render_scoreboard_table_links_and_scores(scoreboard_manifest: Path) -> None:
    with open(scoreboard_manifest) as f:
        manifest = json.load(f)
    table = render_scoreboard_table(manifest)
    assert "[Sample SLAM](https://example.com/slam)" in table
    assert "**88.0**" in table
    assert "Good LiDAR+IMU pair" in table


def test_build_scoreboard_markdown_contains_markers(scoreboard_manifest: Path) -> None:
    with open(scoreboard_manifest) as f:
        manifest = json.load(f)
    md = build_scoreboard_markdown(manifest)
    assert "<!-- SCOREBOARD_TABLE_START -->" in md
    assert "why-ouster-os0-32-scores-77" in md


def test_update_scoreboard_file_preserves_outer_sections(
    scoreboard_manifest: Path, tmp_path: Path
) -> None:
    with open(scoreboard_manifest) as f:
        manifest = json.load(f)
    output = tmp_path / "scoreboard.md"
    update_scoreboard_file(manifest, output)
    manifest["cases"][0]["results"]["overall_score"] = 91.0
    update_scoreboard_file(manifest, output)
    second = output.read_text()
    assert "**91.0**" in second
    assert "Analysis articles" in second


def test_refresh_skips_missing_bags(scoreboard_manifest: Path) -> None:
    with open(scoreboard_manifest) as f:
        manifest = json.load(f)
    refreshed, missing = refresh_manifest_scores(manifest, scoreboard_manifest, repo_fallback=False)
    assert refreshed == 0
    assert missing == 1


def test_scoreboard_manifest_has_thirty_cases() -> None:
    repo_manifest = Path(__file__).resolve().parents[1] / "benchmarks" / "scoreboard.json"
    with open(repo_manifest) as f:
        manifest = json.load(f)
    assert len(manifest["cases"]) == 30
