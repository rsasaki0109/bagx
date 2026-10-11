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
    validate_manifest,
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


def _scored_case(name: str, **overrides) -> dict:
    case = {
        "name": name,
        "provenance": "public",
        "availability": "available",
        "results": {
            "overall_score": 100.0,
            "imu_score": None,
            "sync_score": None,
            "gnss_score": None,
            "domain_score": 100.0,
            "key_finding": "Pipeline joint_states -> planned_path: 6ms median",
            "bagx_version": "0.6.0",
            "evaluated_at": "2026-06-10",
        },
    }
    case.update(overrides)
    return case


def test_validate_manifest_flags_identical_results() -> None:
    manifest = {"cases": [_scored_case("public-row"), _scored_case("sim-row")]}
    problems = validate_manifest(manifest)
    assert any("sim-row: results identical to public-row" in p for p in problems)


def test_validate_manifest_flags_shared_fingerprint() -> None:
    first = _scored_case("a")
    second = _scored_case("b")
    second["results"]["overall_score"] = 90.0
    fingerprint = {"total_messages": 10, "topic_count": 2, "duration_sec": 1.0}
    first["results"]["fingerprint"] = dict(fingerprint)
    second["results"]["fingerprint"] = dict(fingerprint)
    problems = validate_manifest({"cases": [first, second]})
    assert any("b: scored on the same bag as a" in p for p in problems)


def test_validate_manifest_rejects_scores_on_unavailable_source() -> None:
    case = _scored_case("gone", availability="source_unavailable")
    problems = validate_manifest({"cases": [case]})
    assert any("source_unavailable case must not carry scores" in p for p in problems)


def test_validate_manifest_requires_provenance() -> None:
    case = _scored_case("x")
    del case["provenance"]
    problems = validate_manifest({"cases": [case]})
    assert any("x: provenance must be one of" in p for p in problems)


def test_repo_scoreboard_manifest_is_consistent() -> None:
    repo_manifest = Path(__file__).resolve().parents[1] / "benchmarks" / "scoreboard.json"
    with open(repo_manifest) as f:
        manifest = json.load(f)
    assert validate_manifest(manifest) == []


def test_refresh_uses_dogfood_fallback_only_for_simulator_rows(tmp_path: Path) -> None:
    manifest_path = tmp_path / "scoreboard.json"
    manifest = {
        "cases": [
            {
                "name": "public-row",
                "provenance": "public",
                "bag_path": str(tmp_path / "missing" / "nav2_capture"),
                "results": {},
            }
        ]
    }
    refreshed, missing = refresh_manifest_scores(manifest, manifest_path, repo_fallback=True)
    assert (refreshed, missing) == (0, 1)


def test_render_table_marks_simulator_and_unavailable_rows() -> None:
    manifest = {
        "cases": [
            _scored_case("sim", provenance="simulator", display_name="Sim"),
            {
                "name": "gone",
                "display_name": "Gone",
                "provenance": "public",
                "availability": "source_unavailable",
                "results": {"overall_score": None},
            },
        ]
    }
    table = render_scoreboard_table(manifest)
    assert "| Sim | " in table and "| sim |" in table
    assert "| unavailable |" in table
    assert "1 public · 1 simulator captures" in table
