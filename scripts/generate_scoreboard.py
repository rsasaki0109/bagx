#!/usr/bin/env python3
"""Generate docs/scoreboard.md from benchmarks/scoreboard.json.

Usage:
    python scripts/generate_scoreboard.py              # render markdown (cached scores)
    python scripts/generate_scoreboard.py --refresh    # re-eval bags that exist locally
    python scripts/generate_scoreboard.py --write-manifest  # save refreshed scores to JSON
    python scripts/generate_scoreboard.py --check      # validate manifest integrity only
"""

from __future__ import annotations

import argparse
import json
import os
import re
from datetime import date
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = REPO_ROOT / "benchmarks" / "scoreboard.json"
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "scoreboard.md"
MARKER_START = "<!-- SCOREBOARD_TABLE_START -->"
MARKER_END = "<!-- SCOREBOARD_TABLE_END -->"

# Where a case's bag comes from. Only "simulator" cases may be resolved from the
# local dogfood capture cache; a public row must be scored on its upstream bag.
PROVENANCES = ("public", "simulator")
# "source_unavailable": upstream bag can no longer be downloaded; the row is kept
# for tracking but must not carry scores.
AVAILABILITIES = ("available", "source_unavailable")
_SCORE_KEYS = ("overall_score", "imu_score", "sync_score", "gnss_score", "domain_score")


def _resolve_manifest_path(raw_path: str, manifest_dir: Path) -> Path:
    expanded_env = Path(os.path.expandvars(raw_path)).expanduser()
    if expanded_env.is_absolute():
        return expanded_env
    return (manifest_dir / expanded_env).resolve()


def _format_score(value: float | None) -> str:
    if value is None:
        return "—"
    return f"{value:.1f}"


def _pick_key_finding(report_dict: dict[str, Any], domains: list[str]) -> str:
    recommendations = report_dict.get("recommendations", [])
    skip_prefixes = (
        "OK IMU",
        "INFO No GNSS",
        "INFO Short recording",
        "No GNSS data",
    )

    def _clean_rec(text: str) -> str:
        text = re.sub(r"\[[^\]]*\]", "", text).strip()
        return re.sub(r"^OK ", "", text)

    for item in recommendations:
        text = _clean_rec(str(item))
        if not text or text.endswith("topics detected"):
            continue
        if any(text.startswith(prefix) for prefix in skip_prefixes):
            continue
        if "Pipeline" in text or "plan →" in text or "cmd_vel" in text:
            return text
    for item in recommendations:
        text = _clean_rec(str(item))
        if text and not text.endswith("topics detected"):
            if text.startswith("INFO "):
                continue
            if not any(text.startswith(prefix) for prefix in skip_prefixes):
                return text
    findings = report_dict.get("findings", [])
    for finding in findings:
        message = str(finding.get("message", "")).strip()
        if message:
            return message
    if recommendations:
        return re.sub(r"\[[^\]]*\]", "", str(recommendations[0])).strip()
    return f"Evaluated — domains: {', '.join(domains) or 'general'}"


def _fingerprint_from_report(report_dict: dict[str, Any]) -> dict[str, Any]:
    """Cheap identity of the evaluated bag, used to catch two rows scored on one bag."""
    duration = report_dict.get("duration_sec")
    return {
        "total_messages": report_dict.get("total_messages"),
        "topic_count": report_dict.get("topic_count"),
        "duration_sec": round(float(duration), 3) if duration is not None else None,
    }


def _scores_from_report(report_dict: dict[str, Any]) -> dict[str, Any]:
    imu = report_dict.get("imu") or {}
    gnss = report_dict.get("gnss") or {}
    sync = report_dict.get("sync") or {}
    return {
        "overall_score": report_dict.get("overall_score"),
        "imu_score": imu.get("score"),
        "sync_score": sync.get("score"),
        "gnss_score": gnss.get("score"),
        "domain_score": report_dict.get("domain_score"),
    }


def refresh_manifest_scores(
    manifest: dict[str, Any],
    manifest_path: Path,
    *,
    repo_fallback: bool = True,
) -> tuple[int, int]:
    """Run bagx eval on cases whose bags exist; update embedded results."""
    from bagx import __version__
    from bagx.eval import detect_domain_names, evaluate_bag

    manifest_dir = manifest_path.parent
    repo_cache = REPO_ROOT / ".cache" / "dogfood"
    refreshed = 0
    missing = 0

    for case in manifest.get("cases", []):
        if case.get("availability") == "source_unavailable":
            missing += 1
            continue
        raw_path = str(case.get("bag_path", ""))
        if not raw_path:
            missing += 1
            continue

        resolved = _resolve_manifest_path(raw_path, manifest_dir)
        if not resolved.exists() and repo_fallback and case.get("provenance") == "simulator":
            fallback_name = Path(raw_path).name
            fallback = repo_cache / fallback_name
            if fallback.exists():
                resolved = fallback

        if not resolved.exists():
            missing += 1
            continue

        report = evaluate_bag(str(resolved))
        report_dict = report.to_dict()
        domains = sorted(detect_domain_names(report.topic_info, report.custom_domains))
        scores = _scores_from_report(report_dict)
        case["results"] = {
            **scores,
            "key_finding": _pick_key_finding(report_dict, domains),
            "fingerprint": _fingerprint_from_report(report_dict),
            "bagx_version": __version__,
            "evaluated_at": date.today().isoformat(),
        }
        refreshed += 1

    return refreshed, missing


def validate_manifest(manifest: dict[str, Any]) -> list[str]:
    """Return integrity problems in a scoreboard manifest (empty list = clean)."""
    problems: list[str] = []
    seen_names: set[str] = set()
    by_fingerprint: dict[tuple, str] = {}
    by_result: dict[tuple, str] = {}

    for case in manifest.get("cases", []):
        name = str(case.get("name", ""))
        if not name:
            problems.append("case without a name")
            continue
        if name in seen_names:
            problems.append(f"{name}: duplicate case name")
        seen_names.add(name)

        provenance = case.get("provenance")
        if provenance not in PROVENANCES:
            problems.append(f"{name}: provenance must be one of {PROVENANCES}, got {provenance!r}")
        availability = case.get("availability", "available")
        if availability not in AVAILABILITIES:
            problems.append(
                f"{name}: availability must be one of {AVAILABILITIES}, got {availability!r}"
            )

        results = case.get("results") or {}
        if results.get("overall_score") is None:
            continue
        if availability == "source_unavailable":
            problems.append(f"{name}: source_unavailable case must not carry scores")
        for key in ("bagx_version", "evaluated_at"):
            if not results.get(key):
                problems.append(f"{name}: scored case is missing results.{key}")

        fingerprint = results.get("fingerprint")
        if fingerprint:
            fp_key = tuple(sorted(fingerprint.items()))
            if fp_key in by_fingerprint:
                problems.append(
                    f"{name}: scored on the same bag as {by_fingerprint[fp_key]} "
                    f"(fingerprint {fingerprint})"
                )
            else:
                by_fingerprint[fp_key] = name

        result_key = (
            *(results.get(key) for key in _SCORE_KEYS),
            results.get("key_finding"),
        )
        if result_key in by_result:
            problems.append(
                f"{name}: results identical to {by_result[result_key]} "
                "(likely scored on the same bag)"
            )
        else:
            by_result[result_key] = name

    return problems


def _domain_order(domain: str) -> tuple[int, str]:
    order = {
        "SLAM": 0,
        "Autoware": 1,
        "Nav2": 2,
        "MoveIt": 3,
        "Perception": 4,
        "Manipulation": 5,
    }
    return (order.get(domain, 99), domain)


def render_scoreboard_table(manifest: dict[str, Any]) -> str:
    from bagx import __version__

    cases = list(manifest.get("cases", []))
    cases.sort(
        key=lambda c: (
            _domain_order(str(c.get("domain", ""))),
            str(c.get("display_name", c.get("name", ""))),
        )
    )

    scored = [c for c in cases if c.get("results", {}).get("overall_score") is not None]
    public = [c for c in cases if c.get("provenance") == "public"]
    simulator = [c for c in cases if c.get("provenance") == "simulator"]
    public_scored = [c for c in scored if c.get("provenance") == "public"]
    lines = [
        f"**{len(cases)} datasets** tracked ({len(public)} public · {len(simulator)} simulator "
        f"captures) · **{len(scored)} scored** ({len(public_scored)} public) · bagx {__version__}",
        "",
        "| Dataset | Domain | ROS | Data | Overall | IMU | Sync | Key finding | bagx |",
        "|---------|--------|-----|------|---------|-----|------|-------------|------|",
    ]

    for case in cases:
        display = str(case.get("display_name", case.get("name", "")))
        source_url = str(case.get("source_url", ""))
        if source_url:
            display = f"[{display}]({source_url})"

        domain = str(case.get("domain", ""))
        ros = str(case.get("ros", "ros2"))
        data = "sim" if case.get("provenance") == "simulator" else "public"
        if case.get("availability") == "source_unavailable":
            data = "unavailable"
        results = case.get("results", {}) or {}
        overall = _format_score(results.get("overall_score"))
        imu = _format_score(results.get("imu_score"))
        sync = _format_score(results.get("sync_score"))
        key_finding = str(results.get("key_finding", "")).replace("|", "\\|")
        bagx_ver = results.get("bagx_version") or "—"
        lines.append(
            f"| {display} | {domain} | {ros} | {data} | **{overall}** | {imu} | {sync} | {key_finding} | {bagx_ver} |"
        )

    return "\n".join(lines)


def render_reproduce_section(manifest: dict[str, Any]) -> str:
    lines = [
        "## Reproduce a row",
        "",
        "Each manifest entry includes a `reproduce` command. Example:",
        "",
        "```bash",
        "export BAGX_SCOREBOARD_BAGS=/path/to/downloaded/bags",
        "export BAGX_DB3_CACHE=/path/with/free/space  # for .db3.zstd decompression",
        "# also accepts BAGX_REALBAGS for NVIDIA / Autoware mirrors",
        "bagx eval \"$BAGX_SCOREBOARD_BAGS/r2b_galileo2\"",
        "```",
        "",
        "Refresh the table after downloading bags:",
        "",
        "```bash",
        "export BAGX_SCOREBOARD_BAGS=/path/to/bags",
        "python scripts/generate_scoreboard.py --refresh --write-manifest",
        "python scripts/generate_scoreboard.py",
        "```",
        "",
        "Manifest: [`benchmarks/scoreboard.json`](https://github.com/rsasaki0109/bagx/blob/main/benchmarks/scoreboard.json)",
        "",
    ]

    by_domain: dict[str, list[dict[str, Any]]] = {}
    for case in manifest.get("cases", []):
        if not case.get("reproduce") or case.get("availability") == "source_unavailable":
            continue
        domain = str(case.get("domain", "Other"))
        by_domain.setdefault(domain, []).append(case)

    lines.append("### Sample commands by domain")
    lines.append("")
    for domain in sorted(by_domain, key=lambda d: _domain_order(d)[0]):
        lines.append(f"**{domain}**")
        lines.append("")
        for case in by_domain[domain][:2]:
            name = case.get("display_name", case.get("name"))
            lines.append(f"- {name}: `{case['reproduce']}`")
        lines.append("")

    return "\n".join(lines).rstrip()


def build_scoreboard_markdown(manifest: dict[str, Any]) -> str:
    table = render_scoreboard_table(manifest)
    reproduce = render_reproduce_section(manifest)

    header = """# Public Dataset Scoreboard

bagx scores public rosbag datasets with a single command (`bagx eval`). This page is
**generated from** [`benchmarks/scoreboard.json`](https://github.com/rsasaki0109/bagx/blob/main/benchmarks/scoreboard.json)
so scores stay reproducible across releases.

Scores are **readiness hints**, not SLAM trajectory accuracy. They summarize sensor rates,
sync, IMU noise, and stack-specific topic coverage (Nav2 / Autoware / MoveIt / perception).

The **Data** column says where each bag comes from: `public` rows are scored on the
upstream download, `sim` rows on bags captured with `scripts/run_ros_dogfood.py`, and
`unavailable` rows are tracked but unscored because the upstream bag can no longer be
downloaded.

## Analysis articles

- [Why EuRoC MH_01_easy IMU looks noisy](scoreboard/why-euroc-mh01-imu-is-noisy.md)
- [Why Ouster OS0-32 scores 77](scoreboard/why-ouster-os0-32-scores-77.md)
- [Why NTU VIRAL IMU looks noisy](scoreboard/why-ntu-viral-imu-is-noisy.md)
- [Why Autoware all-sensors-bag4 scores 78](scoreboard/why-autoware-all-sensors-bag4-scores-78.md)

"""
    return f"{header}{MARKER_START}\n\n{table}\n\n{MARKER_END}\n\n{reproduce}\n"


def update_scoreboard_file(manifest: dict[str, Any], output_path: Path) -> None:
    content = build_scoreboard_markdown(manifest)
    if output_path.exists():
        existing = output_path.read_text()
        if MARKER_START in existing and MARKER_END in existing:
            # Keep the hand-written intro; regenerate the table and the reproduce section.
            before = existing[:existing.index(MARKER_START)]
            table = render_scoreboard_table(manifest)
            reproduce = render_reproduce_section(manifest)
            content = f"{before}{MARKER_START}\n\n{table}\n\n{MARKER_END}\n\n{reproduce}\n"
    output_path.write_text(content)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate docs/scoreboard.md from manifest")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_MANIFEST,
        help="Path to scoreboard manifest JSON",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output markdown path",
    )
    parser.add_argument(
        "--refresh",
        action="store_true",
        help="Re-run bagx eval for cases whose bag_path exists locally",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate manifest integrity and exit (non-zero on problems)",
    )
    parser.add_argument(
        "--write-manifest",
        action="store_true",
        help="Write refreshed scores back to the manifest JSON",
    )
    args = parser.parse_args()

    manifest_path = args.manifest.resolve()
    with open(manifest_path) as f:
        manifest = json.load(f)

    if args.refresh:
        refreshed, missing = refresh_manifest_scores(manifest, manifest_path)
        print(f"Refreshed {refreshed} case(s); {missing} bag(s) not found locally.")

    problems = validate_manifest(manifest)
    for problem in problems:
        print(f"manifest problem: {problem}")
    if args.check:
        raise SystemExit(1 if problems else 0)
    if problems and args.write_manifest:
        raise SystemExit("Refusing to write a manifest with integrity problems.")

    if args.refresh:
        if args.write_manifest:
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Updated manifest: {manifest_path}")

    update_scoreboard_file(manifest, args.output.resolve())
    print(f"Wrote scoreboard: {args.output.resolve()}")


if __name__ == "__main__":
    main()
