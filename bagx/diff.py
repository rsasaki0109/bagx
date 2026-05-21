"""Compare two bagx eval JSON reports by finding id.

Loads two ``bagx eval --json`` outputs and surfaces what changed between
them. Stable finding ids (`<domain>.<area>.<qualifier>`) are used as the
join key, so renames in human-facing recommendation text never affect the
diff. Designed for PR-time CI checks and release regression tracking.

See ``docs/guide/diff.md`` for usage.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Literal

from rich.console import Console

from bagx.contracts import report_metadata
from bagx.findings import SEVERITY_ORDER

ChangeKind = Literal["new", "gone", "worse", "better", "same"]

_KIND_RANK: dict[str, int] = {"same": 0, "better": 1, "gone": 2, "new": 3, "worse": 4}

_DRIFT_RATIO_THRESHOLD = 0.10


@dataclass
class EvidenceDrift:
    """A meaningful change in an evidence value between two reports."""

    metric: str
    topic: str | None
    baseline_observed: Any
    current_observed: Any
    drift_ratio: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FindingChange:
    """One entry in a findings diff.

    For temporal findings (with time_range), distinct segments of the same
    finding id appear as separate FindingChange entries. The
    baseline_time_range / current_time_range fields record which segment
    each side of the change refers to.
    """

    id: str
    kind: ChangeKind
    baseline_severity: str | None = None
    current_severity: str | None = None
    title: str | None = None
    category: str | None = None
    domain: str | None = None
    affected_topics: list[str] = field(default_factory=list)
    evidence_drift: list[EvidenceDrift] = field(default_factory=list)
    baseline_time_range: dict[str, int] | None = None
    current_time_range: dict[str, int] | None = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["evidence_drift"] = [d.to_dict() for d in self.evidence_drift]
        return data


@dataclass
class FindingsDiff:
    """Result of comparing two eval reports."""

    baseline_path: str
    current_path: str
    changes: list[FindingChange] = field(default_factory=list)

    def by_kind(self, kind: ChangeKind) -> list[FindingChange]:
        return [c for c in self.changes if c.kind == kind]

    def has_regression(self, severity_threshold: str = "warning") -> bool:
        """True when any new or worse finding meets the severity threshold."""
        threshold_rank = SEVERITY_ORDER.get(severity_threshold, -1)
        for change in self.changes:
            if change.kind not in {"new", "worse"}:
                continue
            sev = change.current_severity or ""
            if SEVERITY_ORDER.get(sev, -1) >= threshold_rank:
                return True
        return False

    def to_dict(self) -> dict[str, Any]:
        data = {
            "baseline_path": self.baseline_path,
            "current_path": self.current_path,
            "summary": {
                "new": len(self.by_kind("new")),
                "gone": len(self.by_kind("gone")),
                "worse": len(self.by_kind("worse")),
                "better": len(self.by_kind("better")),
                "same": len(self.by_kind("same")),
            },
            "changes": [c.to_dict() for c in self.changes],
        }
        data.update(report_metadata("diff"))
        return data


def load_findings(path: str | Path) -> tuple[str, list[dict[str, Any]]]:
    """Load a bagx eval JSON file and return its (resolved path, findings)."""
    resolved = Path(path).expanduser().resolve()
    with resolved.open() as f:
        data = json.load(f)
    findings = data.get("findings", [])
    if not isinstance(findings, list):
        raise ValueError(f"{resolved}: 'findings' must be a list (got {type(findings).__name__})")
    return str(resolved), findings


def compare_findings(
    baseline_path: str,
    baseline_findings: list[dict[str, Any]],
    current_path: str,
    current_findings: list[dict[str, Any]],
    *,
    include_same: bool = False,
) -> FindingsDiff:
    """Compute a FindingsDiff from two finding lists.

    Findings are joined by id. Within an id, findings with a ``time_range``
    are matched segment-by-segment: overlapping ranges count as the same
    segment (so a slightly-shifted ``fix_lost`` segment compares cleanly),
    non-overlapping ranges count as different segments (so a new failure
    window shows up as a separate change).
    """
    baseline_by_id: dict[str, list[dict[str, Any]]] = {}
    for f in baseline_findings:
        fid = str(f.get("id", ""))
        if fid:
            baseline_by_id.setdefault(fid, []).append(f)
    current_by_id: dict[str, list[dict[str, Any]]] = {}
    for f in current_findings:
        fid = str(f.get("id", ""))
        if fid:
            current_by_id.setdefault(fid, []).append(f)

    all_ids = sorted(set(baseline_by_id) | set(current_by_id))

    changes: list[FindingChange] = []
    for fid in all_ids:
        b_list = baseline_by_id.get(fid, [])
        c_list = current_by_id.get(fid, [])
        for change in _match_segments(fid, b_list, c_list):
            if change.kind == "same" and not include_same:
                continue
            changes.append(change)

    changes.sort(key=lambda c: (
        -_KIND_RANK.get(c.kind, 0),
        c.id,
        (c.current_time_range or c.baseline_time_range or {}).get("start_ns", 0),
    ))
    return FindingsDiff(
        baseline_path=baseline_path,
        current_path=current_path,
        changes=changes,
    )


def _match_segments(
    fid: str,
    baseline: list[dict[str, Any]],
    current: list[dict[str, Any]],
) -> list[FindingChange]:
    """Pair baseline/current findings of the same id by time_range overlap.

    Bag-global findings (time_range is None or absent) are matched
    positionally, preserving the single-instance behavior of pre-v0.4
    reports. Findings with explicit time_range are matched greedily by
    overlap; unmatched entries become new/gone changes.
    """
    b_global = [f for f in baseline if not _has_time_range(f)]
    c_global = [f for f in current if not _has_time_range(f)]
    b_temporal = [f for f in baseline if _has_time_range(f)]
    c_temporal = [f for f in current if _has_time_range(f)]

    changes: list[FindingChange] = []

    # Bag-global findings: pair positionally (typically one of each).
    for i in range(max(len(b_global), len(c_global))):
        b = b_global[i] if i < len(b_global) else None
        c = c_global[i] if i < len(c_global) else None
        changes.append(_pair_to_change(fid, b, c))

    # Temporal findings: greedy overlap matching.
    matched_current: set[int] = set()
    for b in b_temporal:
        b_tr = b.get("time_range") or {}
        match_idx: int | None = None
        for j, c in enumerate(c_temporal):
            if j in matched_current:
                continue
            c_tr = c.get("time_range") or {}
            if _ranges_overlap(b_tr, c_tr):
                match_idx = j
                break
        if match_idx is not None:
            matched_current.add(match_idx)
            changes.append(_pair_to_change(fid, b, c_temporal[match_idx]))
        else:
            changes.append(_pair_to_change(fid, b, None))

    for j, c in enumerate(c_temporal):
        if j in matched_current:
            continue
        changes.append(_pair_to_change(fid, None, c))

    return changes


def _pair_to_change(
    fid: str,
    baseline: dict[str, Any] | None,
    current: dict[str, Any] | None,
) -> FindingChange:
    if baseline is None and current is not None:
        return _new_change(fid, current)
    if current is None and baseline is not None:
        return _gone_change(fid, baseline)
    assert baseline is not None and current is not None
    return _compare_existing(fid, baseline, current)


def _has_time_range(finding: dict[str, Any]) -> bool:
    tr = finding.get("time_range")
    return isinstance(tr, dict) and "start_ns" in tr and "end_ns" in tr


def _ranges_overlap(a: dict[str, Any], b: dict[str, Any]) -> bool:
    try:
        a_start = int(a["start_ns"])
        a_end = int(a["end_ns"])
        b_start = int(b["start_ns"])
        b_end = int(b["end_ns"])
    except (KeyError, TypeError, ValueError):
        return False
    return a_start <= b_end and b_start <= a_end


def _new_change(fid: str, current: dict[str, Any]) -> FindingChange:
    return FindingChange(
        id=fid,
        kind="new",
        current_severity=current.get("severity"),
        title=current.get("title"),
        category=current.get("category"),
        domain=current.get("domain"),
        affected_topics=list(current.get("affected_topics", []) or []),
        current_time_range=current.get("time_range") if _has_time_range(current) else None,
    )


def _gone_change(fid: str, baseline: dict[str, Any]) -> FindingChange:
    return FindingChange(
        id=fid,
        kind="gone",
        baseline_severity=baseline.get("severity"),
        title=baseline.get("title"),
        category=baseline.get("category"),
        domain=baseline.get("domain"),
        affected_topics=list(baseline.get("affected_topics", []) or []),
        baseline_time_range=baseline.get("time_range") if _has_time_range(baseline) else None,
    )


def _compare_existing(
    fid: str, baseline: dict[str, Any], current: dict[str, Any]
) -> FindingChange:
    b_sev = str(baseline.get("severity", ""))
    c_sev = str(current.get("severity", ""))
    b_rank = SEVERITY_ORDER.get(b_sev, -1)
    c_rank = SEVERITY_ORDER.get(c_sev, -1)

    if c_rank > b_rank:
        kind: ChangeKind = "worse"
    elif c_rank < b_rank:
        kind = "better"
    else:
        kind = "same"

    drift: list[EvidenceDrift] = []
    if kind == "same":
        drift = _evidence_drift(baseline.get("evidence", []), current.get("evidence", []))

    return FindingChange(
        id=fid,
        kind=kind,
        baseline_severity=b_sev or None,
        current_severity=c_sev or None,
        title=current.get("title") or baseline.get("title"),
        category=current.get("category") or baseline.get("category"),
        domain=current.get("domain") or baseline.get("domain"),
        affected_topics=list(current.get("affected_topics", []) or baseline.get("affected_topics", []) or []),
        evidence_drift=drift,
        baseline_time_range=baseline.get("time_range") if _has_time_range(baseline) else None,
        current_time_range=current.get("time_range") if _has_time_range(current) else None,
    )


def _evidence_drift(
    baseline_evidence: list[dict[str, Any]], current_evidence: list[dict[str, Any]]
) -> list[EvidenceDrift]:
    """Return evidence values that changed materially between reports."""
    def keyfn(ev: dict[str, Any]) -> tuple[str, str | None]:
        return str(ev.get("metric", "")), ev.get("topic")

    b_map = {keyfn(ev): ev for ev in baseline_evidence}
    c_map = {keyfn(ev): ev for ev in current_evidence}
    drifts: list[EvidenceDrift] = []
    for key, c_ev in c_map.items():
        b_ev = b_map.get(key)
        if b_ev is None:
            continue
        b_obs = b_ev.get("observed")
        c_obs = c_ev.get("observed")
        if b_obs == c_obs:
            continue
        ratio = _numeric_drift_ratio(b_obs, c_obs)
        if ratio is not None and ratio < _DRIFT_RATIO_THRESHOLD:
            continue
        drifts.append(EvidenceDrift(
            metric=key[0],
            topic=key[1],
            baseline_observed=b_obs,
            current_observed=c_obs,
            drift_ratio=ratio,
        ))
    return drifts


def _numeric_drift_ratio(a: Any, b: Any) -> float | None:
    try:
        af = float(a)
        bf = float(b)
    except (TypeError, ValueError):
        return None
    denom = max(abs(af), abs(bf), 1.0)
    return abs(af - bf) / denom


_KIND_GLYPH = {"new": "+", "gone": "-", "worse": "~", "better": "~", "same": "="}
_KIND_STYLE = {
    "new": "red",
    "worse": "red",
    "gone": "yellow",
    "better": "green",
    "same": "dim",
}


def print_diff_text(diff: FindingsDiff, console: Console | None = None) -> None:
    """Pretty-print a FindingsDiff to a console."""
    if console is None:
        console = Console()
    console.print(
        f"\n[bold]Findings diff[/bold]: "
        f"[dim]{diff.baseline_path}[/dim] → [dim]{diff.current_path}[/dim]"
    )
    counts = ", ".join(
        f"{k}={len(diff.by_kind(k))}" for k in ("new", "gone", "worse", "better", "same")
    )
    console.print(f"  {counts}\n")

    if not diff.changes:
        console.print("[dim]No differences.[/dim]\n")
        return

    for change in diff.changes:
        style = _KIND_STYLE.get(change.kind, "white")
        glyph = _KIND_GLYPH.get(change.kind, "?")
        sev = _format_severity_transition(change)
        console.print(
            f"  [{style}]{glyph} {change.kind.upper():<6}[/{style}] {sev} "
            f"[bold]{change.id}[/bold] — {change.title or ''}"
        )
        span = _format_time_range(change)
        if span:
            console.print(f"    [dim]segment:[/dim] {span}")
        if change.affected_topics:
            console.print(f"    [dim]topics:[/dim] {', '.join(change.affected_topics)}")
        for drift in change.evidence_drift:
            t = f" [{drift.topic}]" if drift.topic else ""
            ratio = f" ({drift.drift_ratio:.0%} drift)" if drift.drift_ratio else ""
            console.print(
                f"    [dim]evidence drift:[/dim] {drift.metric}{t} "
                f"{drift.baseline_observed} → {drift.current_observed}{ratio}"
            )
    console.print()


def print_diff_markdown(diff: FindingsDiff, stream: Any) -> None:
    """Write a markdown rendering suitable for PR comments."""
    summary = diff.to_dict()["summary"]
    write = stream.write
    write("### bagx findings diff\n\n")
    write(f"`{Path(diff.baseline_path).name}` → `{Path(diff.current_path).name}`\n\n")
    write(
        "| new | gone | worse | better | same |\n"
        "| ---: | ---: | ----: | -----: | ---: |\n"
        f"| {summary['new']} | {summary['gone']} | {summary['worse']} | "
        f"{summary['better']} | {summary['same']} |\n\n"
    )
    if not diff.changes:
        write("_No differences._\n")
        return

    has_temporal = any(
        c.baseline_time_range or c.current_time_range for c in diff.changes
    )
    if has_temporal:
        write("| kind | id | severity | segment | title |\n")
        write("| ---- | -- | -------- | ------- | ----- |\n")
        for change in diff.changes:
            sev = _format_severity_transition(change, plain=True)
            title = (change.title or "").replace("|", "\\|")
            span = _format_time_range(change) or "—"
            write(f"| {change.kind} | `{change.id}` | {sev} | {span} | {title} |\n")
    else:
        write("| kind | id | severity | title |\n")
        write("| ---- | -- | -------- | ----- |\n")
        for change in diff.changes:
            sev = _format_severity_transition(change, plain=True)
            title = (change.title or "").replace("|", "\\|")
            write(f"| {change.kind} | `{change.id}` | {sev} | {title} |\n")
    write("\n")


def write_diff_json(diff: FindingsDiff, stream: Any) -> None:
    """Write the diff as JSON."""
    json.dump(diff.to_dict(), stream, indent=2)


def _format_severity_transition(change: FindingChange, *, plain: bool = False) -> str:
    if change.kind == "new":
        return change.current_severity or ""
    if change.kind == "gone":
        return change.baseline_severity or ""
    if change.baseline_severity == change.current_severity:
        return change.current_severity or ""
    arrow = "->" if plain else "→"
    return f"{change.baseline_severity} {arrow} {change.current_severity}"


def _format_time_range(change: FindingChange) -> str:
    """Render time_range info as a compact human-readable span."""
    b = change.baseline_time_range
    c = change.current_time_range
    if not b and not c:
        return ""
    if change.kind == "new" and c:
        return _format_one_range(c)
    if change.kind == "gone" and b:
        return _format_one_range(b)
    if b == c and c:
        return _format_one_range(c)
    if b and c:
        return f"{_format_one_range(b)} → {_format_one_range(c)}"
    if c:
        return _format_one_range(c)
    if b:
        return _format_one_range(b)
    return ""


def _format_one_range(tr: dict[str, Any]) -> str:
    try:
        start = int(tr["start_ns"]) / 1e9
        end = int(tr["end_ns"]) / 1e9
    except (KeyError, TypeError, ValueError):
        return "?"
    if abs(end - start) < 1e-6:
        return f"t={start:.1f}s"
    return f"t={start:.1f}-{end:.1f}s"


def run_diff(
    baseline_path: str | Path,
    current_path: str | Path,
    *,
    include_same: bool = False,
) -> FindingsDiff:
    """Convenience: load both reports and compare them."""
    b_path, b_findings = load_findings(baseline_path)
    c_path, c_findings = load_findings(current_path)
    return compare_findings(
        b_path, b_findings, c_path, c_findings, include_same=include_same
    )


def iter_change_kinds() -> Iterable[ChangeKind]:
    """Iteration helper for typed enumeration."""
    return ("new", "gone", "worse", "better", "same")
