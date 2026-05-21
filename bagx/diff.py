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
    """One entry in a findings diff."""

    id: str
    kind: ChangeKind
    baseline_severity: str | None = None
    current_severity: str | None = None
    title: str | None = None
    category: str | None = None
    domain: str | None = None
    affected_topics: list[str] = field(default_factory=list)
    evidence_drift: list[EvidenceDrift] = field(default_factory=list)

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
    """Compute a FindingsDiff from two finding lists."""
    baseline_by_id = {str(f.get("id", "")): f for f in baseline_findings if f.get("id")}
    current_by_id = {str(f.get("id", "")): f for f in current_findings if f.get("id")}
    all_ids = sorted(set(baseline_by_id) | set(current_by_id))

    changes: list[FindingChange] = []
    for fid in all_ids:
        b = baseline_by_id.get(fid)
        c = current_by_id.get(fid)
        if b is None and c is not None:
            changes.append(_new_change(fid, c))
        elif c is None and b is not None:
            changes.append(_gone_change(fid, b))
        elif b is not None and c is not None:
            change = _compare_existing(fid, b, c)
            if change.kind == "same" and not include_same:
                continue
            changes.append(change)

    changes.sort(key=lambda c: (-_KIND_RANK.get(c.kind, 0), c.id))
    return FindingsDiff(
        baseline_path=baseline_path,
        current_path=current_path,
        changes=changes,
    )


def _new_change(fid: str, current: dict[str, Any]) -> FindingChange:
    return FindingChange(
        id=fid,
        kind="new",
        current_severity=current.get("severity"),
        title=current.get("title"),
        category=current.get("category"),
        domain=current.get("domain"),
        affected_topics=list(current.get("affected_topics", []) or []),
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
