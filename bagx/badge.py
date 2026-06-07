"""Readiness badge generation for bagx eval reports.

Produces a `shields.io endpoint <https://shields.io/badges/endpoint-badge>`_
JSON payload from an :class:`bagx.eval.EvalReport`. Host the resulting file
anywhere reachable by URL (raw GitHub, GitHub Pages, an artifact store) and
reference it from a README::

    ![bag readiness](https://img.shields.io/endpoint?url=https://example.com/bag-badge.json)

The badge colour reflects the composite readiness score so a green badge in a
README signals, at a glance, that a recorded bag is fit for its declared stack.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from bagx.eval import EvalReport

# (inclusive lower bound, shields.io colour) ordered high → low.
_COLOR_THRESHOLDS: tuple[tuple[float, str], ...] = (
    (85.0, "brightgreen"),
    (70.0, "green"),
    (50.0, "yellow"),
    (30.0, "orange"),
    (0.0, "red"),
)

DEFAULT_LABEL = "bag readiness"


def score_color(score: float) -> str:
    """Map a 0-100 readiness score to a shields.io colour keyword."""
    for threshold, color in _COLOR_THRESHOLDS:
        if score >= threshold:
            return color
    return "red"


def eval_badge(report: EvalReport, label: str | None = None) -> dict:
    """Build a shields.io endpoint JSON payload from an eval report.

    The detected domains (Nav2, Autoware, ...) are folded into the label when
    no explicit label is given, so the badge reads e.g. ``Nav2 readiness``.
    """
    from bagx.eval import detect_domain_names

    score = float(report.overall_score)
    if label is None:
        domains = sorted(detect_domain_names(report.topic_info, report.custom_domains))
        label = f"{'/'.join(domains)} readiness" if domains else DEFAULT_LABEL

    return {
        "schemaVersion": 1,
        "label": label,
        "message": f"{score:.1f}/100",
        "color": score_color(score),
    }
