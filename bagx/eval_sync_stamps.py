"""Header-stamp diagnostics alongside the existing record-time sync score.

Nearest header stamps alone can hide an offset by matching a different sample
of a dense periodic stream. Compare stamp-minus-record offsets as well. Those
offsets also contain transport/recording latency, so they are evidence to
investigate, not a measurement of physical sensor misalignment.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bagx.findings import Evidence, Finding, clean_topic_token, finding_id
from bagx.topic_filters import nearest_neighbor_delays_ns

if TYPE_CHECKING:
    from bagx.eval import EvalReport

MIN_SAMPLES = 5
MIN_STAMP_COVERAGE = 0.9
OFFSET_WARNING_MS = 20.0
CLOCK_MISMATCH_MS = 1000.0  # same scale as TF's clock comparison
MAX_STAMP_NS = np.iinfo(np.int64).max


def generate_stamp_sync_findings(report: EvalReport) -> list[Finding]:
    """Diagnose the same pairs as eval's record-time sync, without rescoring."""
    if not report.sync:
        return []
    return [_pair_finding(report, a, b) for a, b in report.sync.topic_pairs]


def _pair_finding(report: EvalReport, a: str, b: str) -> Finding:
    evidence: list[Evidence] = []
    token_a, token_b = clean_topic_token(a), clean_topic_token(b)

    def result(state: str, title: str, *, warning: bool = False) -> Finding:
        return Finding(
            id=finding_id("sync", "header_stamp", token_a, token_b, state),
            title=f"{a} ↔ {b}: {title}",
            severity="warning" if warning else "info",
            category="sync_quality",
            affected_topics=[a, b],
            evidence=evidence,
            confidence="medium" if warning else "low",
            suggested_action=(
                "Check publisher clocks and transport/recording latency before applying "
                "timestamp compensation; this is not a physical sensor delay measurement."
                if warning else None
            ),
        )

    records_a = report._topic_timestamps.get(a, [])
    records_b = report._topic_timestamps.get(b, [])
    if not records_a or not records_b:
        return result("unavailable", "header sync unavailable: no recording timestamps")
    start = max(min(records_a), min(records_b))
    end = min(max(records_a), max(records_b))
    if end <= start:
        return result("unavailable", "header sync unavailable: no recording-time overlap")

    streams = []
    for topic, records in [(a, records_a), (b, records_b)]:
        total = sum(start <= t <= end for t in records)
        samples = sorted(
            ((record, stamp) for record, stamp in report._topic_stamps.get(topic, [])
             if start <= record <= end and 0 < stamp <= MAX_STAMP_NS),
            key=lambda sample: sample[0],
        )
        coverage = len(samples) / max(total, 1)
        evidence.extend([
            Evidence(metric="header_stamp_samples", observed=len(samples), topic=topic),
            Evidence(metric="header_stamp_coverage", observed=round(coverage, 4),
                     expected=">=0.9", topic=topic),
        ])
        if len(samples) < MIN_SAMPLES or coverage < MIN_STAMP_COVERAGE:
            return result("unavailable", "header sync unavailable: insufficient nonzero stamps")
        stamps = np.array([stamp for _, stamp in samples], dtype=np.int64)
        if np.any(stamps[1:] <= stamps[:-1]):
            return result("unavailable", "header sync unavailable: repeated or regressing stamps")
        # Subtract integer nanoseconds before converting to float (epoch times
        # cannot retain nanosecond precision as float64).
        offsets = sorted(stamp - record for record, stamp in samples)
        # Keep twice the median as a Python integer, even for even-sized
        # samples. Large common epoch offsets then cancel without float loss.
        median_twice_ns = offsets[(len(offsets) - 1) // 2] + offsets[len(offsets) // 2]
        evidence.append(Evidence(metric="stamp_minus_record_median_ms",
                                 observed=round(median_twice_ns / 2e6, 6), unit="ms", topic=topic))
        streams.append((stamps, median_twice_ns))

    stamps_a, offset_a = streams[0]
    stamps_b, offset_b = streams[1]
    relative_offset = (offset_b - offset_a) / 2e6
    evidence.append(Evidence(metric="relative_stamp_record_offset_ms",
                             observed=round(relative_offset, 6), unit="ms"))
    offset_text = f"relative stamp/record offset {relative_offset:+.1f}ms (B − A)"
    if abs(relative_offset) >= CLOCK_MISMATCH_MS:
        return result("clock_mismatch", f"{offset_text}; header clocks may differ",
                      warning=True)

    # Walk only acquisition timestamps in the shared interval. Keep bracketing
    # dense samples just outside it; discarding them inflates edge distances.
    stamp_start = max(int(stamps_a[0]), int(stamps_b[0]))
    stamp_end = min(int(stamps_a[-1]), int(stamps_b[-1]))
    shared_a = stamps_a[(stamps_a >= stamp_start) & (stamps_a <= stamp_end)]
    shared_b = stamps_b[(stamps_b >= stamp_start) & (stamps_b <= stamp_end)]
    if min(len(shared_a), len(shared_b)) < MIN_SAMPLES:
        return result("offset" if abs(relative_offset) >= OFFSET_WARNING_MS else "unavailable",
                      f"{offset_text}; insufficient header-time overlap",
                      warning=abs(relative_offset) >= OFFSET_WARNING_MS)

    sparse, dense = ((shared_a, stamps_b) if len(shared_a) <= len(shared_b)
                     else (shared_b, stamps_a))
    delays_ms = nearest_neighbor_delays_ns(sparse, dense) / 1e6
    mean_ms = float(np.mean(delays_ms))
    evidence.extend([
        Evidence(metric="header_nearest_neighbor_mean_ms", observed=round(mean_ms, 6), unit="ms"),
        Evidence(metric="header_nearest_neighbor_p95_ms",
                 observed=round(float(np.percentile(delays_ms, 95)), 6), unit="ms"),
        Evidence(metric="header_nearest_neighbor_count", observed=len(delays_ms)),
    ])
    warning = abs(relative_offset) >= OFFSET_WARNING_MS or mean_ms >= OFFSET_WARNING_MS
    state = "offset" if abs(relative_offset) >= OFFSET_WARNING_MS else (
        "delay_high" if warning else "measured"
    )
    return result(state, f"header nearest-neighbor mean {mean_ms:.1f}ms; {offset_text}",
                  warning=warning)
