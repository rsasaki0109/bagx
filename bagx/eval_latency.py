"""Pipeline latency recommendations for bagx eval.

When the output of a stage carries the header stamp of the input it was
computed from (stamp propagation), latency is measured exactly per message.
Otherwise it is approximated from record times.
"""

from __future__ import annotations

import numpy as np

# Share of output messages whose stamp must match an input stamp before the
# stamp-matched (exact) measurement replaces the record-time approximation.
STAMP_MATCH_MIN_RATIO = 0.5


def _stamp_matched_latencies(
    input_stamps: list[tuple[int, int]] | None,
    output_stamps: list[tuple[int, int]] | None,
    min_samples: int,
) -> list[float] | None:
    """Per-message latency (ms) for outputs whose header stamp equals an input stamp.

    Returns None unless enough outputs match for the pairing to be trusted.
    """
    if not input_stamps or not output_stamps:
        return None
    first_record_by_stamp: dict[int, int] = {}
    for record_ns, stamp_ns in input_stamps:
        first_record_by_stamp.setdefault(stamp_ns, record_ns)
    latencies = [
        (record_ns - first_record_by_stamp[stamp_ns]) / 1e6
        for record_ns, stamp_ns in output_stamps
        if stamp_ns in first_record_by_stamp and record_ns >= first_record_by_stamp[stamp_ns]
    ]
    if len(latencies) < max(min_samples, 1):
        return None
    if len(latencies) < STAMP_MATCH_MIN_RATIO * len(output_stamps):
        return None
    return latencies


def _add_pipeline_latency_recs(
    recs: list[str],
    topic_timestamps: dict[str, list[int]],
    pipeline: list[tuple[str, str, str]],
    *,
    min_output_samples: int = 5,
    stamps: dict[str, list[tuple[int, int]]] | None = None,
) -> None:
    """Measure and report pipeline latency between topic pairs.

    For each (input_topic, output_topic, label): if output header stamps match
    input stamps, report the exact per-message latency; otherwise compute the
    median delay from input to the next output message, which estimates
    processing latency.
    """
    stamps = stamps or {}
    for input_topic, output_topic, label in pipeline:
        exact = _stamp_matched_latencies(
            stamps.get(input_topic), stamps.get(output_topic), min_output_samples
        )
        if exact is not None:
            _append_latency_rec(
                recs, label, exact, note=" (header-stamp matched)",
                slow_suffix="may affect real-time performance",
                too_slow_suffix="too slow for real-time",
            )
            continue

        ts_in = topic_timestamps.get(input_topic, [])
        ts_out = topic_timestamps.get(output_topic, [])
        if len(ts_in) < max(min_output_samples, 1) or len(ts_out) < min_output_samples:
            continue

        arr_in = np.array(sorted(ts_in), dtype=np.int64)
        arr_out = np.array(sorted(ts_out), dtype=np.int64)

        # For each output message, find the most recent input BEFORE it
        # using binary search (j must NOT carry over between iterations)
        latencies_ms = []
        for t_out in arr_out:
            # Binary search: find the rightmost input <= t_out
            idx = int(np.searchsorted(arr_in, t_out, side="right")) - 1
            if 0 <= idx < len(arr_in):
                latency = (t_out - arr_in[idx]) / 1e6  # ns to ms
                # Only count if the input is reasonably recent (< 2x output interval)
                if latency >= 0:
                    latencies_ms.append(latency)

        if not latencies_ms:
            continue

        # If the stages run at a similar period, also evaluate index-aligned
        # pairing. This avoids under-reporting when true latency exceeds one
        # period and "latest input before output" starts matching the next frame.
        if len(arr_in) >= 6 and len(arr_out) >= 6:
            input_period_ms = float(np.median(np.diff(arr_in)) / 1e6)
            output_period_ms = float(np.median(np.diff(arr_out)) / 1e6)
            if input_period_ms > 0 and output_period_ms > 0:
                period_ratio = max(input_period_ms, output_period_ms) / min(input_period_ms, output_period_ms)
                if period_ratio <= 1.25:
                    pair_count = min(len(arr_in), len(arr_out))
                    aligned = ((arr_out[:pair_count] - arr_in[:pair_count]) / 1e6).astype(float)
                    aligned = aligned[aligned >= 0]
                    if len(aligned) >= 5:
                        aligned_median = float(np.median(aligned))
                        predecessor_median = float(np.median(latencies_ms))
                        if aligned_median > predecessor_median + max(input_period_ms * 0.25, 1.0):
                            latencies_ms = aligned.tolist()

        _append_latency_rec(
            recs, label, latencies_ms,
            slow_suffix="may affect real-time performance",
            too_slow_suffix="too slow for real-time",
        )


def _append_latency_rec(
    recs: list[str],
    label: str,
    latencies_ms: list[float],
    *,
    slow_suffix: str,
    too_slow_suffix: str,
    note: str = "",
) -> None:
    median_lat = float(np.median(latencies_ms))
    p95_lat = float(np.percentile(latencies_ms, 95))
    sample_note = ""
    if len(latencies_ms) < 5:
        plural = "s" if len(latencies_ms) != 1 else ""
        sample_note = f" ({len(latencies_ms)} sample{plural})"
    summary = f"Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note}{note}"
    if median_lat < 50:
        recs.append(f"  [green]:heavy_check_mark:[/green] {summary}")
    elif median_lat < 200:
        recs.append(f"  [yellow]:warning:[/yellow] {summary} — {slow_suffix}")
    else:
        recs.append(f"  [red]:x:[/red] {summary} — {too_slow_suffix}")


def _add_event_response_latency_recs(
    recs: list[str],
    topic_timestamps: dict[str, list[int]],
    pairs: list[tuple[str, str, str]],
    *,
    min_input_samples: int = 1,
    max_response_ms: float = 2000.0,
) -> None:
    """Measure sparse event-to-next-response latency."""
    for input_topic, output_topic, label in pairs:
        ts_in = topic_timestamps.get(input_topic, [])
        ts_out = topic_timestamps.get(output_topic, [])
        if len(ts_in) < min_input_samples or not ts_out:
            continue

        arr_in = np.array(sorted(ts_in), dtype=np.int64)
        arr_out = np.array(sorted(ts_out), dtype=np.int64)
        latencies_ms: list[float] = []
        for t_in in arr_in:
            idx = int(np.searchsorted(arr_out, t_in, side="right"))
            if idx >= len(arr_out):
                continue
            latency_ms = float((arr_out[idx] - t_in) / 1e6)
            if 0 <= latency_ms <= max_response_ms:
                latencies_ms.append(latency_ms)

        if len(latencies_ms) < min_input_samples:
            continue

        _append_latency_rec(
            recs, label, latencies_ms,
            slow_suffix="may affect responsiveness",
            too_slow_suffix="too slow for interactive use",
        )
