"""Approximate pipeline latency recommendations for bagx eval."""

from __future__ import annotations

import numpy as np

def _add_pipeline_latency_recs(
    recs: list[str],
    topic_timestamps: dict[str, list[int]],
    pipeline: list[tuple[str, str, str]],
    *,
    min_output_samples: int = 5,
) -> None:
    """Measure and report pipeline latency between topic pairs.

    For each (input_topic, output_topic, label), computes the median delay
    from input to the next output message, which estimates processing latency.
    """
    for input_topic, output_topic, label in pipeline:
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

        median_lat = float(np.median(latencies_ms))
        p95_lat = float(np.percentile(latencies_ms, 95))
        sample_note = ""
        if len(latencies_ms) < 5:
            plural = "s" if len(latencies_ms) != 1 else ""
            sample_note = f" ({len(latencies_ms)} sample{plural})"

        if median_lat < 50:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note}"
            )
        elif median_lat < 200:
            recs.append(
                f"  [yellow]:warning:[/yellow] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — may affect real-time performance"
            )
        else:
            recs.append(
                f"  [red]:x:[/red] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — too slow for real-time"
            )


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

        median_lat = float(np.median(latencies_ms))
        p95_lat = float(np.percentile(latencies_ms, 95))
        sample_note = ""
        if len(latencies_ms) < 5:
            plural = "s" if len(latencies_ms) != 1 else ""
            sample_note = f" ({len(latencies_ms)} sample{plural})"

        if median_lat < 50:
            recs.append(
                f"  [green]:heavy_check_mark:[/green] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note}"
            )
        elif median_lat < 200:
            recs.append(
                f"  [yellow]:warning:[/yellow] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — may affect responsiveness"
            )
        else:
            recs.append(
                f"  [red]:x:[/red] Pipeline {label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — too slow for interactive use"
            )
