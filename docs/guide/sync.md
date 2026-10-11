# sync — Sync Analysis

Analyze time synchronization between two topics.

## Usage

```bash
bagx sync recording.db3 /camera /lidar
bagx sync recording.db3 /imu /gnss --json sync.json
```

## Metrics

For each message in topic A, finds the nearest timestamp in topic B:

| Metric | Description |
|--------|-------------|
| Mean delay | Average nearest-neighbor delay (ms) |
| Max delay | Worst-case delay (ms) |
| Median delay | 50th percentile (ms) |
| P95 delay | 95th percentile (ms) |
| Std | Standard deviation (ms) |
| Outlier rate | Fraction of delays > mean + 3σ |

## Interpreting results

| Mean delay | Quality |
|-----------|---------|
| < 5 ms | Excellent — suitable for tight sensor fusion |
| 5–20 ms | Good — suitable for most SLAM applications |
| 20–50 ms | Fair — may need timestamp compensation |
| > 50 ms | Poor — investigate clock sync or hardware triggers |

These are **recording-time nearest-neighbor distances**, not measured physical
sensor delays. A dense periodic stream can have a close neighbor even when a
publisher's acquisition timestamps have a constant offset.

## Header-stamp diagnostics in eval

`bagx eval recording.db3` also examines `header.stamp` for the sensor pairs in
its sync analysis. The normal output includes `Header sync:` lines. Use
`--findings-only` or `--json eval.json` for the underlying measurements.
By default these are the first five adjacent sensor pairs by name. To cover
every eligible sensor against the best-scoring active IMU, use
`bagx eval recording.db3 --sync-pairing imu`. Without an active IMU, that mode
uses all adjacent sensor pairs. Both modes need at least five shared samples
per stream in a pair.

Two measurements are reported together:

- **Header nearest-neighbor mean / P95:** acquisition timestamp distances,
  walking the sparser stream within the shared header-time interval. Dense
  samples bracketing that interval remain available as nearest neighbors.
- **Relative stamp/record offset:** the median of `header.stamp − record_time`
  for topic B minus that for topic A, using their shared recording-time interval.
  Positive values mean B stamps later relative to its recording time. A common
  offset on both topics cancels out.

For example, a 10 Hz LiDAR with stamps shifted by 100 ms can still find an IMU
stamp within 2.5 ms on a 200 Hz stream. The second measurement exposes the
100 ms difference rather than silently matching a different IMU sample.
It also includes transport and recording latency: **it does not prove a
physical sensor offset**, and should not be applied blindly as a correction.

On the public [AIST Livox MID-360 driving bag](https://zenodo.org/records/14841855),
the unreleased diagnostic reports a **0.88 ms** header-neighbor mean and a
**−102.15 ms** relative stamp/record offset (LiDAR minus IMU). Scan accumulation,
transport and recording latency must be checked before interpreting that
difference as a clock or calibration fault. All 58,217 messages were evaluated:
the overall score stays **98.7**, and the existing findings match main exactly.

Each pair produces a `sync_quality` finding with an id of the form
`sync.header_stamp.<topic_a_token>.<topic_b_token>.<state>`:

| State | Severity | Meaning |
|-------|----------|---------|
| `measured` | info | Measurements available; not proof of hardware synchronization |
| `offset` | warning | Absolute relative stamp/record offset is at least 20 ms |
| `delay_high` | warning | Header nearest-neighbor mean is at least 20 ms |
| `clock_mismatch` | warning | Relative offset is at least 1 s; header clocks may differ, so no header-neighbor distance is reported |
| `unavailable` | info | Missing/zero stamps, fewer than five samples, coverage below 90%, repeated/regressing stamps, or insufficient overlap |

JSON evidence includes `header_stamp_samples`, `header_stamp_coverage`,
`stamp_minus_record_median_ms` per topic and
`relative_stamp_record_offset_ms` (B − A). When header times can be compared,
it also includes `header_nearest_neighbor_mean_ms`,
`header_nearest_neighbor_p95_ms`, and `header_nearest_neighbor_count`.
The sample counts and coverage refer to the shared recording-time interval.
Zero/nonpositive stamps are excluded; repeated or regressing stamps make that
pair unavailable. Large clock differences and start/stop boundaries are not
converted into purported acquisition delays.

Header-stamp checks add findings and recommendations only. Default sync / overall
scores, default sensor-pair selection, the `bagx sync` command's recording-time metrics,
report keys and version numbers are unchanged. Choosing `--sync-pairing imu`
can change scores because it changes which recording-time pairs are evaluated.
New warning findings can affect
CI gates that use severity thresholds, even when score thresholds still pass.
