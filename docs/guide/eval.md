# eval — Quality Evaluation

Evaluate a single bag and produce a composite quality score (0–100).

## Usage

```bash
bagx eval recording.db3
bagx eval recording.db3 --json report.json
bagx eval recording.db3 --rules warehouse_bot
bagx eval recording.db3 --findings-only
bagx eval recording.db3 --severity-min warning
bagx eval recording.db3 --findings-only --severity-min error
bagx eval recording.db3 --include-anomaly       # also merge fix-lost / spike segments
bagx eval recording.db3 --badge badge.json      # emit a shields.io readiness badge
bagx eval recording.db3 --tune fast_lio          # write FAST-LIO starting-point yaml
bagx eval recording.db3 --tune kiss_icp -o cfg/ # write cfg/kiss_icp_tuned.yaml
bagx tune --list                                # list supported frameworks
```

### Output filtering

- `--findings-only` skips the sensor-quality tables and recommendations and
  prints the [structured findings list](#structured-findings) instead.
- `--severity-min {info|warning|error|critical}` filters the findings list to
  that severity or higher. Applied to both the `--findings-only` text view and
  the `--json` payload.
- `--include-anomaly` also runs anomaly detection and merges the resulting
  **temporal findings** (with `time_range`) into the report. Useful when the
  eval output is destined for `bagx diff` or `bagx benchmark` CI gates. Off
  by default because it re-reads the bag.

### Readiness badge

`--badge <file>` writes a [shields.io endpoint](https://shields.io/badges/endpoint-badge)
JSON payload built from the composite score:

```json
{ "schemaVersion": 1, "label": "Nav2 readiness", "message": "76.6/100", "color": "green" }
```

- The **label** folds in the detected stack (`Nav2 readiness`, `Autoware readiness`, ...),
  or `bag readiness` when no domain is detected. Override it with `--badge-label`.
- The **colour** tracks the [overall score](#overall-score): `brightgreen` ≥ 85,
  `green` ≥ 70, `yellow` ≥ 50, `orange` ≥ 30, `red` below.

Host the file somewhere reachable by URL and reference it from a README:

```markdown
![bag readiness](https://img.shields.io/endpoint?url=https://example.com/badge.json)
```

`--badge` composes with `--json`; both files are written in one eval run.

## What it measures

### GNSS Quality
- **Fix rate**: Percentage of messages with valid fix (status ≥ 0)
- **HDOP**: Horizontal dilution of precision from position covariance
- **Altitude**: Mean and standard deviation

### IMU Quality
- **Noise**: Estimated via first-order differencing — mathematically equivalent to Allan Deviation at τ=dt (one sample interval). This removes low-frequency vehicle dynamics and isolates sensor noise, even during motion.
- **Bias stability**: Standard deviation of windowed means — approximates Allan Deviation at longer τ values
- **Frequency**: Measured from message timestamps

!!! info "Multi-IMU handling"
    If a bag contains multiple IMU topics, each is evaluated independently. The best-scoring IMU is reported.

!!! note "Relationship to Allan Variance"
    The noise values reported by bagx are `std(diff(x)) / √2`, which equals the Allan Deviation at τ = 1/fs (sampling period). This was verified on real IMU data (Livox MID-360, 200Hz): diff-based = 0.0114 m/s², Allan σ(τ=dt) = 0.0114 m/s². For the standard Allan VRW/ARW specification (at τ=1s), use a dedicated Allan Variance tool on static data.

### Topic Sync
- **Mean/max delay**: Nearest-neighbor timestamp matching between adjacent topic pairs
- Evaluates the first 5 topic pairs by name order

### Overall Score
Weighted average of available component scores (GNSS, IMU, Sync).

## Structured findings

`bagx eval --json` includes a machine-readable `findings` array alongside the
human-facing `recommendations`. Findings are the recommended surface for CI
gating, benchmark expectations, and cross-release diffing — they have stable
ids and never depend on recommendation text.

```json
{
  "id": "sync.delay.high",
  "title": "Sensor sync delay is high",
  "severity": "warning",
  "category": "sync_quality",
  "domain": "slam",
  "affected_topics": ["/imu", "/lidar"],
  "evidence": [
    {"metric": "mean_delay_ms", "observed": 25.6, "expected": "<20", "unit": "ms", "topic": null}
  ],
  "suggested_action": "Enable deskew or timestamp compensation for tightly-coupled fusion.",
  "confidence": "medium"
}
```

### Field schema

| Field | Type | Notes |
| --- | --- | --- |
| `id` | string | Stable lowercase id, dot-separated (`<domain>.<area>.<qualifier>`). |
| `title` | string | One-line human label. |
| `severity` | `info` / `warning` / `error` / `critical` | See severity policy below. |
| `category` | string | Coarse grouping (`domain_detection`, `sensor_quality`, `topic_presence`, `rate_quality`, `sync_quality`, `workflow_observability`, `workflow_outcome`, `custom_rules`). |
| `domain` | string \| null | `slam` / `nav2` / `autoware` / `moveit` / `perception` / `robotarm` or a custom domain id. |
| `affected_topics` | string[] | Topics this finding refers to. |
| `evidence` | object[] | Measured facts: `metric`, `observed`, optional `expected`, `unit`, `topic`. |
| `suggested_action` | string \| null | Recommended next step. |
| `confidence` | `low` / `medium` / `high` | Defaults to `medium`. |
| `time_range` | object \| null | Optional `{start_ns, end_ns}` (absolute ROS time) for findings scoped to a sub-interval of the bag — see [Temporal findings](#temporal-findings). |

### Severity policy

- **info** — readiness signal is healthy or merely informational.
- **warning** — readiness is degraded but the bag is still usable; user action recommended.
- **error** — readiness is unreliable; the bag should not be used as-is.
- **critical** — reserved for catastrophic readiness loss (e.g. a mandatory topic missing on a safety-critical pipeline). Not emitted by built-in checks today; reserved for downstream plugins.

### Id naming convention

Ids follow `<domain>.<area>.<qualifier>` where the qualifier encodes the
observed state, so passing states are also benchmark-checkable:

- `gnss.fix_rate.{good,gappy,unreliable}`
- `imu.accel_noise.{good,noisy}`, `imu.gyro_noise.{good,noisy}`, `imu.rate.low`
- `sync.delay.{good,high}`
- `nav2.detected`, `nav2.odometry.rate_low`, `nav2.scan.rate_low`, `nav2.cmd_vel.rate_low`
- `nav2.missing_cmd_vel`, `nav2.missing_global_plan`, `nav2.missing_navigate_to_pose`
- `gnss.missing`, `imu.missing`, `gnss.hdop.high`
- `custom.<domain_id>.evaluated`
- `custom.<domain_id>.<label_token>.{pass,fail,skipped}` — one per custom check
- `workflow.<topic_token>.{action_failures,result_failures,missing_service_responses}`

Use these ids in benchmark manifests via [`expected_findings`](benchmark.md#expected-findings).

### JSON Schema

bagx ships a JSON Schema describing the Finding object at
`bagx/schema/findings.schema.json` (Draft 2020-12). Locate it from Python:

```python
from bagx.contracts import findings_schema_path, findings_schema

print(findings_schema_path())   # absolute path to the .json file
schema = findings_schema()       # parsed dict, cached
```

Use it from external tools (Grafana, Slack bots, GitHub Actions) to validate
finding payloads without depending on bagx's Python runtime.

### Temporal findings

A finding may carry a `time_range` object that scopes it to a sub-interval of
the bag. When `--include-anomaly` is set, the anomaly engine aggregates raw
events into temporal findings:

- `anomaly.gnss.fix_lost.<topic>` — segment from drop to recovery (or bag
  end if no recovery). Evidence includes `duration_sec` and `recovered`.
- `anomaly.imu.accel_spike.<topic>` / `anomaly.imu.gyro_spike.<topic>` —
  clusters of spike events within a 30 s window.
- `anomaly.imu.frequency_drop.<topic>` / `anomaly.rate.gap.<topic>` —
  message gap spans.

```json
{
  "id": "anomaly.gnss.fix_lost.gnss",
  "title": "GNSS fix lost on /gnss",
  "severity": "warning",
  "category": "sensor_quality",
  "affected_topics": ["/gnss"],
  "time_range": {"start_ns": 1700000009000000000, "end_ns": 1700000009900000000},
  "evidence": [
    {"metric": "duration_sec", "observed": 0.9, "unit": "s", "topic": "/gnss"},
    {"metric": "recovered", "observed": false, "topic": "/gnss"}
  ]
}
```

`bagx diff` matches temporal findings segment-by-segment by `time_range`
overlap, and `bagx benchmark` accepts a `time_range_overlap` constraint —
see [diff](diff.md) and [benchmark](benchmark.md#expected-findings).

## Custom message stacks

If your bag uses custom messages, you can still layer in domain-specific checks with `--rules`.

`bagx` does not need to decode those payloads for this mode. The rule engine works from:

- topic names
- topic types
- message rates
- timestamp-based latency between topics

The `--rules` argument accepts either a JSON file path or a plugin name from `bagx rules list`.

See [custom-rules — Custom Message Rules](custom-rules.md) for the rule format.

## Customizing thresholds

Via the Python API, you can tune scoring with `EvalConfig`:

```python
from bagx.eval import EvalConfig, evaluate_bag

config = EvalConfig(
    gnss_fix_weight=0.6,
    gnss_hdop_weight=0.4,
    imu_accel_noise_excellent=0.05,   # m/s²
    imu_accel_noise_scale=200.0,
    imu_gyro_noise_excellent=0.005,   # rad/s
    imu_gyro_noise_scale=2000.0,
    sync_delay_excellent_ms=5.0,
    sync_delay_scale=1.5,
)

report = evaluate_bag("recording.db3", config=config)
print(report.overall_score)
```
