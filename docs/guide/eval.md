# eval — Quality Evaluation

Evaluate a single bag and produce a composite quality score (0–100).

## Usage

```bash
bagx eval recording.db3
bagx eval recording.db3 --json report.json
bagx eval recording.db3 --rules warehouse_bot
```

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
- `workflow.<topic_token>.{action_failures,result_failures,missing_service_responses}`

Use these ids in benchmark manifests via [`expected_findings`](benchmark.md#expected-findings).

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
