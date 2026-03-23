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
