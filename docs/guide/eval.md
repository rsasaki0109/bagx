# eval — Quality Evaluation

Evaluate a single bag and produce a composite quality score (0–100).

## Usage

```bash
bagx eval recording.db3
bagx eval recording.db3 --json report.json
```

## What it measures

### GNSS Quality
- **Fix rate**: Percentage of messages with valid fix (status ≥ 0)
- **HDOP**: Horizontal dilution of precision from position covariance
- **Altitude**: Mean and standard deviation

### IMU Quality
- **Noise**: Estimated via first-order differencing (high-pass filter), removing vehicle dynamics
- **Bias stability**: Drift of windowed means over time
- **Frequency**: Measured from message timestamps

!!! info "Multi-IMU handling"
    If a bag contains multiple IMU topics, each is evaluated independently. The best-scoring IMU is reported.

### Topic Sync
- **Mean/max delay**: Nearest-neighbor timestamp matching between adjacent topic pairs
- Evaluates the first 5 topic pairs by name order

### Overall Score
Weighted average of available component scores (GNSS, IMU, Sync).

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
