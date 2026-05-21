# anomaly — Anomaly Detection

Automatically detect anomalies and outliers in sensor data.

## Usage

```bash
bagx anomaly recording.db3
bagx anomaly recording.db3 --topic /gnss
bagx anomaly recording.db3 --json anomalies.json
```

## Detection types

### GNSS
- **position_jump**: Position change > 50m between consecutive messages
- **fix_drop**: Fix status transitions from valid to invalid
- **hdop_spike**: HDOP exceeds mean + 3σ

### IMU
- **accel_spike**: Acceleration magnitude exceeds mean + 4σ
- **gyro_spike**: Gyro rate exceeds mean + 4σ
- **imu_frequency_drop**: Message gap > 3× median interval

### General
- **rate_gap**: Any topic with message gap > 3× median interval

## Severity levels

| Level | Meaning |
|-------|---------|
| **high** | Likely sensor failure or data corruption |
| **medium** | Unusual but may be legitimate (e.g., rapid maneuver) |
| **low** | Minor deviation, informational |

## Temporal findings

Each anomaly run also produces a `findings` array — raw events aggregated
into structured `Finding` objects with `time_range` (absolute ROS time
nanoseconds). GNSS fix-drop events are paired with the next recovery (or
the bag end) to form `anomaly.gnss.fix_lost.<topic>` segments; other event
types cluster by `(topic, type)` within a 30 second window.

Anomaly severities map to finding severities as `low → info`,
`medium → warning`, `high → error`.

For CI workflows, prefer `bagx eval --include-anomaly` so the temporal
findings ship alongside the rest of the eval report and can be gated with
`bagx diff` or `bagx benchmark`. See
[eval — Temporal findings](eval.md#temporal-findings).
