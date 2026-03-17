# scenario — Scene Extraction

Identify and extract time segments where notable or dangerous scenarios occur.

## Usage

```bash
bagx scenario recording.db3
bagx scenario recording.db3 --json scenarios.json
```

## Detection rules

| Rule | Trigger | Default threshold |
|------|---------|-------------------|
| **gnss_lost** | Consecutive no-fix messages | > 2.0 seconds |
| **sensor_dropout** | Topic stops publishing | > 2.0 seconds |
| **high_dynamics** | IMU acceleration magnitude | > 15.0 m/s² |
| **sync_degraded** | Sustained inter-topic delay | > 100 ms |

## Python API with custom thresholds

```python
from bagx.scenario import detect_scenarios

report = detect_scenarios(
    "recording.db3",
    gnss_lost_threshold_sec=5.0,      # more tolerant
    accel_threshold_mps2=20.0,        # higher threshold
    dropout_threshold_sec=1.0,        # more sensitive
)
```
