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
