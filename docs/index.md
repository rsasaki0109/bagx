# bagx

**Post-processing analysis engine for ROS2 rosbag data.**

> **[Try bagx in your browser](https://rsasaki0109.github.io/bagx/demo/)** — drop a `.db3` bag,
> get a readiness report. No install, and your bag never leaves your machine.

> bagx is not a replacement for ros2 bag.
> ros2 bag handles recording & playback (I/O). bagx handles evaluation, understanding & comparison (Analysis).

## What can you do with bagx?

```bash
# Score a bag's sensor quality
bagx eval recording.db3
bagx rules list
bagx eval recording.db3 --rules warehouse_bot

# Compare two runs
bagx compare run_A.db3 run_B.db3

# Check time sync between sensors
bagx sync recording.db3 /camera /lidar

# Find sensor anomalies
bagx anomaly recording.db3

# Export for ML pipelines
bagx export recording.db3 --ai --format parquet

# Batch-evaluate a dataset
bagx batch eval ./recordings/*.db3 --csv summary.csv

# Re-run a benchmark suite on curated public/private bags
bagx benchmark benchmarks/open_data_suite.json
bagx benchmark benchmarks/non_slam_suite.json
```

## Real-world results

Evaluated on public SLAM datasets (see the [scoreboard](scoreboard.md) for all 30 rows and
reproduce commands):

| Dataset | Duration | Messages | GNSS | IMU | Sync | Overall |
|---------|----------|----------|------|-----|------|---------|
| Livox MID-360 (AIST driving) | 277s | 58K | N/A | 97.3 | 100.0 | **98.7** |
| TUM VI calib-imu1 | 52s | 18K | N/A | 90.8 | 100.0 | **95.4** |
| NTU VIRAL eee_03 (drone) | 181s | 390K | 80.0 | 72.0 | 100.0 | **84.0** |
| EuRoC MH_01_easy | 188s | 47K | N/A | 23.1 | 97.6 | **60.3** |

## Quick install

```bash
pip install bagx
```

See [Getting Started](getting-started.md) for details.
