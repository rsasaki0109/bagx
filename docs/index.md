# bagx

**Post-processing analysis engine for ROS2 rosbag data.**

> bagx is not a replacement for ros2 bag.
> ros2 bag handles recording & playback (I/O). bagx handles evaluation, understanding & comparison (Analysis).

## What can you do with bagx?

```bash
# Score a bag's sensor quality
bagx eval recording.db3

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

Evaluated on public LiDAR SLAM datasets:

| Dataset | Duration | Messages | GNSS | IMU | Sync | Overall |
|---------|----------|----------|------|-----|------|---------|
| Livox MID-360 | 277s | 58K | N/A | 97.3 | 70.0 | **83.7** |
| Newer College | 193s | 226K | N/A | 84.5 | 100.0 | **92.3** |
| NTU VIRAL (drone) | 584s | 1.2M | 80.0 | 59.5 | 100.0 | **79.8** |
| Ouster OS0-32 | 47s | 5K | N/A | 79.6 | 73.6 | **76.6** |

## Quick install

```bash
pip install bagx
```

See [Getting Started](getting-started.md) for details.
