# batch — Batch Processing

Evaluate or analyze multiple bags at once.

## Usage

```bash
# Evaluate all bags in a directory
bagx batch eval ./recordings/

# Glob pattern
bagx batch eval ./data/*.db3 --csv summary.csv

# Multiple paths
bagx batch eval bag1.db3 bag2.db3 bag3.db3

# Batch anomaly detection
bagx batch anomaly ./recordings/ --json anomalies.json
```

## CSV output

`bagx batch eval` with `--csv` produces:

| Column | Description |
|--------|-------------|
| `bag_path` | Path to the bag file |
| `duration_sec` | Recording duration |
| `message_count` | Total messages |
| `gnss_score` | GNSS quality score |
| `imu_score` | IMU quality score |
| `sync_score` | Sync quality score |
| `overall_score` | Composite score |

## Path resolution

- **Files**: Used directly (`bag.db3`)
- **Directories**: Recursively scans for `.db3` and `.mcap` files
- **Globs**: Expanded by the shell (`*.db3`)
