# export — Data Export

Export bag data to AI/analytics-friendly formats.

## Usage

```bash
# Parquet (default)
bagx export recording.db3

# JSON format
bagx export recording.db3 --format json

# AI-friendly mode (relative timestamps)
bagx export recording.db3 --ai --format parquet

# Specific topics only
bagx export recording.db3 --topics /gnss,/imu

# Custom output directory
bagx export recording.db3 --output ./my_export
```

## Options

| Flag | Description |
|------|-------------|
| `--format`, `-f` | `parquet` (default) or `json` |
| `--output`, `-o` | Output directory (default: `./export`) |
| `--topics`, `-t` | Comma-separated topic filter |
| `--ai` | Relative timestamps from bag start |
| `--flatten` / `--no-flatten` | Flatten nested fields (default: flatten) |

## AI mode

With `--ai`, timestamps are normalized to seconds from bag start (first message = 0.0). This is useful for ML pipelines where absolute epoch timestamps are not needed.

## Output structure

One file per topic:

```
export/
  gnss.parquet          # /gnss topic
  imu.parquet           # /imu topic
  ouster_points.parquet # /ouster/points topic
```

Each file contains columns: `timestamp_sec`, `timestamp_ns`, plus flattened message fields.
