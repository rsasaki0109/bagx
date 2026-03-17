# bagx

Post-processing analysis engine for ROS2 rosbag data.

> **bagx is not a replacement for ros2 bag.**
> ros2 bag handles recording & playback (I/O). bagx handles evaluation, understanding & comparison (Analysis).

## Install

```bash
pip install -e .
```

With ROS2 installed, full message deserialization via `rosbag2_py` is available.
Without ROS2, basic analysis of `.db3` files still works (SQLite fallback with built-in CDR parsers).

Optional extras:

```bash
pip install -e ".[mcap]"  # .mcap file support
pip install -e ".[llm]"   # LLM-powered ask command (Anthropic / OpenAI)
```

## Commands

### `bagx info` — Bag summary

```bash
bagx info recording.db3
```

### `bagx eval` — Quality evaluation

Evaluate a single bag and produce a composite quality score.

```bash
bagx eval recording.db3
bagx eval recording.db3 --json report.json
```

Metrics:
- **GNSS**: Fix rate, HDOP statistics
- **IMU**: Accel/gyro noise, bias stability, frequency
- **SYNC**: Inter-topic mean/max delay
- **Overall score**: 0–100

### `bagx compare` — Compare two bags

```bash
bagx compare A.db3 B.db3
bagx compare A.db3 B.db3 --json diff.json
```

Output:
- Per-metric A/B values and diff
- improved / degraded / unchanged verdict
- Overall winner

### `bagx sync` — Inter-topic sync analysis

```bash
bagx sync recording.db3 /gnss /lidar
bagx sync recording.db3 /imu /camera --json sync.json
```

Output:
- Mean, max, median, P95 delay
- Standard deviation, outlier rate

### `bagx export` — Export to AI/analytics formats

```bash
# Parquet (default)
bagx export recording.db3

# AI-friendly mode (relative timestamps, normalized)
bagx export recording.db3 --ai --format parquet

# JSON, specific topics only
bagx export recording.db3 --format json --topics /gnss,/imu

# No flattening
bagx export recording.db3 --no-flatten
```

### `bagx anomaly` — Anomaly detection

Automatically detect anomalies and outliers in sensor data.

```bash
bagx anomaly recording.db3
bagx anomaly recording.db3 --topic /gnss
bagx anomaly recording.db3 --json anomalies.json
```

Detects:
- **GNSS**: Position jumps, sudden fix loss, HDOP spikes
- **IMU**: Accel/gyro spikes (N*σ), frequency drops
- **General**: Message rate gaps (>3x median interval)

### `bagx scenario` — Dangerous scene extraction

Extract time segments where notable or dangerous scenarios occur.

```bash
bagx scenario recording.db3
bagx scenario recording.db3 --json scenarios.json
```

Rules:
- **GNSS lost**: Consecutive no-fix exceeding threshold duration
- **Sensor dropout**: Topic stops publishing beyond threshold
- **High dynamics**: IMU acceleration magnitude exceeds threshold (hard braking, sharp turns)
- **Sync degraded**: Sustained inter-topic delay above threshold

### `bagx ask` — Natural language queries

Ask questions about a bag file, answered by an LLM.

```bash
bagx ask recording.db3 "What sensors are in this bag?"
bagx ask recording.db3 "Is the GNSS quality good?" --provider openai
```

Requires `ANTHROPIC_API_KEY` or `OPENAI_API_KEY` environment variable.

### `bagx scene` — 3D state extraction

Extract position, orientation, and velocity time series.

```bash
bagx scene recording.db3
bagx scene recording.db3 --csv scene.csv
bagx scene recording.db3 --topics /odom,/imu
```

Auto-detects PoseStamped, Odometry, Imu, NavSatFix, and TFMessage topics.

### `bagx batch` — Batch processing

Evaluate or analyze multiple bags at once.

```bash
bagx batch eval *.db3 --csv summary.csv
bagx batch eval ./recordings/
bagx batch anomaly *.db3 --json anomalies.json
```

## Design

- **CLI-first**: Commands are the primary interface
- **Reproducible & scriptable**: Designed for batch processing and CI/CD
- **Data output**: Numbers, JSON, Parquet, CSV — no GUI
- **Modular**: Each command is an independent module
- **ROS2-optional**: Works without ROS2 via SQLite + CDR fallback

## Project structure

```
bagx/
  cli.py        # typer-based CLI entry point
  reader.py     # Bag reading (rosbag2_py / mcap / SQLite fallback)
  eval.py       # Quality evaluation engine
  compare.py    # Two-bag comparison
  sync.py       # Inter-topic sync analysis
  export.py     # Parquet / JSON export
  anomaly.py    # Anomaly detection
  scenario.py   # Dangerous scene extraction
  ask.py        # LLM-powered natural language queries
  scene.py      # 3D state extraction
  batch.py      # Batch processing
  schema.py     # Schema inference & normalization
```

## Tech stack

- **typer** + **rich**: CLI & display
- **rosbag2_py**: ROS2 bag reading (optional)
- **mcap** + **mcap-ros2-support**: MCAP format support (optional)
- **numpy / pandas**: Numerical analysis
- **pyarrow**: Parquet output
- **anthropic / openai**: LLM integration (optional)

<!-- CLI_REFERENCE_START -->

## CLI Reference

```
Usage: bagx [OPTIONS] COMMAND [ARGS]...

 Post-processing analysis engine for ROS2 rosbag data.

╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --install-completion          Install completion for the current shell.      │
│ --show-completion             Show completion for the current shell, to copy │
│                               it or customize the installation.              │
│ --help                        Show this message and exit.                    │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ───────────────────────────────────────────────────────────────────╮
│ eval       Evaluate quality of a single bag file.                            │
│ compare    Compare quality metrics of two bag files.                         │
│ sync       Analyze time synchronization between two topics.                  │
│ export     Export bag data to AI/analytics-friendly formats.                 │
│ anomaly    Detect anomalies and outliers in sensor data.                     │
│ scenario   Identify and extract dangerous or interesting scenarios.          │
│ ask        Ask a natural language question about a bag file, answered by an  │
│            LLM.                                                              │
│ scene      Extract 3D state (position, orientation, velocity) time series.   │
│ info       Show bag file summary information.                                │
│ batch      Batch operations on multiple bags                                 │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx eval`

```
Usage: bagx eval [OPTIONS] BAG

 Evaluate quality of a single bag file.

 Analyzes GNSS fix rate/HDOP, IMU noise/bias, and inter-topic sync,
 producing a composite quality score.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file (.db3 or directory) [required]      │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx compare`

```
Usage: bagx compare [OPTIONS] BAG_A BAG_B

 Compare quality metrics of two bag files.

 Evaluates both bags and reports per-metric differences,
 indicating which bag is better overall.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag_a      TEXT  Path to the first bag file [required]                  │
│ *    bag_b      TEXT  Path to the second bag file [required]                 │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx sync`

```
Usage: bagx sync [OPTIONS] BAG TOPIC_A TOPIC_B

 Analyze time synchronization between two topics.

 Reports mean/max/median delay, variance, P95, and outlier rate.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag          TEXT  Path to the bag file [required]                      │
│ *    topic_a      TEXT  First topic name [required]                          │
│ *    topic_b      TEXT  Second topic name [required]                         │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx export`

```
Usage: bagx export [OPTIONS] BAG

 Export bag data to AI/analytics-friendly formats.

 Outputs one file per topic in JSON or Parquet format,
 with optional timestamp normalization and field flattening.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --output   -o                  TEXT  Output directory [default: ./export]    │
│ --format   -f                  TEXT  Output format: parquet or json          │
│                                      [default: parquet]                      │
│ --topics   -t                  TEXT  Comma-separated topic names (default:   │
│                                      all)                                    │
│ --ai                                 Enable AI-friendly mode (relative       │
│                                      timestamps, normalized)                 │
│ --flatten      --no-flatten          Flatten nested message fields           │
│                                      [default: flatten]                      │
│ --help                               Show this message and exit.             │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx anomaly`

```
Usage: bagx anomaly [OPTIONS] BAG

 Detect anomalies and outliers in sensor data.

 Finds GNSS position jumps, IMU spikes, message rate gaps,
 and other anomalous events in the bag file.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --topic  -t      TEXT  Analyze only this topic                               │
│ --json   -j      TEXT  Output JSON report to file                            │
│ --help                 Show this message and exit.                           │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx scenario`

```
Usage: bagx scenario [OPTIONS] BAG

 Identify and extract dangerous or interesting scenarios.

 Detects GNSS loss, sensor dropouts, high dynamics events,
 and sync degradation periods.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx ask`

```
Usage: bagx ask [OPTIONS] BAG QUESTION

 Ask a natural language question about a bag file, answered by an LLM.

 Gathers bag context (summary, eval, message samples) and sends it
 along with your question to an LLM for analysis.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag           TEXT  Path to the bag file (.db3 or directory) [required] │
│ *    question      TEXT  Natural language question about the bag [required]  │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --provider  -p      TEXT  LLM provider: 'anthropic' or 'openai'              │
│                           [default: anthropic]                               │
│ --help                    Show this message and exit.                        │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx scene`

```
Usage: bagx scene [OPTIONS] BAG

 Extract 3D state (position, orientation, velocity) time series.

 Auto-detects topics with scene-relevant message types (PoseStamped,
 Odometry, Imu, NavSatFix, TFMessage) unless specific topics are given.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --csv     -c      TEXT  Export scene states to CSV file                      │
│ --json    -j      TEXT  Output JSON report to file                           │
│ --topics  -t      TEXT  Comma-separated topic names (default: auto-detect)   │
│ --help                  Show this message and exit.                          │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx info`

```
Usage: bagx info [OPTIONS] BAG

 Show bag file summary information.

╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --help          Show this message and exit.                                  │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx batch`

```
Usage: bagx batch [OPTIONS] COMMAND [ARGS]...

 Batch operations on multiple bags

╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --help          Show this message and exit.                                  │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ───────────────────────────────────────────────────────────────────╮
│ eval      Evaluate quality of multiple bag files.                            │
│ anomaly   Run anomaly detection on multiple bag files.                       │
╰──────────────────────────────────────────────────────────────────────────────╯
```


<!-- CLI_REFERENCE_END -->
