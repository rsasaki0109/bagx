# bagx

[![PyPI](https://img.shields.io/pypi/v/bagx)](https://pypi.org/project/bagx/)
[![CI](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/bagx/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)

Post-processing analysis engine for ROS2 rosbag data.

> **bagx is not a replacement for ros2 bag.**
> ros2 bag handles recording & playback (I/O). bagx handles evaluation, understanding & comparison (Analysis).

## What can you do with bagx?

**Before a test drive** — check sensor readiness:
```bash
bagx eval pre_drive.db3
# → GNSS fix rate 98%, HDOP 1.2, IMU 200Hz, Score: 87/100 ✓
```

**After a test drive** — compare two runs:
```bash
bagx compare run_A.db3 run_B.db3
# → GNSS Fix Rate: A=95% B=72% (degraded)
# → IMU Noise: A=0.04 B=0.03 (improved)
# → Winner: A
```

**Debugging sensor issues** — find anomalies:
```bash
bagx anomaly recording.db3
# → [12.3s] /gnss: position_jump (47.2m, severity: high)
# → [45.1s] /imu: accel_spike (23.4 m/s², severity: high)
# → [78.9s] /lidar: rate_gap (850ms, severity: medium)
```

**Checking time sync** — are sensors synchronized?
```bash
bagx sync recording.db3 /camera /lidar
# → Mean: 3.2ms | Max: 12.1ms | P95: 8.4ms | Outliers: 0.2%
```

**Extracting dangerous scenes** — for safety review:
```bash
bagx scenario recording.db3
# → [10.5s–14.2s] gnss_lost (3.7s, severity: high)
# → [32.1s–32.8s] high_dynamics (brake, 18.3 m/s²)
```

**Preparing data for ML** — export to Parquet:
```bash
bagx export recording.db3 --ai --format parquet
# → gnss.parquet (142 KB), imu.parquet (891 KB), lidar.parquet (2.3 MB)
```

**Asking questions in plain English**:
```bash
bagx ask recording.db3 "Is this bag suitable for SLAM evaluation?"
# → The bag contains LiDAR at 10Hz and IMU at 200Hz with good sync (3ms).
#   GNSS fix rate is 95%. Suitable for outdoor SLAM evaluation.
```

**Batch evaluation** — score an entire dataset:
```bash
bagx batch eval ./recordings/*.db3 --csv summary.csv
# → 12 bags evaluated, scores: 34–91, mean: 72.4
```

## Real-world evaluation results

Evaluated on public LiDAR SLAM datasets to demonstrate what bagx can tell you about your data.

### Batch scoring across datasets

```
$ bagx batch eval ./slam_datasets/

┏━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━┳━━━━━━┳━━━━━━━┳━━━━━━━┳━━━━━━━━━┓
┃ Bag                      ┃ Duration (s) ┃  Messages ┃ GNSS ┃   IMU ┃  Sync ┃ Overall ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━╇━━━━━━━━━━━╇━━━━━━╇━━━━━━━╇━━━━━━━╇━━━━━━━━━┩
│ Livox MID-360            │        277.2 │    58,217 │  N/A │  97.3 │  70.0 │    83.7 │
│ Ouster OS0-32            │         47.3 │     5,208 │  N/A │  79.6 │  73.6 │    76.6 │
│ Newer College (handheld) │        193.0 │   225,758 │  N/A │  84.5 │ 100.0 │    92.3 │
│ NTU VIRAL (drone)        │        583.6 │ 1,248,476 │ 80.0 │  59.5 │ 100.0 │    79.8 │
└──────────────────────────┴──────────────┴───────────┴──────┴───────┴───────┴─────────┘
```

**What this tells you:**

- **Newer College** scores highest (92.3) — excellent sensor sync across 4 cameras + 2 IMUs + LiDAR, low IMU noise. Ideal for LiDAR SLAM.
- **Livox MID-360** has the best IMU (97.3) but 25ms sync delay between IMU and LiDAR — tightly-coupled SLAM should compensate for this latency.
- **NTU VIRAL** has perfect GNSS (100% fix) and sync, but IMU scores lower (59.5) due to noisier drone-mounted sensors.
- **Ouster OS0-32** internal IMU is usable (79.6) but not great — consider adding an external IMU for better SLAM performance.

### Detailed evaluation (NTU VIRAL drone dataset)

```
$ bagx eval ntu_viral_tnp_01.db3

Duration: 583.6s | Messages: 1,248,476 | Topics: 19

             GNSS Quality
┏━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━┓
┃ Metric                ┃      Value ┃
┡━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━┩
│ Messages              │      28975 │
│ Fix Rate              │     100.0% │
│ Altitude (mean±std)   │ 76.5±2.4   │
│ Score                 │   80.0/100 │
└───────────────────────┴────────────┘
             IMU Quality
┏━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Metric               ┃                        Value ┃
┡━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ Messages             │                       225142 │
│ Accel Noise (xyz)    │       0.3049, 0.3180, 0.6054 │
│ Gyro Noise (xyz)     │ 0.009956, 0.014712, 0.004014 │
│ Accel Bias Stability │                       0.0321 │
│ Score                │                     59.5/100 │
└──────────────────────┴──────────────────────────────┘
             Topic Sync Quality
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━┳━━━━━━━━━━┓
┃ Topic Pair                                 ┃ Mean (ms) ┃ Max (ms) ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━╇━━━━━━━━━━┩
│ /dji_sdk/imu ↔ /imu/imu                   │       1.3 │     21.9 │
│ /dji_sdk/gps_position ↔ /dji_sdk/imu      │       1.6 │     31.5 │
│ Score                                      │ 100.0/100 │          │
└────────────────────────────────────────────┴───────────┴──────────┘

Overall Score: 79.8/100
```

### Cross-IMU sync analysis

```
$ bagx sync ntu_viral.db3 /dji_sdk/imu /os1_cloud_node1/imu

┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━┓
┃ Topic Pair                           ┃  Count ┃ Mean ms ┃  Max ms ┃  P95 ms ┃ Outlier ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━┩
│ /dji_sdk/imu ↔ /os1_cloud_node1/imu │ 231793 │    2.50 │   10.14 │    4.75 │    0.0% │
└──────────────────────────────────────┴────────┴─────────┴─────────┴─────────┴─────────┘
```

→ DJI onboard IMU and Ouster LiDAR IMU are synchronized within 2.5ms on average, with zero outliers. Suitable for multi-IMU fusion.

### Datasets used

- [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/) — Drone with GNSS, 4 IMUs, 2 LiDARs, cameras, UWB
- [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/) — Handheld with Ouster LiDAR, Alphasense cameras + IMU
- [glim](https://koide3.github.io/glim/) — Indoor/outdoor with Livox MID-360
- [Koide LiDAR-Camera Calibration](https://koide3.github.io/direct_visual_lidar_calibration/) — Static calibration bags with Livox

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
