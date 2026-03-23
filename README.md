# bagx

[![PyPI](https://img.shields.io/pypi/v/bagx)](https://pypi.org/project/bagx/)
[![CI](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/bagx/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)

**One command to know if your rosbag is ready for SLAM.**

```bash
pip install bagx && bagx eval recording.db3
```

<p align="center">
  <img src="docs/eval_demo.svg" alt="bagx eval output" width="700">
</p>

**You get SLAM-ready parameter values, not just scores.**

## Why bagx?

You recorded a rosbag. Now what?

| Without bagx | With bagx |
|---|---|
| Open PlotJuggler, manually inspect each topic | `bagx eval bag.db3` → scores + recommendations |
| Guess IMU noise parameters for SLAM config | Exact noise values to copy-paste into config |
| SLAM diverges, no idea why | `bagx anomaly bag.db3` → find the IMU spike at 45.1s |
| "Is this IMU good enough for LIO?" | bagx tells you: "noisy, LiDAR-only may outperform LIO" |
| Compare two sensor configs manually | `bagx compare A.db3 B.db3` → quantified winner |

## Real results on public SLAM datasets

```
$ bagx batch eval ./slam_datasets/

┃ Dataset              ┃ Duration ┃ Messages ┃ GNSS ┃  IMU ┃  Sync ┃ Overall ┃
┡━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━╇━━━━━━━━━━╇━━━━━━╇━━━━━━╇━━━━━━━╇━━━━━━━━━┩
│ Newer College        │    193s  │   226K   │  N/A │ 84.5 │ 100.0 │    92.3 │
│ Livox MID-360        │    277s  │    58K   │  N/A │ 97.3 │  70.0 │    83.7 │
│ NTU VIRAL (drone)    │    584s  │  1,248K  │ 80.0 │ 59.5 │ 100.0 │    79.8 │
│ Ouster OS0-32        │     47s  │     5K   │  N/A │ 79.6 │  73.6 │    76.6 │
```

**What this tells you instantly:**

- **Newer College** (92.3) — best data quality, ideal for benchmarking LiDAR SLAM
- **Livox MID-360** — best IMU (97.3) but 25ms sync delay, needs deskew compensation
- **NTU VIRAL** — IMU too noisy (59.5) for tightly-coupled LIO, use LiDAR-only
- **Ouster OS0-32** — 50Hz IMU is too slow for LIO (200Hz+ recommended)

## Install

```bash
pip install bagx
```

Works **without ROS2** — reads `.db3` files directly via SQLite + built-in CDR parsers.

## All commands

| Command | What it does |
|---------|-------------|
| `bagx eval bag.db3` | Score sensor quality + actionable recommendations |
| `bagx compare A.db3 B.db3` | Side-by-side metric diff with winner |
| `bagx sync bag.db3 /imu /lidar` | Time sync analysis (mean, P95, outliers) |
| `bagx anomaly bag.db3` | Find sensor spikes, dropouts, jumps |
| `bagx scenario bag.db3` | Extract GNSS-lost / high-dynamics segments |
| `bagx export bag.db3 --ai` | Parquet/JSON for ML pipelines |
| `bagx scene bag.db3 --csv out.csv` | 3D trajectory extraction |
| `bagx ask bag.db3 "Is this good for SLAM?"` | LLM-powered Q&A |
| `bagx batch eval *.db3 --csv summary.csv` | Score entire datasets |
| `bagx info bag.db3` | Quick topic/message summary |

## IMU noise = Allan Deviation at τ=dt

bagx estimates IMU noise via `std(diff(x))/√2`, which is mathematically equivalent to Allan Deviation at one sample interval. Verified on real data:

```
Livox MID-360 (200Hz):
  diff-based noise:  0.0114 m/s²
  Allan σ(τ=dt):     0.0114 m/s²   ← exact match
```

For static IMU data, bagx automatically computes full Allan Variance (VRW/ARW + bias instability).

## Links

- [Documentation](https://rsasaki0109.github.io/bagx/)
- [PyPI](https://pypi.org/project/bagx/)
- [Changelog](https://github.com/rsasaki0109/bagx/releases)

## License

MIT
