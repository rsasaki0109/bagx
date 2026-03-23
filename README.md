# bagx

[![PyPI](https://img.shields.io/pypi/v/bagx)](https://pypi.org/project/bagx/)
[![CI](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/bagx/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)

**Stop guessing why your SLAM diverges. One command finds the problem.**

```bash
pip install bagx
bagx eval your_bag.db3
```

<p align="center">
  <img src="docs/eval_demo.svg" alt="bagx eval output" width="700">
</p>

bagx analyzes your rosbag and gives you:
- **Exact IMU noise values** to paste into your SLAM config
- **Sync delay warnings** so you know when to enable deskew
- **"Use LiDAR-only"** when your IMU is too noisy for LIO

## The problem

You recorded a rosbag. You run SLAM. It diverges. Now you spend hours asking:

- *"Is my IMU noise too high for LIO?"* → `bagx eval` tells you
- *"What noise value should I set in the SLAM config?"* → bagx gives you the exact number
- *"Are my LiDAR and IMU actually synchronized?"* → `bagx sync` shows the delay
- *"Where did the data quality drop?"* → `bagx anomaly` pinpoints the exact timestamp
- *"Which of my 10 bags is the best for benchmarking?"* → `bagx batch eval` ranks them all

## Example: catching a real problem

```
$ bagx eval ouster_os0-32.db3

Overall Score: 76.6/100

Recommendations:
  ✔ IMU accel noise 0.087 m/s² — good for LIO, set imu_acc_noise_density to 0.087
  ⚠ IMU gyro noise 0.022 rad/s — consider lowering IMU integration weight
  ⚠ IMU rate 50Hz is low — 200Hz+ recommended for tightly-coupled LIO
  ⚠ LiDAR↔IMU sync delay 23ms — enable per-point deskew in SLAM
```

Without bagx, you'd discover these issues *after* hours of failed SLAM runs.

## Install

```bash
pip install bagx
```

Works **without ROS2** — reads `.db3` files directly via SQLite.

## Commands

| Command | One-liner |
|---------|-----------|
| `bagx eval bag.db3` | Is this bag ready for SLAM? What parameters should I use? |
| `bagx compare A.db3 B.db3` | Which sensor config is better? |
| `bagx sync bag.db3 /imu /lidar` | Are my sensors synchronized? |
| `bagx anomaly bag.db3` | Where did sensor quality drop? |
| `bagx scenario bag.db3` | Find GNSS-lost and high-dynamics segments |
| `bagx export bag.db3 --ai` | Export to Parquet/JSON for ML |
| `bagx batch eval *.db3 --csv` | Rank an entire dataset |
| `bagx ask bag.db3 "question"` | Ask questions via LLM |

## Tested on public datasets

| Dataset | IMU | Sync | Score | Insight |
|---------|-----|------|-------|---------|
| Newer College | 84 | 100 | **92** | Best data, ideal for SLAM benchmarking |
| Livox MID-360 | 97 | 70 | **84** | Great IMU, but 25ms sync — needs deskew |
| NTU VIRAL | 60 | 100 | **80** | IMU too noisy for LIO, use LiDAR-only |
| Ouster OS0-32 | 80 | 74 | **77** | 50Hz IMU too slow, add external IMU |

## Links

- [Documentation](https://rsasaki0109.github.io/bagx/)
- [PyPI](https://pypi.org/project/bagx/)
- [Changelog](https://github.com/rsasaki0109/bagx/releases)
