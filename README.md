# bagx

[![PyPI](https://img.shields.io/pypi/v/bagx)](https://pypi.org/project/bagx/)
[![CI](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/bagx/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://rsasaki0109.github.io/bagx/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)

**One command to check if your rosbag data is ready — for SLAM, Nav2, Autoware, or MoveIt.**

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
- **Framework-aware checks** for Nav2, Autoware, MoveIt, and RGB-D robot arms
- **Manifest-driven benchmark suites** you can rerun across public and private bags

## The problem

You recorded a rosbag. You run SLAM. It diverges. Now you spend hours asking:

- *"Is my IMU noise too high for LIO?"* → `bagx eval` tells you
- *"What noise value should I set in the SLAM config?"* → bagx gives you the exact number
- *"Are my LiDAR and IMU actually synchronized?"* → `bagx sync` shows the delay
- *"Where did the data quality drop?"* → `bagx anomaly` pinpoints the exact timestamp
- *"Which of my 10 bags is the best for benchmarking?"* → `bagx batch eval` ranks them all
- *"Is my odometry fast enough for Nav2?"* → bagx checks rate and warns if too slow
- *"Are all Autoware sensing topics alive?"* → bagx verifies camera, LiDAR, GNSS rates

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
| `bagx eval bag.db3` | Is this bag ready? Auto-detects SLAM / Nav2 / Autoware / MoveIt |
| `bagx compare A.db3 B.db3` | Which sensor config is better? |
| `bagx sync bag.db3 /imu /lidar` | Are my sensors synchronized? |
| `bagx anomaly bag.db3` | Where did sensor quality drop? |
| `bagx scenario bag.db3` | Find GNSS-lost and high-dynamics segments |
| `bagx export bag.db3 --ai` | Export to Parquet/JSON for ML |
| `bagx batch eval *.db3 --csv` | Rank an entire dataset |
| `bagx ask bag.db3 "question"` | Ask questions via LLM |
| `bagx benchmark suite.json` | Re-run a curated benchmark suite and fail CI on regressions |

## Tested on public datasets

| Dataset | IMU | Sync | Score | Insight |
|---------|-----|------|-------|---------|
| Newer College | 84 | 100 | **92** | Best data, ideal for SLAM benchmarking |
| Livox MID-360 | 97 | 70 | **84** | Great IMU, but 25ms sync — needs deskew |
| NTU VIRAL | 60 | 100 | **80** | IMU too noisy for LIO, use LiDAR-only |
| Ouster OS0-32 | 80 | 74 | **77** | 50Hz IMU too slow, add external IMU |

## Auto-detects your framework

bagx recognizes topic patterns and gives framework-specific advice:

Namespaced topics are supported too, so real bags like `/robot/odom`,
`/sensing/lidar/top/pointcloud_raw_ex`, or `/move_group/display_planned_path`
are detected without renaming topics first.

```
$ bagx eval nav2_robot.db3
Nav2 topics detected
  ✔ Odometry (/robot/odom) at 50Hz — good for Nav2
  ✔ LaserScan (/robot/scan) at 12Hz — good for costmap
  ✔ Global plan (/plan) recorded 3 times — planner output is visible
  ✔ NavigateToPose status (/navigate_to_pose/_action/status) recorded
  ✔ Pipeline scan → cmd_vel (full loop): 41ms median, 81ms P95
  ✔ Pipeline plan → cmd_vel onset: 11ms median, 76ms P95 (3 samples)

$ bagx eval autoware_vehicle.db3
Autoware topics detected
  ✔ LiDAR (/sensing/lidar/top/pointcloud_raw_ex) at 10Hz
  ✔ Camera (/sensing/camera/front/image_raw/compressed) at 30Hz
  ✔ GNSS (/sensing/gnss/ublox/nav_sat_fix)
  ℹ Sensing/localization-only Autoware bag — skipping planning/control checks

$ bagx eval moveit_arm.db3
MoveIt topics detected
  ✔ JointState (/joint_states) at 115Hz — good for motion planning
  ✔ MoveGroup action activity recorded on /move_action/_action/status
  ✔ Joint trajectory controller activity recorded on /panda_arm_controller/follow_joint_trajectory/_action/status
  ✔ Pipeline joint_states → planned_path: 6ms median, 6ms P95 (1 sample)
  ✔ Pipeline planned_path → arm execution: 5ms median, 5ms P95 (1 sample)
```

## Dogfooding

bagx is dogfooded against:

- Autoware real bags, including the official `all-sensors-bag4` dataset from the Autoware documentation datasets page
- Nav2 simulator bags captured with `scripts/run_ros_dogfood.py nav2-gazebo`
- MoveIt simulator bags captured with `scripts/run_ros_dogfood.py moveit-demo`

Recent dogfood runs:

- `nav2-deep-final-20260324-185315`: `Overall 100.0/100`, `LaserScan 13Hz`, `plan → cmd_vel onset 11ms median`
- `moveit-exec-final-20260324-185315`: `Overall 100.0/100`, `JointState 115Hz`, `planned_path → arm execution 5ms median`
- `autoware_isuzu_all_sensors_bag4`: real bag, `Overall 72.2/100`, plus a sensing-only note when planning/control topics are absent
- `driving_20_kmh_2022_06_10-16_01_55_compressed`: official Autoware open-data bag, `Overall 99.0/100`, LiDAR packet sync and `/vehicle/status/velocity_status` captured
- `r2b_robotarm`: official NVIDIA open-data MCAP, `Overall 96.0/100`, RGB-D + joint-state manipulation perception checks

## Benchmark suites

`bagx` now supports manifest-driven benchmark suites so public bags can become repeatable regression tests instead of one-off dogfood runs.

The repository includes [benchmarks/open_data_suite.json](/workspace/ai_coding_ws/bagx/benchmarks/open_data_suite.json), which covers:

- Autoware official `all-sensors-bag4`
- Autoware official `driving_20_kmh_2022_06_10-16_01_55_compressed`
- NVIDIA official `r2b_robotarm`

Typical usage:

```bash
export BAGX_REALBAGS=/tmp/bagx_realbags
bagx benchmark benchmarks/open_data_suite.json
bagx benchmark benchmarks/open_data_suite.json --json benchmark-report.json
```

JSON outputs now include `schema_version`, `report_type`, and `bagx_version`, so they are easier to gate in CI and compare across releases.

Typical local workflow:

```bash
# Assumes the ROS overlay is already installed at .cache/ros_overlay_nav2_test
python3 scripts/run_ros_dogfood.py nav2-gazebo \
  --duration 40 \
  --record-dir .cache/dogfood/nav2-deep-final
bagx eval .cache/dogfood/nav2-deep-final

python3 scripts/run_ros_dogfood.py moveit-demo \
  --duration 40 \
  --record-dir .cache/dogfood/moveit-exec-final
bagx eval .cache/dogfood/moveit-exec-final
```

## Links

- [Documentation](https://rsasaki0109.github.io/bagx/)
- [PyPI](https://pypi.org/project/bagx/)
- [Changelog](https://github.com/rsasaki0109/bagx/releases)
