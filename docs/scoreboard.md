# Public Dataset Scoreboard

bagx scores public rosbag datasets with a single command (`bagx eval`). This page is
**generated from** [`benchmarks/scoreboard.json`](https://github.com/rsasaki0109/bagx/blob/main/benchmarks/scoreboard.json)
so scores stay reproducible across releases.

Scores are **readiness hints**, not SLAM trajectory accuracy. They summarize sensor rates,
sync, IMU noise, and stack-specific topic coverage (Nav2 / Autoware / MoveIt / perception).

## Analysis articles

- [Why EuRoC MH_01_easy IMU looks noisy](scoreboard/why-euroc-mh01-imu-is-noisy.md)
- [Why Ouster OS0-32 scores 77](scoreboard/why-ouster-os0-32-scores-77.md)
- [Why NTU VIRAL IMU looks noisy](scoreboard/why-ntu-viral-imu-is-noisy.md)
- [Why Autoware all-sensors-bag4 scores 72](scoreboard/why-autoware-all-sensors-scores-72.md)

<!-- SCOREBOARD_TABLE_START -->

**30 public datasets** tracked · **26 scored** · bagx 0.6.0

| Dataset | Domain | ROS | Overall | IMU | Sync | Key finding | bagx |
|---------|--------|-----|---------|-----|------|-------------|------|
| [EuRoC MH_01_easy](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) | SLAM | ros1 | **60.3** | 23.1 | 97.6 | WARNING IMU accel noise 0.7935 m/s² — noisy, LiDAR-only odometry may outperform LIO | 0.6.0 |
| [Hilti SLAM Challenge 2022](https://hilti-challenge.com/dataset-2022.html) | SLAM | ros2 | **—** | — | — | Pending download — construction-site LiDAR+IMU benchmark | — |
| [Livox MID-360](https://www.livoxtech.com/mid-360) | SLAM | ros2 | **83.7** | 97.3 | 70.0 | IMU is excellent but LiDAR↔IMU sync ~25ms — enable deskew | 0.4.0 |
| [M2DGR (urban street)](https://github.com/SJTU-ViSYS/M2DGR) | SLAM | ros1 | **—** | — | — | Pending download (~21GB) — multi-sensor GNSS/IMU/LiDAR urban dataset | — |
| [NTU VIRAL (drone)](https://github.com/ntu-aris/ntu_viral_dataset) | SLAM | ros2 | **79.8** | 59.5 | 100.0 | Noisy IMU — LiDAR-only SLAM (e.g. KISS-ICP) recommended | 0.4.0 |
| [Newer College (handheld)](https://ori-drs.github.io/newer_college_dataset/) | SLAM | ros2 | **92.3** | 84.5 | 100.0 | LiDAR+IMU quality is strong enough for SLAM benchmarking | 0.4.0 |
| [Ouster OS0-32 (static)](https://data.ouster.io/) | SLAM | ros2 | **76.6** | 79.6 | 73.6 | 50Hz IMU is slow for deskew; sync margin is thin | 0.4.0 |
| [TUM VI calib-imu1](https://cvg.cit.tum.de/tumvi/dataset/calibration) | SLAM | ros1 | **95.4** | 90.8 | 100.0 | IMU accel noise 0.0515 m/s² — good for LIO, set imu_acc_noise_density to 0.0515 | 0.6.0 |
| [UrbanLoco (Alameda)](https://github.com/weisongwen/UrbanLoco) | SLAM | ros1 | **—** | — | — | Pending download (multi-GB) — long urban GNSS/IMU/LiDAR sequence | — |
| [UrbanNav HK TST](https://github.com/IPNL-POLYU/UrbanNavDataset) | SLAM | ros1 | **—** | — | — | Pending download (~35GB) — urban canyon GNSS + LiDAR fusion bag | — |
| [AutoCore Ouster OS1-64](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **100.0** | — | — | LiDAR (/sensing/lidar/top/rectified/pointcloud) at 10Hz | 0.6.0 |
| [Autoware all-sensors-bag1](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **90.2** | 85.8 | 75.1 | GNSS fix rate 100% — suitable as ground truth reference | 0.6.0 |
| [Autoware all-sensors-bag2](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **86.1** | 69.8 | 74.6 | GNSS fix rate 100% — suitable as ground truth reference | 0.6.0 |
| [Autoware all-sensors-bag3](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **81.7** | 78.7 | 59.0 | GNSS fix rate 100% — suitable as ground truth reference | 0.6.0 |
| [Autoware all-sensors-bag4](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **72.2** | — | — | Sensing/localization-only bag — planning/control topics absent | 0.4.0 |
| [Autoware all-sensors-bag5](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **84.5** | 77.9 | 64.5 | GNSS fix rate 100% — suitable as ground truth reference | 0.6.0 |
| [Autoware all-sensors-bag6](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **77.0** | 40.3 | 69.9 | GNSS fix rate 100% — suitable as ground truth reference | 0.6.0 |
| [Autoware driving_20_kmh](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **99.0** | — | — | LiDAR packets and /vehicle/status/velocity_status at healthy rates | 0.4.0 |
| [Autoware driving_30_kmh](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) | Autoware | ros2 | **98.9** | — | 97.9 | Sensor sync good (8.4ms) | 0.6.0 |
| [Nav2 Gazebo (deep capture)](https://github.com/rsasaki0109/bagx) | Nav2 | ros2 | **100.0** | 100.0 | — | Pipeline scan → costmap: 43ms median, 82ms P95 | 0.6.0 |
| [Nav2 Gazebo (goal capture)](https://github.com/rsasaki0109/bagx) | Nav2 | ros2 | **88.6** | 100.0 | 77.1 | IMU accel noise 0.0170 m/s² — excellent, set imu_acc_noise_density to 0.0170 | 0.6.0 |
| [Nav2 Gazebo (headless)](https://github.com/rsasaki0109/bagx) | Nav2 | ros2 | **100.0** | 100.0 | — | Control command (/cmd_vel) at 10Hz — good for control-loop observability | 0.6.0 |
| [TurtleBot3 Walker](https://huggingface.co/datasets/okritvik/TurtleBot3_Walker) | Nav2 | ros2 | **100.0** | 100.0 | — | Pipeline scan → costmap: 43ms median, 82ms P95 | 0.6.0 |
| [Franka Panda (MoveIt)](https://huggingface.co/datasets/adriankobras/panda) | MoveIt | ros2 | **100.0** | — | — | Pipeline joint_states → planned_path: 6ms median, 6ms P95 (1 sample) | 0.6.0 |
| [MoveIt execution capture](https://github.com/rsasaki0109/bagx) | MoveIt | ros2 | **100.0** | — | — | Pipeline joint_states → planned_path: 6ms median, 6ms P95 (1 sample) | 0.6.0 |
| [MoveIt plan-only capture](https://github.com/rsasaki0109/bagx) | MoveIt | ros2 | **66.7** | — | — | Pipeline joint_states → planned_path: 9ms median, 9ms P95 (1 sample) | 0.6.0 |
| [NVIDIA r2b_galileo](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim) | Perception | ros2 | **90.5** | — | — | Eight camera streams + IMU + chassis odom time-aligned | 0.4.0 |
| [NVIDIA r2b_galileo2](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim) | Perception | ros2 | **95.3** | — | — | RGB-D + infra + camera_info — reusable for perception export | 0.4.0 |
| [NVIDIA r2b_whitetunnel](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim) | Perception | ros2 | **91.3** | — | — | Multi-camera + IMU; noisy accel flagged but perception coverage solid | 0.4.0 |
| [NVIDIA r2b_robotarm](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim) | Manipulation | ros2 | **96.0** | — | — | RGB-D + joint_states suitable for arm perception benchmarking | 0.4.0 |

<!-- SCOREBOARD_TABLE_END -->

## Reproduce a row

Each manifest entry includes a `reproduce` command. Example:

```bash
export BAGX_SCOREBOARD_BAGS=/path/to/downloaded/bags
# also accepts BAGX_REALBAGS for NVIDIA / Autoware mirrors
bagx eval "$BAGX_SCOREBOARD_BAGS/r2b_galileo2"
```

Refresh the table after downloading bags:

```bash
export BAGX_SCOREBOARD_BAGS=/path/to/bags
python scripts/generate_scoreboard.py --refresh --write-manifest
python scripts/generate_scoreboard.py
```

Manifest: [`benchmarks/scoreboard.json`](https://github.com/rsasaki0109/bagx/blob/main/benchmarks/scoreboard.json)

### Sample commands by domain

**SLAM**

- Newer College (handheld): `bagx eval newer_college.db3`
- Livox MID-360: `bagx eval livox_mid360.db3`

**Autoware**

- Autoware all-sensors-bag4: `aws s3 sync s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags/all-sensors-bag4_compressed/ ./all-sensors-bag4_compressed --no-sign-request && bagx eval all-sensors-bag4_compressed`
- Autoware driving_20_kmh: `aws s3 sync s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags/driving_20_kmh_2022_06_10-16_01_55_compressed/ ./driving_20 --no-sign-request && bagx eval driving_20`

**Nav2**

- TurtleBot3 Walker: `bagx eval turtlebot3_walker`
- Nav2 Gazebo (deep capture): `python scripts/run_ros_dogfood.py nav2-gazebo && bagx eval <captured_bag>`

**MoveIt**

- Franka Panda (MoveIt): `bagx eval panda`
- MoveIt execution capture: `python scripts/run_ros_dogfood.py moveit-demo && bagx eval <captured_bag>`

**Perception**

- NVIDIA r2b_galileo: `bagx eval r2b_galileo`
- NVIDIA r2b_whitetunnel: `bagx eval r2b_whitetunnel`

**Manipulation**

- NVIDIA r2b_robotarm: `bagx eval r2b_robotarm`
