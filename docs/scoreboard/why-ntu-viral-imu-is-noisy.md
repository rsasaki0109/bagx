# Why NTU VIRAL IMU looks noisy

**Overall: 84.0/100** · Domain: SLAM · ROS 1 · Drone platform · sequence `eee_03` · bagx 0.6.0 + unreleased sync fix

## The headline number

NTU VIRAL is a **multi-sensor drone dataset**: two Ouster OS1 LiDARs, a stereo pair, a
VN100 IMU, UWB ranging, DJI flight-controller GNSS/IMU, and Leica laser-tracker ground
truth. On `eee_03` (181 s, 19 topics, 389,804 messages) the time sync is clean and GNSS
is present, so the score is pulled down by the **IMU at 72.0**.

| Component | Score | Why |
|-----------|-------|-----|
| GNSS | 80.0 | `/dji_sdk/gps_position` has a 100% fix rate; it carries no usable covariance, so the HDOP half of the score is a neutral 50 |
| IMU (`/imu/imu`) | 72.0 | 389 Hz; accel noise 0.16 / 0.37 / 0.35 m/s² (x/y/z), gyro 0.011 / 0.011 / 0.003 rad/s |
| Sync | 100.0 | Every checked pair (DJI GNSS/IMU, VN100 IMU, left camera, OS1 node1 IMU/points) is within 2.4 ms mean |
| **Overall** | **84.0** | (80.0 + 72.0 + 100.0) / 3 |

## What drives the low IMU sub-score

bagx estimates IMU noise via `std(diff(x))/√2`. On a flying platform that number includes
real high-frequency motion and airframe vibration, not only sensor noise, so it is an upper
bound. The accel average (0.30 m/s²) is above the threshold bagx uses for tightly coupled
LIO, while the gyro (0.0085 rad/s) is fine.

bagx therefore recommends **LiDAR-only or LiDAR-forward odometry** (e.g. KISS-ICP), or
LIO with a deliberately inflated accelerometer noise and some IMU pre-filtering, rather
than default LIO weights. The bag is not static, so Allan variance is not computed; take
noise densities from the dataset's `imu_v100.yaml` calibration, not from this flight.

## Why sync scores 100

bagx pairs each message of the sparser topic with the nearest message of the denser one.
Every LiDAR scan and camera frame has an IMU sample within a few milliseconds, which is
what LiDAR–IMU and visual–inertial pipelines need.

## Practical takeaway

| Goal | Recommendation |
|------|----------------|
| LiDAR odometry benchmark | Good candidate — sync is clean |
| LIO with default IMU weights | Expect tuning; inflate accel noise or pre-filter |
| VIO with raw IMU | Use the provided IMU calibration, not diff-based noise from flight |

## Reproduce

```bash
curl -L -o eee_03.zip https://researchdata.ntu.edu.sg/api/access/datafile/68132   # 2.0 GB, DR-NTU, CC BY-NC 4.0
unzip eee_03.zip
pip install bagx[ros1]
bagx eval eee_03/eee_03.bag
```

Compare with the [Livox MID-360 row](../scoreboard.md) (98.7 overall) for a ground-vehicle
LiDAR+IMU bag with a quiet IMU.
