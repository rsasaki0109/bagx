# Why EuRoC MH_01_easy IMU looks noisy

**Overall: 60.3/100** · Domain: SLAM · ROS 1 · MAV handheld sequence

## The headline number

EuRoC is a classic visual–inertial benchmark, but **MH_01_easy** scores only **60.3** overall
because IMU quality lands at **23.1** while sync remains strong at **97.6**.

## What drives the low IMU sub-score

bagx estimates IMU noise via `std(diff(x))/√2` on accelerometer and gyro streams. On this bag:

- Accelerometer noise is flagged at **0.79 m/s²** — far above SLAM-oriented thresholds.
- The platform is a flying MAV with aggressive motion; diff-based noise includes real dynamics.
- EuRoC IMU units are older MEMS hardware; many LIO papers apply extra filtering anyway.

bagx therefore recommends **LiDAR-only or LiDAR-forward odometry** rather than tight LIO with
default IMU weights.

## Why sync still scores well

LiDAR and IMU timestamps are reasonably aligned (97.6 sync score). The recording is **usable**;
the warning is about **fusion strategy and IMU preprocessing**, not a broken bag.

## Practical takeaway

| Goal | Recommendation |
|------|----------------|
| VIO / LIO benchmark | Expect IMU tuning; compare against NTU VIRAL pattern |
| LiDAR odometry only | Reasonable candidate — sync is clean |
| Default LIO noise model | Not ideal without recalibration or denoising |

## Reproduce

```bash
pip install bagx[ros1]
export BAGX_SCOREBOARD_BAGS=/path/to/bags
bagx eval MH_01_easy.bag
```

Compare with [TUM VI calib-imu1](../scoreboard.md) (95.4 overall) for a calibration-grade reference.
