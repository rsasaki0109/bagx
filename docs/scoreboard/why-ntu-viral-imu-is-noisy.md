# Why NTU VIRAL IMU looks noisy

**Overall: 79.8/100** · Domain: SLAM · ROS 2 · Drone platform

## The headline number

NTU VIRAL is a strong **multi-sensor drone dataset** with excellent time sync (100.0 sync score)
and usable GNSS (80.0). The overall score is pulled down by **IMU quality at 59.5**.

## What drives the low IMU sub-score

bagx estimates IMU noise via `std(diff(x))/√2` on accelerometer and gyro streams. On NTU VIRAL:

- Accelerometer and gyro noise exceed SLAM-oriented thresholds.
- High-frequency vibration and aggressive flight dynamics inflate diff-based noise estimates.
- The platform is not a static calibration rig — "noise" includes real motion content.

bagx therefore recommends **LiDAR-only or LiDAR-forward SLAM** (e.g. KISS-ICP) rather than
tight LiDAR–IMU fusion without extra filtering or IMU denoising.

## Why sync still scores 100

Sensor timestamps are well aligned across LiDAR, cameras, and IMU. The bag is **usable**;
the warning is about **fusion strategy**, not broken recordings.

## Practical takeaway

| Goal | Recommendation |
|------|----------------|
| LiDAR odometry benchmark | Good candidate — sync is clean |
| LIO with default IMU weights | Expect tuning; check IMU preprocessing |
| VIO with raw IMU | Not ideal without denoising / different noise model |

## Reproduce

```bash
bagx eval ntu_viral.db3
```

Compare with [Newer College](../scoreboard.md) (92.3 overall) for a handheld reference bag.
