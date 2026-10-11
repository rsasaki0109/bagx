# Why Autoware all-sensors-bag4 scores 78

**Overall: 77.9/100** · Domain: Autoware · ROS 2 · bagx 0.6.0

## This is not a "bad bag"

The Leo Drive Isuzu **all-sensors-bag4** is an official Autoware open-data recording
(17.8 s, 17 topics, 9,489 messages). The Autoware domain checks pass almost entirely. What
pulls the overall score down is the **IMU and sync sub-scores**, not missing topics.

## Score breakdown

Overall is the mean of the sub-scores that apply to the bag:

| Component | Score | Why |
|-----------|-------|-----|
| GNSS | 99.9 | `/gnss/fix` has a 100% fix rate |
| Domain (Autoware) | 95.0 | LiDAR at 9–10 Hz ✔, GNSS ✔; the three `lucid_vision` cameras run at 14 Hz, just under the 15 Hz perception target |
| Sync | 67.7 | Camera↔camera and GNSS↔camera pairs show ~24–31 ms mean and up to ~200 ms max offset |
| IMU | 48.8 | Accel noise 0.43 m/s² (x axis 0.57), gyro y 0.048 rad/s |
| **Overall** | **77.9** | (99.9 + 95.0 + 67.7 + 48.8) / 4 |

## Reading the IMU number

bagx estimates noise as `std(diff(x)) / √2`. On a **moving vehicle** that number includes
real vibration and road motion, not only sensor noise, so it is an upper bound. The bag is
also too short (<60 s) and not static, so bagx cannot run Allan variance and says so. Treat
the IMU warning as "do not take noise densities from this bag", not "the IMU is broken".

## Reading the sync number

The cameras are not hardware-triggered against each other or against GNSS. bagx pairs each
message with the nearest message on the other topic, so 14 Hz cameras with free-running
phase produce tens of milliseconds of offset by construction. For multi-camera perception
that is worth knowing; for LiDAR-only localization it does not matter
(`/sensing/imu/imu_data` ↔ concatenated pointcloud is 25 ms mean, 81 ms max).

## Planning / control

bag4 is a sensing/localization bundle. bagx notes that planning/control topics are absent
and **skips** those checks instead of penalizing them, which is why the domain score stays
at 95.

## When to use this bag

- Sensor bring-up and LiDAR/GNSS localization regression tests
- bagx regression on Autoware *sensing* detection
- Not as a source of IMU noise parameters, and not as an end-to-end Autoware gate (use
  `driving_20_kmh`, 99.0, for that)

## Reproduce

```bash
aws s3 sync s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags/all-sensors-bag4_compressed/ ./all-sensors-bag4_compressed --no-sign-request
bagx eval all-sensors-bag4_compressed
```

See also [the scoreboard](../scoreboard.md).
