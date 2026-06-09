# Why Autoware all-sensors-bag4 scores 72

**Overall: 72.2/100** · Domain: Autoware · ROS 2

## This is not a "bad bag"

The Leo Drive Isuzu **all-sensors-bag4** is an official Autoware open-data recording with
healthy sensing topics. The score reflects **what is in the file**, not vehicle performance.

## Why overall ≠ 99 like driving_20

`driving_20_kmh` scores **99.0** because it includes full-stack signals bagx checks for
Autoware readiness:

- Vehicle velocity status at high rate
- LiDAR packet streams with sync checks
- Planning / control visibility where present

`all-sensors-bag4` is primarily a **sensing and localization bundle**. bagx detects Autoware
and scores domain coverage, but many planning/control topics are absent. The eval report
notes sensing/localization-only context — that caps the domain sub-score.

## How bagx computes this row

| Component | all-sensors-bag4 | driving_20 |
|-----------|------------------|------------|
| Domain (Autoware) | ~72 — partial stack | ~99 — fuller stack |
| GNSS / LiDAR / camera | Present | Present + vehicle status |
| Planning/control | Mostly missing | Recorded |

Overall is the mean of applicable sub-scores (GNSS, IMU, sync when relevant, domain).

## When to use each bag

- **all-sensors-bag4** — Calibrate perception/localization pipelines, sensor bring-up, bagx
  regression on Autoware *sensing* detection.
- **driving_20** — End-to-end Autoware readiness gate before integration tests.

## Reproduce

```bash
aws s3 sync s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags/all-sensors-bag4_compressed/ ./all-sensors-bag4_compressed --no-sign-request
bagx eval all-sensors-bag4_compressed
```

See also [driving_20 on the scoreboard](../scoreboard.md).
