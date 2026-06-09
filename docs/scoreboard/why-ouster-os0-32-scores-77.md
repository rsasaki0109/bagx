# Why Ouster OS0-32 scores 77

**Overall: 76.6/100** · Domain: SLAM · ROS 2

## What bagx sees

The Ouster OS0-32 static capture is a useful teaching example: the LiDAR data is fine,
but the **IMU lane is marginal for LiDAR deskew**.

| Metric | Value | bagx interpretation |
|--------|-------|---------------------|
| IMU score | 79.6 | Acceptable noise, but not excellent |
| IMU rate | ~50 Hz | Below the 100–200 Hz sweet spot for deskew |
| Sync score | 73.6 | LiDAR↔IMU delay leaves little margin |
| Overall | 76.6 | Mean of IMU + sync (no GNSS in this bag) |

## Why the score is not higher

1. **IMU frequency** — At ~50 Hz, point-wise deskew competes with motion blur on faster rotations.
   bagx flags this relative to SLAM-oriented thresholds used for handheld LiDAR+IMU bags.
2. **Sync** — A ~25 ms class delay between LiDAR and IMU erodes the sync sub-score even when
   timestamps look consistent.
3. **Short static segment** — Allan bias metrics are limited on short recordings; bagx notes
   that longer static captures improve IMU characterization.

## What to do before running SLAM

```bash
bagx eval ouster_os0-32.db3 --tune fast_lio
```

Typical mitigations:

- Increase IMU rate (hardware or driver config) if possible.
- Enable per-point deskew and verify sync with `bagx sync`.
- Record a longer static segment for IMU noise / bias estimation.

## Reproduce

```bash
bagx eval ouster_os0-32.db3
```

See the [scoreboard](../scoreboard.md) for the full dataset list.
