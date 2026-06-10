#!/usr/bin/env python3
"""Generate the synthetic sample bag bundled with the browser demo.

Reuses the CDR builders from tests/conftest.py to create a small,
deterministic handheld-SLAM-flavored .db3 with a mix of good and
warning findings:

- /imu        sensor_msgs/msg/Imu      100 Hz (low for LIO -> warning)
- /gnss/fix   sensor_msgs/msg/NavSatFix  5 Hz with a degraded patch
- /odom       nav_msgs/msg/Odometry     30 Hz
- /tf         tf2_msgs/msg/TFMessage    30 Hz

Usage:
    python3 scripts/generate_demo_bag.py [output.db3]
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "demo" / "sample_bag.db3"

spec = importlib.util.spec_from_file_location(
    "bagx_test_conftest", REPO_ROOT / "tests" / "conftest.py"
)
conftest = importlib.util.module_from_spec(spec)
spec.loader.exec_module(conftest)

START_SEC = 1_700_000_000
DURATION_S = 8.0


def _stamp(t: float) -> tuple[int, int, int]:
    """Return (sec, nanosec, timestamp_ns) for offset t seconds."""
    ns = int((START_SEC + t) * 1e9)
    return ns // 1_000_000_000, ns % 1_000_000_000, ns


def main() -> None:
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_OUTPUT
    output.parent.mkdir(parents=True, exist_ok=True)
    output.unlink(missing_ok=True)
    for suffix in ("-shm", "-wal"):
        Path(str(output) + suffix).unlink(missing_ok=True)

    rng = np.random.default_rng(42)
    messages = []

    # IMU: 100 Hz, static (accel ~ 9.81) with realistic noise levels.
    imu_rate = 100
    for i in range(int(DURATION_S * imu_rate)):
        sec, nanosec, ts = _stamp(i / imu_rate)
        accel = (
            float(rng.normal(0.0, 0.05)),
            float(rng.normal(0.0, 0.05)),
            float(9.81 + rng.normal(0.0, 0.05)),
        )
        gyro = tuple(float(rng.normal(0.0, 0.002)) for _ in range(3))
        messages.append(
            {
                "topic": "/imu",
                "timestamp_ns": ts,
                "data": conftest.build_imu_cdr(sec, nanosec, accel=accel, gyro=gyro),
            }
        )

    # GNSS: 5 Hz, good fix except a degraded patch in the middle.
    gnss_rate = 5
    n_gnss = int(DURATION_S * gnss_rate)
    for i in range(n_gnss):
        sec, nanosec, ts = _stamp(i / gnss_rate)
        in_patch = n_gnss * 2 // 5 <= i < n_gnss * 3 // 5
        status = -1 if in_patch else 0
        hdop_sq = 9.0 if in_patch else 1.2
        messages.append(
            {
                "topic": "/gnss/fix",
                "timestamp_ns": ts,
                "data": conftest.build_navsatfix_cdr(
                    sec,
                    nanosec,
                    status=status,
                    latitude=35.6812 + i * 1e-6,
                    longitude=139.7671 + i * 1e-6,
                    altitude=40.0,
                    hdop_squared=hdop_sq,
                ),
            }
        )

    # Odometry + TF: 30 Hz, slow forward motion.
    odom_rate = 30
    for i in range(int(DURATION_S * odom_rate)):
        t = i / odom_rate
        sec, nanosec, ts = _stamp(t)
        position = (0.5 * t, 0.0, 0.0)
        orientation = (0.0, 0.0, 0.0, 1.0)
        messages.append(
            {
                "topic": "/odom",
                "timestamp_ns": ts,
                "data": conftest.build_odometry_cdr(
                    sec, nanosec, position, orientation, (0.5, 0.0, 0.0), (0.0, 0.0, 0.0)
                ),
            }
        )
        messages.append(
            {
                "topic": "/tf",
                "timestamp_ns": ts,
                "data": conftest.build_tf_message_cdr(
                    sec, nanosec, position, orientation, "odom", "base_link"
                ),
            }
        )

    topics = [
        {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        {"name": "/gnss/fix", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/odom", "type": "nav_msgs/msg/Odometry", "format": "cdr"},
        {"name": "/tf", "type": "tf2_msgs/msg/TFMessage", "format": "cdr"},
    ]
    conftest._create_db3(output, topics, messages)
    size_kb = output.stat().st_size / 1024
    print(f"wrote {output} ({len(messages)} msgs, {size_kb:.0f} KiB)")


if __name__ == "__main__":
    main()
