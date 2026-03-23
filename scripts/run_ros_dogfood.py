#!/usr/bin/env python3
"""Launch ROS overlay demos and optionally record dogfood bags."""

from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OVERLAY = REPO_ROOT / ".cache/ros_overlay_nav2_test"


@dataclass(frozen=True)
class Scenario:
    launch_cmd: list[str]
    topics: tuple[str, ...]
    needs_xvfb: bool = False
    exercise_cmds: tuple[str, ...] = ()
    startup_delay: float = 5.0
    record_warmup_delay: float = 0.0
    include_hidden_topics: bool = False
    record_qos_overrides: dict[str, dict[str, str | int]] = field(default_factory=dict)


SCENARIOS = {
    "nav2-loopback": Scenario(
        launch_cmd=[
            "ros2",
            "launch",
            "nav2_loopback_sim",
            "loopback_simulation.launch.py",
            "scan_frame_id:=base_footprint",
        ],
        topics=(
            "/clock",
            "/cmd_vel",
            "/initialpose",
            "/odom",
            "/scan",
            "/tf",
            "/tf_static",
        ),
        exercise_cmds=(
            (
                "timeout 3s ros2 topic pub -r 2 /initialpose "
                "geometry_msgs/msg/PoseWithCovarianceStamped "
                "\"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
                "orientation: {z: 0.0, w: 1.0}}}}\""
            ),
            (
                "timeout 4s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "
                "\"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}\""
            ),
        ),
        startup_delay=8.0,
    ),
    "moveit-demo": Scenario(
        launch_cmd=[
            "ros2",
            "launch",
            "moveit_resources_panda_moveit_config",
            "demo.launch.py",
        ],
        topics=(
            "/display_planned_path",
            "/joint_states",
            "/monitored_planning_scene",
            "/move_action/_action/feedback",
            "/move_action/_action/status",
            "/execute_trajectory/_action/feedback",
            "/execute_trajectory/_action/status",
            "/panda_arm_controller/follow_joint_trajectory/_action/feedback",
            "/panda_arm_controller/follow_joint_trajectory/_action/status",
            "/planning_scene",
            "/planning_scene_world",
            "/tf",
            "/tf_static",
        ),
        needs_xvfb=True,
        exercise_cmds=(
            "python3 scripts/trigger_moveit_plan.py",
        ),
        startup_delay=0.0,
        record_warmup_delay=5.0,
        include_hidden_topics=True,
        record_qos_overrides={
            "/display_planned_path": {
                "history": "keep_last",
                "depth": 10,
                "reliability": "reliable",
                "durability": "volatile",
            },
        },
    ),
    "nav2-gazebo": Scenario(
        launch_cmd=[
            "ros2",
            "launch",
            str(REPO_ROOT / "scripts/nav2_headless/tb3_headless_launch.py"),
            "use_rviz:=False",
            "headless:=True",
            (
                "robot_sdf:="
                f"{REPO_ROOT / 'scripts/nav2_headless/gz_waffle_headless_gpu_lidar.sdf.xacro'}"
            ),
        ],
        topics=(
            "/amcl_pose",
            "/clock",
            "/cmd_vel",
            "/global_costmap/costmap",
            "/initialpose",
            "/imu",
            "/joint_states",
            "/local_costmap/costmap",
            "/odom",
            "/plan",
            "/navigate_to_pose/_action/feedback",
            "/navigate_to_pose/_action/status",
            "/scan",
            "/tf",
            "/tf_static",
        ),
        needs_xvfb=False,
        exercise_cmds=(
            (
                "timeout 3s ros2 topic pub -r 2 /initialpose "
                "geometry_msgs/msg/PoseWithCovarianceStamped "
                "\"{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, "
                "orientation: {z: 0.0, w: 1.0}}}}\""
            ),
            "python3 scripts/trigger_nav2_goal.py --x -1.0 --y -0.5",
        ),
        startup_delay=10.0,
        include_hidden_topics=True,
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "scenario",
        choices=sorted(SCENARIOS),
        help="Scenario to launch and optionally record.",
    )
    parser.add_argument(
        "--overlay-root",
        type=Path,
        default=DEFAULT_OVERLAY,
        help=f"Overlay root directory (default: {DEFAULT_OVERLAY})",
    )
    parser.add_argument(
        "--record-dir",
        type=Path,
        help="Output directory for rosbag2 recording.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="How long to keep the scenario running in seconds (default: %(default)s).",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=None,
        help="Delay before exercise / recording begins in seconds (default: scenario-specific).",
    )
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="Extra ROS launch arg, e.g. namespace:=robot.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Extra topic to record. Defaults are scenario-specific.",
    )
    parser.add_argument(
        "--no-exercise",
        action="store_true",
        help="Disable scenario-specific exercise commands.",
    )
    return parser.parse_args()


def run_shell(command: str) -> list[str]:
    return ["bash", "-lc", command]


def quoted(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def launch_wrapper(overlay_root: Path, needs_xvfb: bool, command: list[str]) -> list[str]:
    if needs_xvfb:
        helper = overlay_root / "run_with_xvfb.bash"
        return [str(helper), *command]

    activate = overlay_root / "activate.bash"
    return run_shell(f"source {shlex.quote(str(activate))} >/dev/null && exec {quoted(command)}")


def write_qos_overrides(path: Path, overrides: dict[str, dict[str, str | int]]) -> None:
    lines: list[str] = []
    for topic, profile in overrides.items():
        lines.append(f"{topic}:")
        for key, value in profile.items():
            lines.append(f"  {key}: {value}")
    path.write_text("\n".join(lines) + "\n")


def record_command(
    overlay_root: Path,
    record_dir: Path,
    topics: list[str],
    *,
    include_hidden_topics: bool = False,
    qos_overrides_path: Path | None = None,
) -> list[str]:
    activate = overlay_root / "activate.bash"
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(record_dir),
    ]
    if include_hidden_topics:
        cmd.append("--include-hidden-topics")
    if qos_overrides_path is not None:
        cmd.extend(["--qos-profile-overrides-path", str(qos_overrides_path)])
    cmd.extend(["--topics", *topics])
    return run_shell(f"source {shlex.quote(str(activate))} >/dev/null && exec {quoted(cmd)}")


def exercise_commands(overlay_root: Path, scenario: Scenario) -> list[list[str]]:
    activate = overlay_root / "activate.bash"
    commands: list[list[str]] = []
    for command in scenario.exercise_cmds:
        commands.append(
            run_shell(f"source {shlex.quote(str(activate))} >/dev/null && {command}")
        )
    return commands


def terminate(process: subprocess.Popen[str], label: str) -> None:
    if process.poll() is not None:
        return
    print(f"stopping {label}", file=sys.stderr)
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main() -> int:
    args = parse_args()
    overlay_root = args.overlay_root.resolve()
    activate = overlay_root / "activate.bash"
    if not activate.exists():
        raise SystemExit(f"overlay activate script not found: {activate}")

    scenario = SCENARIOS[args.scenario]
    startup_delay = args.startup_delay
    if startup_delay is None:
        startup_delay = scenario.startup_delay
    launch_cmd = scenario.launch_cmd + args.launch_arg
    launch_proc = subprocess.Popen(
        launch_wrapper(overlay_root, scenario.needs_xvfb, launch_cmd),
        cwd=REPO_ROOT,
        start_new_session=True,
        text=True,
    )

    record_proc: subprocess.Popen[str] | None = None
    exercise_procs: list[subprocess.Popen[str]] = []
    return_code = 0
    try:
        time.sleep(startup_delay)

        if args.record_dir:
            topics = list(dict.fromkeys([*scenario.topics, *args.topic]))
            args.record_dir.parent.mkdir(parents=True, exist_ok=True)
            qos_overrides_path: Path | None = None
            if scenario.record_qos_overrides:
                qos_overrides_path = args.record_dir.parent / f".{args.record_dir.name}.qos.yaml"
                write_qos_overrides(qos_overrides_path, scenario.record_qos_overrides)
            record_proc = subprocess.Popen(
                record_command(
                    overlay_root,
                    args.record_dir.resolve(),
                    topics,
                    include_hidden_topics=scenario.include_hidden_topics,
                    qos_overrides_path=qos_overrides_path.resolve() if qos_overrides_path else None,
                ),
                cwd=REPO_ROOT,
                start_new_session=True,
                text=True,
            )
            if scenario.record_warmup_delay > 0:
                time.sleep(scenario.record_warmup_delay)

        if not args.no_exercise:
            for command in exercise_commands(overlay_root, scenario):
                exercise_procs.append(
                    subprocess.Popen(
                        command,
                        cwd=REPO_ROOT,
                        start_new_session=True,
                        text=True,
                    )
                )
                time.sleep(1.0)

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            if launch_proc.poll() is not None:
                return_code = launch_proc.returncode or 0
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for idx, process in enumerate(exercise_procs, start=1):
            terminate(process, f"exercise-{idx}")
        if record_proc is not None:
            terminate(record_proc, "rosbag-record")
        terminate(launch_proc, "launch")

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
