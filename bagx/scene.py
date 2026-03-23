"""3D scene state extraction from rosbag data.

Extracts position, orientation, velocity time series from bag files,
merging data from multiple sensor sources into a unified timeline.
"""

from __future__ import annotations

import csv
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO

from rich.console import Console
from rich.table import Table

from bagx.contracts import report_metadata
from bagx.reader import BagReader, Message

logger = logging.getLogger(__name__)


# Message types that carry scene-relevant data
_SCENE_TYPES = {
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
    "nav_msgs/msg/Odometry",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/NavSatFix",
    "tf2_msgs/msg/TFMessage",
}


@dataclass
class SceneState:
    timestamp_sec: float
    position: tuple[float, float, float] | None = None
    orientation: tuple[float, float, float, float] | None = None
    linear_velocity: tuple[float, float, float] | None = None
    angular_velocity: tuple[float, float, float] | None = None
    acceleration: tuple[float, float, float] | None = None
    source_topic: str = ""


@dataclass
class SceneReport:
    bag_path: str
    states: list[SceneState] = field(default_factory=list)
    duration_sec: float = 0.0
    sources: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict:
        d = {
            "bag_path": self.bag_path,
            "duration_sec": self.duration_sec,
            "sources": self.sources,
            "state_count": len(self.states),
            "states": [_state_to_dict(s) for s in self.states],
        }
        d.update(report_metadata("scene"))
        return d


def _state_to_dict(s: SceneState) -> dict:
    d: dict = {"timestamp_sec": s.timestamp_sec, "source_topic": s.source_topic}
    if s.position is not None:
        d["position"] = {"x": s.position[0], "y": s.position[1], "z": s.position[2]}
    if s.orientation is not None:
        d["orientation"] = {
            "x": s.orientation[0],
            "y": s.orientation[1],
            "z": s.orientation[2],
            "w": s.orientation[3],
        }
    if s.linear_velocity is not None:
        d["linear_velocity"] = {
            "x": s.linear_velocity[0],
            "y": s.linear_velocity[1],
            "z": s.linear_velocity[2],
        }
    if s.angular_velocity is not None:
        d["angular_velocity"] = {
            "x": s.angular_velocity[0],
            "y": s.angular_velocity[1],
            "z": s.angular_velocity[2],
        }
    if s.acceleration is not None:
        d["acceleration"] = {
            "x": s.acceleration[0],
            "y": s.acceleration[1],
            "z": s.acceleration[2],
        }
    return d


def _is_scene_type(msg_type: str) -> bool:
    """Check if a message type carries scene-relevant data."""
    return msg_type in _SCENE_TYPES


def _extract_state_from_message(msg: Message, msg_type: str) -> SceneState | None:
    """Extract a SceneState from a single message based on its type."""
    data = msg.data
    ts = msg.timestamp_sec

    if msg_type == "sensor_msgs/msg/NavSatFix":
        return _state_from_navsatfix(data, ts, msg.topic)
    elif msg_type == "sensor_msgs/msg/Imu":
        return _state_from_imu(data, ts, msg.topic)
    elif msg_type == "geometry_msgs/msg/PoseStamped":
        return _state_from_pose_stamped(data, ts, msg.topic)
    elif msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped":
        return _state_from_pose_with_covariance(data, ts, msg.topic)
    elif msg_type == "nav_msgs/msg/Odometry":
        return _state_from_odometry(data, ts, msg.topic)
    elif msg_type == "tf2_msgs/msg/TFMessage":
        return _state_from_tf(data, ts, msg.topic)
    return None


def _state_from_navsatfix(data: dict, ts: float, topic: str) -> SceneState | None:
    """Extract position from NavSatFix (lat/lon/alt as x/y/z)."""
    lat = data.get("latitude")
    lon = data.get("longitude")
    alt = data.get("altitude")
    if lat is None or lon is None:
        return None
    return SceneState(
        timestamp_sec=ts,
        position=(lat, lon, alt if alt is not None else 0.0),
        source_topic=topic,
    )


def _state_from_imu(data: dict, ts: float, topic: str) -> SceneState | None:
    """Extract angular_velocity, acceleration, and optionally orientation from IMU."""
    ang_vel = None
    accel = None
    orient = None

    av = data.get("angular_velocity")
    if isinstance(av, dict) and "x" in av:
        ang_vel = (av["x"], av["y"], av["z"])

    la = data.get("linear_acceleration")
    if isinstance(la, dict) and "x" in la:
        accel = (la["x"], la["y"], la["z"])

    ori = data.get("orientation")
    if isinstance(ori, dict) and "w" in ori:
        # Only include if not all zeros (some IMUs don't provide orientation)
        if not (ori["x"] == 0.0 and ori["y"] == 0.0 and ori["z"] == 0.0 and ori["w"] == 0.0):
            orient = (ori["x"], ori["y"], ori["z"], ori["w"])

    if ang_vel is None and accel is None and orient is None:
        return None

    return SceneState(
        timestamp_sec=ts,
        orientation=orient,
        angular_velocity=ang_vel,
        acceleration=accel,
        source_topic=topic,
    )


def _state_from_pose_stamped(data: dict, ts: float, topic: str) -> SceneState | None:
    """Extract position + orientation from PoseStamped."""
    pos = data.get("position") or data.get("pose", {}).get("position")
    ori = data.get("orientation") or data.get("pose", {}).get("orientation")

    position = None
    orientation = None

    if isinstance(pos, dict) and "x" in pos:
        position = (pos["x"], pos["y"], pos["z"])
    if isinstance(ori, dict) and "w" in ori:
        orientation = (ori["x"], ori["y"], ori["z"], ori["w"])

    if position is None and orientation is None:
        return None

    return SceneState(
        timestamp_sec=ts,
        position=position,
        orientation=orientation,
        source_topic=topic,
    )


def _state_from_pose_with_covariance(data: dict, ts: float, topic: str) -> SceneState | None:
    """Extract position + orientation from PoseWithCovarianceStamped."""
    pose = data.get("pose", {})
    if isinstance(pose, dict) and "pose" in pose:
        pose = pose["pose"]
    return _state_from_pose_stamped(pose, ts, topic)


def _state_from_odometry(data: dict, ts: float, topic: str) -> SceneState | None:
    """Extract position + orientation + velocity from Odometry."""
    pose = data.get("pose", {})
    if isinstance(pose, dict) and "pose" in pose:
        pose = pose["pose"]

    position = None
    orientation = None
    linear_vel = None
    angular_vel = None

    pos = pose.get("position") if isinstance(pose, dict) else None
    ori = pose.get("orientation") if isinstance(pose, dict) else None

    if isinstance(pos, dict) and "x" in pos:
        position = (pos["x"], pos["y"], pos["z"])
    if isinstance(ori, dict) and "w" in ori:
        orientation = (ori["x"], ori["y"], ori["z"], ori["w"])

    twist = data.get("twist", {})
    if isinstance(twist, dict) and "twist" in twist:
        twist = twist["twist"]
    lin = twist.get("linear") if isinstance(twist, dict) else None
    ang = twist.get("angular") if isinstance(twist, dict) else None

    if isinstance(lin, dict) and "x" in lin:
        linear_vel = (lin["x"], lin["y"], lin["z"])
    if isinstance(ang, dict) and "x" in ang:
        angular_vel = (ang["x"], ang["y"], ang["z"])

    if position is None and orientation is None and linear_vel is None and angular_vel is None:
        return None

    return SceneState(
        timestamp_sec=ts,
        position=position,
        orientation=orientation,
        linear_velocity=linear_vel,
        angular_velocity=angular_vel,
        source_topic=topic,
    )


def _state_from_tf(data: dict, ts: float, topic: str) -> SceneState | None:
    """Extract position + orientation from the first transform in TFMessage."""
    transforms = data.get("transforms", [])
    if not transforms:
        return None

    t = transforms[0]
    transform = t.get("transform", {})

    position = None
    orientation = None

    tr = transform.get("translation")
    if isinstance(tr, dict) and "x" in tr:
        position = (tr["x"], tr["y"], tr["z"])

    rot = transform.get("rotation")
    if isinstance(rot, dict) and "w" in rot:
        orientation = (rot["x"], rot["y"], rot["z"], rot["w"])

    if position is None and orientation is None:
        return None

    return SceneState(
        timestamp_sec=ts,
        position=position,
        orientation=orientation,
        source_topic=topic,
    )


def extract_scene(
    bag_path: str,
    topics: list[str] | None = None,
    output_json: TextIO | None = None,
) -> SceneReport:
    """Extract 3D scene states from a bag file.

    Auto-detects topics with scene-relevant message types unless
    specific topics are provided.
    """
    reader = BagReader(bag_path)
    summary = reader.summary()

    # Determine which topics to read
    if topics:
        read_topics = topics
        topic_type_map = {
            name: info.type
            for name, info in summary.topics.items()
            if name in topics
        }
    else:
        # Auto-detect scene-relevant topics
        topic_type_map = {
            name: info.type
            for name, info in summary.topics.items()
            if _is_scene_type(info.type)
        }
        read_topics = list(topic_type_map.keys()) if topic_type_map else None
        if not topic_type_map:
            logger.warning("No scene-relevant topics found in %s", bag_path)

    states: list[SceneState] = []
    sources: dict[str, int] = {}

    for msg in reader.read_messages(topics=read_topics):
        msg_type = topic_type_map.get(msg.topic, "")
        state = _extract_state_from_message(msg, msg_type)
        if state is not None:
            states.append(state)
            sources[msg.topic] = sources.get(msg.topic, 0) + 1

    # Sort by timestamp
    states.sort(key=lambda s: s.timestamp_sec)

    # Calculate duration
    duration = 0.0
    if len(states) >= 2:
        duration = states[-1].timestamp_sec - states[0].timestamp_sec

    report = SceneReport(
        bag_path=str(bag_path),
        states=states,
        duration_sec=duration,
        sources=sources,
    )

    if output_json:
        json.dump(report.to_dict(), output_json, indent=2)

    return report


def export_scene_csv(report: SceneReport, path: Path) -> None:
    """Export scene states as CSV."""
    fieldnames = [
        "timestamp",
        "pos_x",
        "pos_y",
        "pos_z",
        "ori_x",
        "ori_y",
        "ori_z",
        "ori_w",
        "vel_x",
        "vel_y",
        "vel_z",
        "ang_vel_x",
        "ang_vel_y",
        "ang_vel_z",
        "accel_x",
        "accel_y",
        "accel_z",
        "source",
    ]

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for s in report.states:
            row: dict = {"timestamp": s.timestamp_sec, "source": s.source_topic}
            if s.position is not None:
                row["pos_x"] = s.position[0]
                row["pos_y"] = s.position[1]
                row["pos_z"] = s.position[2]
            if s.orientation is not None:
                row["ori_x"] = s.orientation[0]
                row["ori_y"] = s.orientation[1]
                row["ori_z"] = s.orientation[2]
                row["ori_w"] = s.orientation[3]
            if s.linear_velocity is not None:
                row["vel_x"] = s.linear_velocity[0]
                row["vel_y"] = s.linear_velocity[1]
                row["vel_z"] = s.linear_velocity[2]
            if s.angular_velocity is not None:
                row["ang_vel_x"] = s.angular_velocity[0]
                row["ang_vel_y"] = s.angular_velocity[1]
                row["ang_vel_z"] = s.angular_velocity[2]
            if s.acceleration is not None:
                row["accel_x"] = s.acceleration[0]
                row["accel_y"] = s.acceleration[1]
                row["accel_z"] = s.acceleration[2]

            writer.writerow(row)


def print_scene_report(report: SceneReport, console: Console | None = None) -> None:
    """Pretty-print a scene extraction report."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Scene Report: [cyan]{report.bag_path}[/cyan][/bold]")
    console.print(f"Total states: {len(report.states):,} | Duration: {report.duration_sec:.2f}s")
    console.print(f"Sources: {len(report.sources)}")

    if report.sources:
        table = Table(title="Sources", show_header=True)
        table.add_column("Topic", style="bold")
        table.add_column("Count", justify="right")
        table.add_column("Rate", justify="right")

        for topic, count in sorted(report.sources.items()):
            rate = f"{count / report.duration_sec:.1f} Hz" if report.duration_sec > 0 else "N/A"
            table.add_row(topic, str(count), rate)

        console.print(table)

    # Coverage statistics
    if report.states:
        n = len(report.states)
        pos_count = sum(1 for s in report.states if s.position is not None)
        ori_count = sum(1 for s in report.states if s.orientation is not None)
        vel_count = sum(1 for s in report.states if s.linear_velocity is not None)
        ang_count = sum(1 for s in report.states if s.angular_velocity is not None)
        acc_count = sum(1 for s in report.states if s.acceleration is not None)

        cov_table = Table(title="Field Coverage", show_header=True)
        cov_table.add_column("Field", style="bold")
        cov_table.add_column("Count", justify="right")
        cov_table.add_column("Coverage", justify="right")

        for name, cnt in [
            ("Position", pos_count),
            ("Orientation", ori_count),
            ("Linear Velocity", vel_count),
            ("Angular Velocity", ang_count),
            ("Acceleration", acc_count),
        ]:
            pct = cnt / n * 100 if n > 0 else 0
            cov_table.add_row(name, str(cnt), f"{pct:.1f}%")

        console.print(cov_table)

    console.print()
