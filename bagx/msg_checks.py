"""Content checks for common sensor messages.

Rates and timing say whether a topic was recorded; these checks say whether
what was recorded is usable: scans with no valid ranges, uncalibrated
``camera_info``, truncated images, and ``joint_states`` topics shared by
publishers with different joint sets.

Works on both the SQLite/CDR fallback dicts (``num_ranges``, ``data_size``)
and fully decoded messages (``ranges``, ``data``). Only small per-topic
counters are kept; no message payloads are stored.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from bagx.findings import Evidence, Finding, clean_topic_token, finding_id
from bagx.reader import Message

LASER_SCAN_TYPES = {"sensor_msgs/msg/LaserScan", "sensor_msgs/LaserScan"}
CAMERA_INFO_TYPES = {"sensor_msgs/msg/CameraInfo", "sensor_msgs/CameraInfo"}
IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/Image"}
JOINT_STATE_TYPES = {"sensor_msgs/msg/JointState", "sensor_msgs/JointState"}

# Distinct values tracked per topic before we stop recording new ones.
MAX_DISTINCT = 8
# Mean share of in-range scan returns below which a scan topic is flagged.
LOW_VALID_RANGE_RATIO = 0.5


def _frame_id(data: dict[str, Any]) -> str:
    frame = data.get("frame_id")
    if frame is None and isinstance(data.get("header"), dict):
        frame = data["header"].get("frame_id")
    return str(frame or "")


def _seq(data: dict[str, Any], *keys: str) -> list:
    """Return the first present sequence field as a list (arrays have no truth value)."""
    for key in keys:
        value = data.get(key)
        if value is not None:
            return list(value)
    return []


def _add_distinct(values: list, value: Any) -> None:
    if value not in values and len(values) < MAX_DISTINCT:
        values.append(value)


def _sequence_len(value: Any) -> int:
    if value is None:
        return 0
    if isinstance(value, (bytes, bytearray, memoryview, str)):
        return len(value)
    try:
        return len(value)
    except TypeError:
        return 0


@dataclass
class _ScanStats:
    count: int = 0
    valid_ratio_sum: float = 0.0
    all_invalid: int = 0
    beam_counts: list[int] = field(default_factory=list)
    bad_config: str | None = None
    empty_frame: int = 0

    def add(self, data: dict[str, Any]) -> None:
        if data.get("_parse_error"):
            return
        range_min = float(data.get("range_min", 0.0))
        range_max = float(data.get("range_max", 0.0))
        if "num_ranges" in data:
            n = int(data["num_ranges"])
            valid = int(data.get("num_valid_ranges", 0))
        else:
            ranges = np.asarray(_seq(data, "ranges"), dtype=np.float32)
            n = int(ranges.size)
            valid = int(np.count_nonzero(
                np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
            ))
        self.count += 1
        _add_distinct(self.beam_counts, n)
        if n:
            self.valid_ratio_sum += valid / n
            if valid == 0:
                self.all_invalid += 1
        if not _frame_id(data):
            self.empty_frame += 1
        if self.bad_config is None:
            if float(data.get("angle_increment", 1.0)) == 0.0:
                self.bad_config = "angle_increment is 0"
            elif range_max <= range_min:
                self.bad_config = f"range_max ({range_max:g}) <= range_min ({range_min:g})"


@dataclass
class _CameraInfoStats:
    count: int = 0
    uncalibrated: int = 0
    zero_size: int = 0
    sizes: list[tuple[int, int]] = field(default_factory=list)
    empty_frame: int = 0

    def add(self, data: dict[str, Any]) -> None:
        if data.get("_parse_error"):
            return
        self.count += 1
        k = _seq(data, "k", "K")
        if not k or not k[0] or (len(k) > 4 and not k[4]):
            self.uncalibrated += 1
        width, height = int(data.get("width", 0)), int(data.get("height", 0))
        if not width or not height:
            self.zero_size += 1
        _add_distinct(self.sizes, (width, height))
        if not _frame_id(data):
            self.empty_frame += 1


@dataclass
class _ImageStats:
    count: int = 0
    size_mismatch: int = 0
    sizes: list[tuple[int, int]] = field(default_factory=list)
    encodings: list[str] = field(default_factory=list)
    empty_frame: int = 0

    def add(self, data: dict[str, Any]) -> None:
        if data.get("_parse_error"):
            return
        self.count += 1
        width, height = int(data.get("width", 0)), int(data.get("height", 0))
        step = int(data.get("step", 0))
        size = data["data_size"] if "data_size" in data else _sequence_len(data.get("data"))
        if step and height and size != step * height:
            self.size_mismatch += 1
        _add_distinct(self.sizes, (width, height))
        _add_distinct(self.encodings, str(data.get("encoding", "")))
        if not _frame_id(data):
            self.empty_frame += 1


@dataclass
class _JointStateStats:
    count: int = 0
    name_sets: list[tuple[str, ...]] = field(default_factory=list)
    length_mismatch: int = 0
    nan_positions: int = 0

    def add(self, data: dict[str, Any]) -> None:
        if data.get("_parse_error"):
            return
        self.count += 1
        names = tuple(str(n) for n in _seq(data, "name"))
        _add_distinct(self.name_sets, names)
        positions = _seq(data, "position")
        if positions and len(positions) != len(names):
            self.length_mismatch += 1
        if any(isinstance(p, float) and math.isnan(p) for p in positions):
            self.nan_positions += 1


_STATS_BY_TYPE = (
    (LASER_SCAN_TYPES, _ScanStats),
    (CAMERA_INFO_TYPES, _CameraInfoStats),
    (IMAGE_TYPES, _ImageStats),
    (JOINT_STATE_TYPES, _JointStateStats),
)


class MessageCheckAccumulator:
    """Streams messages from a bag read pass into per-topic content stats."""

    def __init__(self, topic_types: dict[str, str]):
        self._stats: dict[str, Any] = {}
        for topic, msg_type in topic_types.items():
            for types, cls in _STATS_BY_TYPE:
                if msg_type in types:
                    self._stats[topic] = cls()

    def add(self, msg: Message) -> None:
        stats = self._stats.get(msg.topic)
        if stats is not None and isinstance(msg.data, dict):
            stats.add(msg.data)

    def build(self) -> list[Finding]:
        findings: list[Finding] = []
        images = {t: s for t, s in self._stats.items() if isinstance(s, _ImageStats)}
        for topic, stats in sorted(self._stats.items()):
            if not stats.count:
                continue
            if isinstance(stats, _ScanStats):
                findings.extend(_scan_findings(topic, stats))
            elif isinstance(stats, _CameraInfoStats):
                findings.extend(_camera_info_findings(topic, stats, images))
            elif isinstance(stats, _ImageStats):
                findings.extend(_image_findings(topic, stats))
            elif isinstance(stats, _JointStateStats):
                findings.extend(_joint_state_findings(topic, stats))
        return findings


def _finding(kind: str, topic: str, **kwargs: Any) -> Finding:
    return Finding(
        id=finding_id("msg", kind, clean_topic_token(topic)),
        category="message_content",
        affected_topics=[topic],
        **kwargs,
    )


def _empty_frame_finding(topic: str, empty: int, count: int) -> list[Finding]:
    if not empty:
        return []
    return [_finding(
        "empty_frame_id", topic,
        title=f"{topic} has an empty header.frame_id in {empty}/{count} messages",
        severity="warning",
        evidence=[Evidence(metric="empty_frame_id_messages", observed=empty, expected=0)],
        suggested_action="Set frame_id in the driver; without it the data cannot be placed via TF.",
        confidence="high",
    )]


def _scan_findings(topic: str, s: _ScanStats) -> list[Finding]:
    findings: list[Finding] = []
    if s.bad_config:
        findings.append(_finding(
            "scan_config", topic,
            title=f"{topic} has an invalid LaserScan configuration: {s.bad_config}",
            severity="error",
            evidence=[Evidence(metric="scan_config", observed=s.bad_config)],
            suggested_action="Fix the driver parameters; consumers reject or misplace these scans.",
            confidence="high",
        ))
    ratio = s.valid_ratio_sum / s.count
    if s.all_invalid == s.count:
        findings.append(_finding(
            "scan_no_valid_ranges", topic,
            title=f"{topic}: every scan has zero valid ranges",
            severity="error",
            evidence=[Evidence(metric="valid_range_ratio", observed=0.0, expected=">= 0.5")],
            suggested_action=(
                "Check range_min/range_max and the sensor itself (covered, disconnected, or "
                "returning inf/NaN)."
            ),
            confidence="high",
        ))
    elif ratio < LOW_VALID_RANGE_RATIO:
        findings.append(_finding(
            "scan_low_valid_ranges", topic,
            title=f"{topic}: only {ratio:.0%} of scan returns are within range on average",
            severity="warning",
            evidence=[
                Evidence(metric="valid_range_ratio", observed=round(ratio, 3),
                         expected=f">= {LOW_VALID_RANGE_RATIO}"),
                Evidence(metric="scans_with_no_valid_ranges", observed=s.all_invalid),
            ],
            suggested_action=(
                "Mostly-empty scans weaken costmaps and scan matching; check range limits and "
                "occlusions near the sensor."
            ),
            confidence="medium",
        ))
    if len(s.beam_counts) > 1:
        findings.append(_finding(
            "scan_beam_count_changes", topic,
            title=f"{topic} beam count changes during the bag ({sorted(s.beam_counts)})",
            severity="warning",
            evidence=[Evidence(metric="beam_counts", observed=sorted(s.beam_counts), expected=1)],
            suggested_action=(
                "Some consumers (e.g. AMCL, scan matchers) assume a fixed beam count; check for "
                "two publishers or a driver restart."
            ),
            confidence="medium",
        ))
    findings.extend(_empty_frame_finding(topic, s.empty_frame, s.count))
    return findings


def _matching_image_topics(info_topic: str, images: dict[str, _ImageStats]) -> list[str]:
    namespace = info_topic.rsplit("/", 1)[0]
    return [t for t in images if t.rsplit("/", 1)[0] == namespace]


def _camera_info_findings(
    topic: str, s: _CameraInfoStats, images: dict[str, _ImageStats]
) -> list[Finding]:
    findings: list[Finding] = []
    if s.uncalibrated:
        findings.append(_finding(
            "camera_uncalibrated", topic,
            title=f"{topic} has an empty intrinsic matrix K in {s.uncalibrated}/{s.count} messages",
            severity="warning",
            evidence=[Evidence(metric="uncalibrated_messages", observed=s.uncalibrated,
                               expected=0)],
            suggested_action=(
                "The camera was not calibrated when recorded; perception and projection using "
                "this camera_info will be wrong."
            ),
            confidence="high",
        ))
    if s.zero_size:
        findings.append(_finding(
            "camera_info_zero_size", topic,
            title=f"{topic} reports width or height 0 in {s.zero_size}/{s.count} messages",
            severity="warning",
            evidence=[Evidence(metric="zero_size_messages", observed=s.zero_size, expected=0)],
            suggested_action="Republish camera_info with the real image size.",
            confidence="high",
        ))
    info_sizes = {size for size in s.sizes if all(size)}
    for image_topic in _matching_image_topics(topic, images):
        image_sizes = {size for size in images[image_topic].sizes if all(size)}
        if info_sizes and image_sizes and not info_sizes & image_sizes:
            findings.append(_finding(
                "camera_info_size_mismatch", topic,
                title=(
                    f"{topic} size {sorted(info_sizes)} does not match {image_topic} "
                    f"{sorted(image_sizes)}"
                ),
                severity="warning",
                evidence=[
                    Evidence(metric="camera_info_size", observed=sorted(info_sizes),
                             topic=topic),
                    Evidence(metric="image_size", observed=sorted(image_sizes),
                             topic=image_topic),
                ],
                suggested_action=(
                    "The calibration belongs to a different resolution; rescale K/P or record "
                    "the matching camera_info."
                ),
                confidence="medium",
            ))
            findings[-1].affected_topics.append(image_topic)
    findings.extend(_empty_frame_finding(topic, s.empty_frame, s.count))
    return findings


def _image_findings(topic: str, s: _ImageStats) -> list[Finding]:
    findings: list[Finding] = []
    if s.size_mismatch:
        findings.append(_finding(
            "image_size_mismatch", topic,
            title=f"{topic}: data size != step × height in {s.size_mismatch}/{s.count} images",
            severity="error",
            evidence=[Evidence(metric="mismatched_images", observed=s.size_mismatch,
                               expected=0)],
            suggested_action="Images are truncated or mislabelled; fix step/encoding in the driver.",
            confidence="high",
        ))
    if len(s.sizes) > 1 or len(s.encodings) > 1:
        changes = []
        if len(s.sizes) > 1:
            changes.append(f"resolution {sorted(s.sizes)}")
        if len(s.encodings) > 1:
            changes.append(f"encoding {sorted(s.encodings)}")
        findings.append(_finding(
            "image_format_changes", topic,
            title=f"{topic} changes {' and '.join(changes)} during the bag",
            severity="warning",
            evidence=[
                Evidence(metric="resolutions", observed=sorted(s.sizes)),
                Evidence(metric="encodings", observed=sorted(s.encodings)),
            ],
            suggested_action="Check for two publishers or a mid-recording driver reconfigure.",
            confidence="medium",
        ))
    findings.extend(_empty_frame_finding(topic, s.empty_frame, s.count))
    return findings


def _joint_state_findings(topic: str, s: _JointStateStats) -> list[Finding]:
    findings: list[Finding] = []
    if len(s.name_sets) > 1:
        sizes = sorted(len(names) for names in s.name_sets)
        findings.append(_finding(
            "joint_state_name_set_changes", topic,
            title=(
                f"{topic} carries {len(s.name_sets)} different joint sets "
                f"(sizes {sizes}) — likely several publishers on one topic"
            ),
            severity="warning",
            evidence=[Evidence(metric="joint_name_sets",
                               observed=[list(names) for names in s.name_sets], expected=1)],
            suggested_action=(
                "Consumers that index joints by position break; merge with "
                "joint_state_publisher or give each source its own topic."
            ),
            confidence="medium",
        ))
    if s.length_mismatch:
        findings.append(_finding(
            "joint_state_length_mismatch", topic,
            title=(
                f"{topic}: position length != name length in "
                f"{s.length_mismatch}/{s.count} messages"
            ),
            severity="error",
            evidence=[Evidence(metric="mismatched_messages", observed=s.length_mismatch,
                               expected=0)],
            suggested_action="The publisher fills position inconsistently with name; fix it.",
            confidence="high",
        ))
    if s.nan_positions:
        findings.append(_finding(
            "joint_state_nan", topic,
            title=f"{topic} has NaN joint positions in {s.nan_positions}/{s.count} messages",
            severity="warning",
            evidence=[Evidence(metric="nan_messages", observed=s.nan_positions, expected=0)],
            suggested_action="Check encoder/driver health; NaN joints break FK and TF.",
            confidence="high",
        ))
    return findings
