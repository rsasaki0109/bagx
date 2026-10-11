"""TF tree validation for recorded bags.

Rebuilds the transform tree from ``/tf`` and ``/tf_static`` and reports the
problems that make a bag hard to replay: frames with two parents, cycles,
disconnected trees, dynamic transforms that pause or stop, header stamps that
do not line up with sensor stamps, and sensor ``frame_id`` values that never
appear in the tree.

Only the transform graph and timestamps are kept; transform values are not
interpolated or composed.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from rich.console import Console
from rich.markup import escape

from bagx.findings import Evidence, Finding, TimeRange, clean_topic_token, finding_id
from bagx.reader import BagReader, Message

logger = logging.getLogger(__name__)

TF_MESSAGE_TYPES = {"tf2_msgs/msg/TFMessage", "tf2_msgs/TFMessage", "tf/tfMessage"}

# A dynamic edge must be silent for at least this long, and for GAP_FACTOR times
# its median update interval, before a pause is reported.
MIN_GAP_SEC = 0.5
GAP_FACTOR = 5.0
# A dynamic edge that goes quiet this long before the bag ends is reported.
MIN_STOP_SEC = 2.0
# If a sensor's (header stamp - record time) differs from TF's by more than this,
# TF lookups at the sensor's stamps will extrapolate.
CLOCK_MISMATCH_SEC = 1.0
# (stamp - record time) samples kept per topic for the clock comparison. Both TF
# and sensor samples come from the start of the bag, so they stay comparable.
SENSOR_STAMP_SAMPLES = 50
TF_STAMP_SAMPLES = 200


@dataclass
class TfEdge:
    parent: str
    child: str
    static: bool
    count: int
    topics: list[str] = field(default_factory=list)
    rate_hz: float | None = None
    max_gap_sec: float | None = None
    first_ns: int | None = None
    last_ns: int | None = None


@dataclass
class TfReport:
    bag_path: str
    tf_topics: list[str] = field(default_factory=list)
    frames: list[str] = field(default_factory=list)
    roots: list[str] = field(default_factory=list)
    edges: list[TfEdge] = field(default_factory=list)
    sensor_frames: dict[str, str] = field(default_factory=dict)
    findings: list[Finding] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "bag_path": self.bag_path,
            "tf_topics": list(self.tf_topics),
            "frames": list(self.frames),
            "roots": list(self.roots),
            "edges": [
                {
                    "parent": edge.parent,
                    "child": edge.child,
                    "static": edge.static,
                    "count": edge.count,
                    "topics": list(edge.topics),
                    "rate_hz": edge.rate_hz,
                    "max_gap_sec": edge.max_gap_sec,
                    "first_ns": edge.first_ns,
                    "last_ns": edge.last_ns,
                }
                for edge in self.edges
            ],
            "sensor_frames": dict(self.sensor_frames),
            "findings": [finding.to_dict() for finding in self.findings],
        }


def is_tf_topic(topic: str, msg_type: str) -> bool:
    return msg_type in TF_MESSAGE_TYPES


def _normalize_frame(frame: Any) -> str:
    # tf2 ignores a leading slash (ROS1 bags commonly use "/base_link").
    return str(frame or "").strip().lstrip("/")


def _stamp_ns(entry: dict[str, Any]) -> int | None:
    if "stamp_sec" in entry:
        return int(entry.get("stamp_sec", 0)) * 1_000_000_000 + int(entry.get("stamp_nanosec", 0))
    header = entry.get("header")
    if isinstance(header, dict):
        stamp = header.get("stamp")
        if isinstance(stamp, dict):
            nanosec = stamp.get("nanosec", stamp.get("nsec", 0))
            return int(stamp.get("sec", 0)) * 1_000_000_000 + int(nanosec or 0)
    return None


def _entry_frame(entry: dict[str, Any]) -> str:
    frame = entry.get("frame_id")
    if frame is None and isinstance(entry.get("header"), dict):
        frame = entry["header"].get("frame_id")
    return _normalize_frame(frame)


class _EdgeStats:
    """Streaming per-edge statistics; no per-message timestamps are kept."""

    __slots__ = (
        "count", "first_ns", "last_ns", "max_gap_ns", "gap_start_ns",
        "topics", "zero_stamps", "regressions", "last_stamp_ns",
    )

    def __init__(self) -> None:
        self.count = 0
        self.first_ns = 0
        self.last_ns = 0
        self.max_gap_ns = 0
        self.gap_start_ns = 0
        self.topics: set[str] = set()
        self.zero_stamps = 0
        self.regressions = 0
        self.last_stamp_ns: int | None = None

    def add(self, record_ns: int, stamp_ns: int | None, topic: str) -> None:
        if self.count == 0:
            self.first_ns = self.last_ns = record_ns
        else:
            gap = record_ns - self.last_ns
            if gap > self.max_gap_ns:
                self.max_gap_ns = gap
                self.gap_start_ns = self.last_ns
            self.first_ns = min(self.first_ns, record_ns)
            self.last_ns = max(self.last_ns, record_ns)
        self.count += 1
        self.topics.add(topic)
        if stamp_ns is None:
            return
        if stamp_ns == 0:
            self.zero_stamps += 1
            return
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.regressions += 1
        self.last_stamp_ns = stamp_ns

    def typical_interval_ns(self) -> float:
        # duration/count rather than a median: robust to burst timestamps and
        # needs no stored intervals.
        return (self.last_ns - self.first_ns) / (self.count - 1) if self.count > 1 else 0.0


class TfAccumulator:
    """Collects TF edges and sensor frame ids while a bag is being read.

    ``evaluate_bag`` feeds every message through :meth:`add` during its single
    read pass, so TF validation does not need a second pass over the bag.
    """

    def __init__(self, topic_types: dict[str, str]):
        self.tf_topics = sorted(t for t, ty in topic_types.items() if is_tf_topic(t, ty))
        self._tf_topic_set = set(self.tf_topics)
        self._edges: dict[tuple[str, str, bool], _EdgeStats] = defaultdict(_EdgeStats)
        self._sensor_frames: dict[str, str] = {}
        # (header stamp - record time) samples, compared between TF and sensors.
        self._tf_offsets: dict[str, list[int]] = defaultdict(list)
        self._sensor_offsets: dict[str, list[int]] = defaultdict(list)
        # Non-TF topics with nothing left to collect (no frame_id, or samples full).
        self._done: set[str] = set()

    def add(self, msg: Message) -> None:
        if msg.topic in self._done:
            return
        data = msg.data
        if not isinstance(data, dict):
            return
        if msg.topic in self._tf_topic_set:
            static = "tf_static" in msg.topic
            for entry in data.get("transforms") or []:
                if not isinstance(entry, dict):
                    continue
                parent = _entry_frame(entry)
                child = _normalize_frame(entry.get("child_frame_id"))
                if not parent or not child:
                    continue
                stamp = _stamp_ns(entry)
                self._edges[(parent, child, static)].add(msg.timestamp_ns, stamp, msg.topic)
                offsets = self._tf_offsets[msg.topic]
                if stamp and not static and len(offsets) < TF_STAMP_SAMPLES:
                    offsets.append(stamp - msg.timestamp_ns)
            return

        frame = _entry_frame(data)
        if not frame:
            # Frame ids are fixed per topic; a frameless first message means a frameless topic.
            if msg.topic not in self._sensor_frames:
                self._done.add(msg.topic)
            return
        self._sensor_frames.setdefault(msg.topic, frame)
        samples = self._sensor_offsets[msg.topic]
        stamp = _stamp_ns(data)
        if stamp:
            samples.append(stamp - msg.timestamp_ns)
        if len(samples) >= SENSOR_STAMP_SAMPLES:
            self._done.add(msg.topic)

    def build(
        self,
        bag_path: str,
        bag_end_ns: int | None = None,
        required_frames: list[str] | None = None,
    ) -> TfReport:
        report = TfReport(
            bag_path=str(bag_path),
            tf_topics=list(self.tf_topics),
            sensor_frames=dict(sorted(self._sensor_frames.items())),
        )
        for (parent, child, static), stats in sorted(self._edges.items()):
            report.edges.append(_build_edge(parent, child, static, stats))

        frames: set[str] = set()
        for edge in report.edges:
            frames.update((edge.parent, edge.child))
        report.frames = sorted(frames)
        report.roots = _find_roots(report.edges)

        findings = report.findings
        if not report.edges:
            findings.extend(_no_tf_findings(report, required_frames))
            return report

        findings.extend(_structure_findings(report))
        findings.extend(_timing_findings(report, self._edges, bag_end_ns))
        findings.extend(_clock_findings(report, self._tf_offsets, self._sensor_offsets))
        findings.extend(_sensor_frame_findings(report))
        if required_frames:
            findings.extend(_required_frame_findings(report, required_frames))
        return report


def analyze_tf(bag_path: str, required_frames: list[str] | None = None) -> TfReport:
    """Read a bag and validate its TF tree."""
    reader = BagReader(bag_path)
    summary = reader.summary()
    accumulator = TfAccumulator({name: info.type for name, info in summary.topics.items()})
    for msg in reader.read_messages(topics=None):
        accumulator.add(msg)
    return accumulator.build(
        bag_path,
        bag_end_ns=summary.end_time_ns or None,
        required_frames=required_frames,
    )


def _build_edge(parent: str, child: str, static: bool, stats: _EdgeStats) -> TfEdge:
    edge = TfEdge(
        parent=parent,
        child=child,
        static=static,
        count=stats.count,
        topics=sorted(stats.topics),
        first_ns=stats.first_ns,
        last_ns=stats.last_ns,
    )
    if static or stats.count < 2:
        return edge
    duration = (stats.last_ns - stats.first_ns) / 1e9
    if duration > 0:
        edge.rate_hz = round((stats.count - 1) / duration, 2)
    edge.max_gap_sec = round(stats.max_gap_ns / 1e9, 3)
    return edge


def _find_roots(edges: list[TfEdge]) -> list[str]:
    children = {edge.child for edge in edges}
    parents = {edge.parent for edge in edges}
    roots = sorted(parents - children)
    # A tree made only of a cycle has no root; report its smallest frame so the
    # connected-component count stays meaningful.
    for component in _components(edges):
        if not component & set(roots):
            roots.append(min(component))
    return sorted(roots)


def _components(edges: list[TfEdge]) -> list[set[str]]:
    adjacency: dict[str, set[str]] = defaultdict(set)
    for edge in edges:
        adjacency[edge.parent].add(edge.child)
        adjacency[edge.child].add(edge.parent)
    seen: set[str] = set()
    components: list[set[str]] = []
    for start in sorted(adjacency):
        if start in seen:
            continue
        stack = [start]
        component: set[str] = set()
        while stack:
            node = stack.pop()
            if node in component:
                continue
            component.add(node)
            stack.extend(adjacency[node] - component)
        seen |= component
        components.append(component)
    return components


def _find_cycle(edges: list[TfEdge]) -> list[str] | None:
    graph: dict[str, set[str]] = defaultdict(set)
    for edge in edges:
        graph[edge.parent].add(edge.child)
    state: dict[str, int] = {}
    path: list[str] = []

    def visit(node: str) -> list[str] | None:
        state[node] = 1
        path.append(node)
        for nxt in sorted(graph.get(node, ())):
            if state.get(nxt) == 1:
                return path[path.index(nxt):] + [nxt]
            if nxt not in state:
                found = visit(nxt)
                if found:
                    return found
        path.pop()
        state[node] = 2
        return None

    for node in sorted(graph):
        if node not in state:
            found = visit(node)
            if found:
                return found
    return None


def _edge_token(parent: str, child: str) -> str:
    return f"{clean_topic_token(parent)}.{clean_topic_token(child)}"


def _structure_findings(report: TfReport) -> list[Finding]:
    findings: list[Finding] = []
    parents_by_child: dict[str, set[str]] = defaultdict(set)
    kinds_by_pair: dict[tuple[str, str], set[bool]] = defaultdict(set)
    topics_by_child: dict[str, set[str]] = defaultdict(set)
    for edge in report.edges:
        parents_by_child[edge.child].add(edge.parent)
        kinds_by_pair[(edge.parent, edge.child)].add(edge.static)
        topics_by_child[edge.child].update(edge.topics)

    for child, parents in sorted(parents_by_child.items()):
        if len(parents) > 1:
            findings.append(Finding(
                id=finding_id("tf", "multiple_parents", clean_topic_token(child)),
                title=f"TF frame '{child}' has {len(parents)} parents: {', '.join(sorted(parents))}",
                severity="error",
                category="tf_tree",
                affected_topics=sorted(topics_by_child[child]),
                evidence=[Evidence(metric="parent_frames", observed=sorted(parents), expected=1)],
                suggested_action=(
                    "tf2 keeps only one parent per frame, so lookups flip between branches. "
                    "Stop one publisher or rename one of the child frames."
                ),
                confidence="high",
            ))

    cycle = _find_cycle(report.edges)
    if cycle:
        findings.append(Finding(
            id="tf.cycle",
            title=f"TF tree contains a cycle: {' → '.join(cycle)}",
            severity="error",
            category="tf_tree",
            affected_topics=list(report.tf_topics),
            evidence=[Evidence(metric="cycle_frames", observed=cycle)],
            suggested_action="Remove one of the transforms in the loop; a TF tree must be acyclic.",
            confidence="high",
        ))

    for (parent, child), kinds in sorted(kinds_by_pair.items()):
        if kinds == {True, False}:
            findings.append(Finding(
                id=finding_id("tf", "static_dynamic_conflict", _edge_token(parent, child)),
                title=f"TF {parent} → {child} is published on both /tf and /tf_static",
                severity="warning",
                category="tf_tree",
                affected_topics=list(report.tf_topics),
                evidence=[Evidence(metric="published_as", observed=["static", "dynamic"])],
                suggested_action="Publish each transform from exactly one source.",
                confidence="high",
            ))

    if len(report.roots) > 1:
        findings.append(Finding(
            id="tf.disconnected",
            title=(
                f"TF tree is split into {len(report.roots)} disconnected trees "
                f"(roots: {', '.join(report.roots)})"
            ),
            severity="warning",
            category="tf_tree",
            affected_topics=list(report.tf_topics),
            evidence=[Evidence(metric="root_frames", observed=list(report.roots), expected=1)],
            suggested_action=(
                "Frames in different trees cannot be looked up against each other. Record the "
                "missing link (often map → odom from localization, or a static sensor mount)."
            ),
            confidence="medium",
        ))
    return findings


def _edge_list(edges: list[TfEdge], limit: int = 3) -> str:
    names = [f"{edge.parent} → {edge.child}" for edge in edges]
    shown = ", ".join(names[:limit])
    return f"{shown} (+{len(names) - limit} more)" if len(names) > limit else shown


def _group_key(*ns: int) -> tuple[int, ...]:
    # Edges from one publisher pause together; group them at 10ms resolution.
    return tuple(round(value / 1e7) for value in ns)


def _timing_findings(
    report: TfReport,
    edge_stats: dict[tuple[str, str, bool], _EdgeStats],
    bag_end_ns: int | None,
) -> list[Finding]:
    findings: list[Finding] = []
    gaps: dict[tuple[int, ...], list[tuple[TfEdge, int, int, float, float]]] = defaultdict(list)
    stops: dict[tuple[int, ...], list[tuple[TfEdge, int]]] = defaultdict(list)

    for edge in report.edges:
        if edge.static:
            continue
        stats = edge_stats[(edge.parent, edge.child, False)]
        token = _edge_token(edge.parent, edge.child)

        if stats.count >= 3:
            typical = stats.typical_interval_ns()
            threshold = max(MIN_GAP_SEC * 1e9, GAP_FACTOR * typical)
            if stats.max_gap_ns > threshold:
                gap_start = stats.gap_start_ns
                gap_end = gap_start + stats.max_gap_ns
                gaps[_group_key(gap_start, gap_end)].append(
                    (edge, gap_start, gap_end, typical, threshold)
                )
            if bag_end_ns and bag_end_ns - stats.last_ns > max(threshold, MIN_STOP_SEC * 1e9):
                stops[_group_key(stats.last_ns)].append((edge, stats.last_ns))

        if stats.zero_stamps:
            findings.append(Finding(
                id=finding_id("tf", "zero_stamp", token),
                title=(
                    f"TF {edge.parent} → {edge.child} has {stats.zero_stamps} "
                    "transform(s) stamped 0"
                ),
                severity="warning",
                category="tf_tree",
                affected_topics=list(edge.topics),
                evidence=[Evidence(metric="zero_stamps", observed=stats.zero_stamps, expected=0)],
                suggested_action="Set header.stamp on dynamic transforms (now(), not Time()).",
                confidence="high",
            ))
        if stats.regressions:
            findings.append(Finding(
                id=finding_id("tf", "stamp_regression", token),
                title=(
                    f"TF {edge.parent} → {edge.child} header stamps go backwards "
                    f"{stats.regressions} time(s)"
                ),
                severity="warning",
                category="tf_tree",
                affected_topics=list(edge.topics),
                evidence=[
                    Evidence(metric="stamp_regressions", observed=stats.regressions, expected=0)
                ],
                suggested_action=(
                    "Two publishers or a clock reset are likely; tf2 drops out-of-order data "
                    "('TF_OLD_DATA')."
                ),
                confidence="medium",
            ))

    for group in gaps.values():
        edges = [item[0] for item in group]
        first, gap_start, gap_end, typical, threshold = group[0]
        gap_sec = (gap_end - gap_start) / 1e9
        findings.append(Finding(
            id=finding_id("tf", "edge_gap", _edge_token(first.parent, first.child)),
            title=(
                f"TF paused for {gap_sec:.2f}s (normally every {typical / 1e6:.0f}ms) on "
                f"{_edge_list(edges)}"
            ),
            severity="warning",
            category="tf_tree",
            affected_topics=sorted({t for edge in edges for t in edge.topics}),
            evidence=[
                Evidence(metric="max_gap", observed=round(gap_sec, 3),
                         expected=round(threshold / 1e9, 3), unit="s"),
                Evidence(metric="edges", observed=[f"{e.parent}->{e.child}" for e in edges]),
            ],
            suggested_action=(
                "Lookups through these edges fail or extrapolate during the gap; check the "
                "publishing node (localization, odometry, robot_state_publisher) at this time."
            ),
            confidence="medium",
            time_range=TimeRange(gap_start, gap_end),
        ))

    for group in stops.values():
        edges = [item[0] for item in group]
        last_ns = group[0][1]
        silent = (bag_end_ns - last_ns) / 1e9
        findings.append(Finding(
            id=finding_id("tf", "edge_stopped", _edge_token(edges[0].parent, edges[0].child)),
            title=f"TF stops {silent:.1f}s before the end of the bag on {_edge_list(edges)}",
            severity="warning",
            category="tf_tree",
            affected_topics=sorted({t for edge in edges for t in edge.topics}),
            evidence=[
                Evidence(metric="silent_tail", observed=round(silent, 3), unit="s"),
                Evidence(metric="edges", observed=[f"{e.parent}->{e.child}" for e in edges]),
            ],
            suggested_action=(
                "Data recorded after this point cannot be transformed through these edges; "
                "trim the bag or check why the publisher stopped."
            ),
            confidence="medium",
            time_range=TimeRange(last_ns, int(bag_end_ns)),
        ))
    return findings


def _clock_clusters(sources: dict[str, float]) -> list[list[str]]:
    """Group sources whose offsets lie within CLOCK_MISMATCH_SEC of a neighbour."""
    ordered = sorted(sources, key=sources.get)
    clusters: list[list[str]] = [[ordered[0]]]
    for topic in ordered[1:]:
        if sources[topic] - sources[clusters[-1][-1]] > CLOCK_MISMATCH_SEC * 1e9:
            clusters.append([topic])
        else:
            clusters[-1].append(topic)
    return clusters


def _clock_findings(
    report: TfReport,
    tf_offsets: dict[str, list[int]],
    sensor_offsets: dict[str, list[int]],
) -> list[Finding]:
    """Flag stamped topics that use a different clock from the rest.

    Each TF topic and each sensor topic is one source, measured as the median
    of (header stamp - record time). Sources are clustered by offset; the
    largest cluster is the reference clock and every other source is reported.
    When no cluster is largest, the split itself is reported once.
    """
    tf_sources = {topic: float(np.median(v)) for topic, v in tf_offsets.items() if v}
    if not tf_sources:
        return []
    sources = dict(tf_sources)
    sources.update({topic: float(np.median(v)) for topic, v in sensor_offsets.items() if v})
    clusters = _clock_clusters(sources)
    if len(clusters) < 2:
        return []

    sizes = sorted((len(cluster) for cluster in clusters), reverse=True)
    if sizes[0] == sizes[1]:
        groups = [sorted(cluster) for cluster in clusters]
        spread = (max(sources.values()) - min(sources.values())) / 1e9
        return [Finding(
            id="tf.clock_split",
            title=(
                f"Stamped topics use {len(groups)} different clocks ({spread:.1f}s apart): "
                + " vs ".join("(" + ", ".join(group) + ")" for group in groups)
            ),
            severity="warning",
            category="tf_tree",
            affected_topics=sorted(sources),
            evidence=[
                Evidence(metric="clock_groups", observed=groups, expected=1),
                Evidence(metric="offset_spread", observed=round(spread, 3), unit="s"),
            ],
            suggested_action=(
                "TF lookups between topics on different clocks extrapolate. Make every "
                "publisher stamp with the same clock (system time or /clock)."
            ),
            confidence="medium",
        )]

    reference = max(clusters, key=len)
    consensus = float(np.median([sources[topic] for topic in reference]))
    findings: list[Finding] = []
    for topic in sorted(set(sources) - set(reference)):
        offset = (sources[topic] - consensus) / 1e9
        is_tf = topic in tf_sources
        findings.append(Finding(
            id=finding_id("tf", "clock_mismatch", clean_topic_token(topic)),
            title=(
                f"{topic} header stamps are {offset:+.1f}s off from the other "
                f"{len(reference)} stamped topics — TF lookups will extrapolate"
            ),
            severity="warning",
            category="tf_tree",
            affected_topics=[topic],
            evidence=[Evidence(metric="stamp_offset_vs_reference", observed=round(offset, 3),
                               expected=0, unit="s", topic=topic)],
            suggested_action=(
                ("This TF publisher stamps with a different clock than the sensors "
                 "(device time, or use_sim_time mismatch).")
                if is_tf else
                ("This sensor stamps with a different clock than TF (driver using device "
                 "time, or use_sim_time mismatch).")
            ),
            confidence="medium",
        ))
    return findings


def _sensor_frame_findings(report: TfReport) -> list[Finding]:
    frames = set(report.frames)
    missing: dict[str, list[str]] = defaultdict(list)
    for topic, frame in report.sensor_frames.items():
        if frame not in frames:
            missing[frame].append(topic)
    findings: list[Finding] = []
    for frame, topics in sorted(missing.items()):
        findings.append(Finding(
            id=finding_id("tf", "frame_missing", clean_topic_token(frame)),
            title=f"Frame '{frame}' used by {', '.join(topics)} is not in the TF tree",
            severity="warning",
            category="tf_tree",
            affected_topics=topics,
            evidence=[Evidence(metric="frame_id", observed=frame, expected="present in TF tree")],
            suggested_action=(
                "Record the transform for this frame (usually a static sensor mount on "
                "/tf_static), or the data cannot be placed relative to the robot."
            ),
            confidence="high",
        ))
    return findings


def _required_frame_findings(report: TfReport, required_frames: list[str]) -> list[Finding]:
    required = [_normalize_frame(frame) for frame in required_frames if _normalize_frame(frame)]
    findings: list[Finding] = []
    frames = set(report.frames)
    for frame in required:
        if frame not in frames:
            findings.append(Finding(
                id=finding_id("tf", "required_frame_missing", clean_topic_token(frame)),
                title=f"Required TF frame '{frame}' is not in the bag",
                severity="error",
                category="tf_tree",
                affected_topics=list(report.tf_topics),
                evidence=[Evidence(metric="frame", observed=None, expected=frame)],
                suggested_action="Record the publisher that owns this frame.",
                confidence="high",
            ))
    component_of: dict[str, int] = {}
    for index, component in enumerate(_components(report.edges)):
        for frame in component:
            component_of[frame] = index
    for first, second in zip(required, required[1:]):
        if first not in frames or second not in frames:
            continue
        if component_of[first] != component_of[second]:
            findings.append(Finding(
                id=finding_id(
                    "tf", "required_chain_broken",
                    f"{clean_topic_token(first)}.{clean_topic_token(second)}",
                ),
                title=f"Required TF chain is broken: no path between '{first}' and '{second}'",
                severity="error",
                category="tf_tree",
                affected_topics=list(report.tf_topics),
                evidence=[Evidence(metric="connected", observed=False, expected=True)],
                suggested_action="Record the transform(s) linking these frames.",
                confidence="high",
            ))
    return findings


def _no_tf_findings(report: TfReport, required_frames: list[str] | None) -> list[Finding]:
    distinct = sorted(set(report.sensor_frames.values()))
    if required_frames:
        return [Finding(
            id="tf.not_recorded",
            title="No TF recorded, but TF frames were required",
            severity="error",
            category="tf_tree",
            evidence=[Evidence(metric="tf_topics", observed=0, expected=">= 1")],
            suggested_action="Record /tf and /tf_static.",
            confidence="high",
        )]
    if len(distinct) < 2:
        return []
    return [Finding(
        id="tf.not_recorded",
        title=(
            f"No TF recorded; {len(distinct)} sensor frames ({', '.join(distinct[:5])}"
            f"{', ...' if len(distinct) > 5 else ''}) cannot be related to each other from the bag"
        ),
        severity="info",
        category="tf_tree",
        affected_topics=sorted(report.sensor_frames),
        evidence=[Evidence(metric="sensor_frames", observed=distinct)],
        suggested_action=(
            "Fine if extrinsics come from a config file; otherwise record /tf_static."
        ),
        confidence="medium",
    )]


def print_tf_report(report: TfReport, console: Console | None = None) -> None:
    from rich.table import Table

    console = console or Console()
    console.print(f"\n[bold]TF Tree:[/bold] {report.bag_path}")
    if not report.edges:
        console.print("  No TF topics recorded.")
    else:
        console.print(
            f"  Topics: {', '.join(report.tf_topics)} · Frames: {len(report.frames)} · "
            f"Roots: {', '.join(report.roots)}"
        )
        table = Table(show_header=True)
        table.add_column("Parent → Child")
        table.add_column("Kind")
        table.add_column("Count", justify="right")
        table.add_column("Rate", justify="right")
        table.add_column("Max gap", justify="right")
        for edge in report.edges:
            table.add_row(
                f"{edge.parent} → {edge.child}",
                "static" if edge.static else "dynamic",
                str(edge.count),
                f"{edge.rate_hz:.1f}Hz" if edge.rate_hz else "—",
                f"{edge.max_gap_sec:.3f}s" if edge.max_gap_sec is not None else "—",
            )
        console.print(table)

    if not report.findings:
        console.print("[green]✔ No TF problems found[/green]")
        return
    markers = {"info": "[cyan]ℹ[/cyan]", "warning": "[yellow]⚠[/yellow]",
               "error": "[red]✘[/red]", "critical": "[red]✘[/red]"}
    console.print("\n[bold]Findings:[/bold]")
    for finding in report.findings:
        console.print(f"  {markers.get(finding.severity, '•')} {escape(finding.title)}")
