"""Domain plugin SDK for bagx readiness evaluation.

A domain plugin answers three questions for a given bag's topic table:

- **detect** — does this bag belong to my stack?
- **representative_topics** — which topics best summarise the stack?
- **generate_findings** — what structured findings should the eval report
  contribute for this stack?

Built-in domains (Nav2, Autoware, MoveIt, Perception, RobotArm, Control) are
exposed through the same protocol, and third-party packages can ship their
own domains via the ``bagx.domains`` entry point. The plugin's display
``name`` becomes the domain id used in finding ids (``<domain_id>.detected``)
after normalisation by :func:`bagx.findings.finding_id`.

This module deliberately avoids importing :mod:`bagx.eval` at module load
time to keep the dependency graph acyclic; built-in plugins resolve their
detection/finding helpers lazily.
"""

from __future__ import annotations

import importlib.metadata as _md
import inspect
import json
import logging
import re
from dataclasses import dataclass, field
from functools import partial
from typing import TYPE_CHECKING, Any, Callable, Protocol, runtime_checkable

if TYPE_CHECKING:
    from bagx.eval import EvalReport
    from bagx.findings import Finding

logger = logging.getLogger("bagx.domain_plugins")

ENTRY_POINT_GROUP = "bagx.domains"


@runtime_checkable
class DomainPlugin(Protocol):
    """Protocol implemented by built-in and third-party domain plugins."""

    name: str

    def detect(self, topic_info: dict[str, dict]) -> bool: ...

    def representative_topics(self, topic_info: dict[str, dict]) -> list[str]: ...

    def generate_findings(self, report: "EvalReport") -> list["Finding"]: ...


def _empty_topics(_topic_info: dict[str, dict]) -> list[str]:
    return []


def _empty_findings(_report: "EvalReport") -> list["Finding"]:
    return []


@dataclass
class SimpleDomainPlugin:
    """Function-backed plugin convenient for built-in and example usage.

    Pass callables for the three hooks. ``representative`` and ``findings``
    are optional and default to returning empty lists, which is useful for
    plugins that only need to flag detection without contributing further
    findings.
    """

    name: str
    detect_fn: Callable[[dict[str, dict]], bool]
    representative_fn: Callable[[dict[str, dict]], list[str]] = field(default=_empty_topics)
    findings_fn: Callable[["EvalReport"], list["Finding"]] = field(default=_empty_findings)
    _is_builtin: bool = field(default=False, init=False, repr=False)

    def detect(self, topic_info: dict[str, dict]) -> bool:
        return bool(self.detect_fn(topic_info))

    def representative_topics(self, topic_info: dict[str, dict]) -> list[str]:
        result = self.representative_fn(topic_info)
        if isinstance(result, str):
            raise TypeError("representative_topics must return topic names, not a string")
        return list(result)

    def generate_findings(self, report: "EvalReport") -> list["Finding"]:
        return list(self.findings_fn(report))


def _builtin_plugins() -> list[DomainPlugin]:
    """Return the built-in domain plugins.

    Imports are deferred so :mod:`bagx.eval` can import this module without
    creating a circular dependency.
    """
    from bagx.eval_topics import (
        _has_control_signature,
        _has_nav2_signature,
        _select_topics,
    )

    def _autoware_topics(topics: dict[str, dict]) -> list[str]:
        prefixes = (
            "/sensing/",
            "/perception/",
            "/planning/",
            "/control/",
            "/localization/",
            "/vehicle/",
        )
        return [name for name in topics if any(name.startswith(p) for p in prefixes)]

    def _detect_autoware(topics: dict[str, dict]) -> bool:
        return bool(_autoware_topics(topics))

    def _detect_moveit(topics: dict[str, dict]) -> bool:
        joint = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        planned = _select_topics(
            topics,
            type_markers=("displaytrajectory",),
            suffixes=("/display_planned_path",),
            contains=("display_planned_path", "planned_path"),
        )
        scene = _select_topics(
            topics,
            type_markers=("planningscene",),
            suffixes=("/planning_scene", "/monitored_planning_scene"),
            contains=("planning_scene", "monitored_planning_scene"),
        )
        move_action = _select_topics(topics, contains=("move_action/_action/status",))
        execute_action = _select_topics(topics, contains=("execute_trajectory/_action/status",))
        controller_action = _select_topics(
            topics, contains=("follow_joint_trajectory/_action/status",)
        )
        evidence = sum(
            bool(m) for m in [joint, planned, scene, move_action, execute_action, controller_action]
        )
        return evidence >= 2 or (bool(joint) and bool(planned))

    def _color_image_topics(topics: dict[str, dict]) -> list[str]:
        images = _select_topics(topics, type_markers=("image", "compressedimage"))
        return [n for n in images if "depth" not in n.lower() and "infra" not in n.lower()]

    def _depth_image_topics(topics: dict[str, dict]) -> list[str]:
        images = _select_topics(topics, type_markers=("image", "compressedimage"))
        return [n for n in images if "depth" in n.lower()]

    def _infra_image_topics(topics: dict[str, dict]) -> list[str]:
        images = _select_topics(topics, type_markers=("image", "compressedimage"))
        return [n for n in images if "infra" in n.lower()]

    def _detect_robotarm(topics: dict[str, dict]) -> bool:
        joint = _select_topics(
            topics,
            type_markers=("jointstate",),
            suffixes=("/joint_states",),
            contains=("joint_states",),
        )
        return bool(joint) and bool(_color_image_topics(topics) or _depth_image_topics(topics))

    def _detect_perception(topics: dict[str, dict]) -> bool:
        camera_info = _select_topics(
            topics,
            type_markers=("camerainfo",),
            contains=("camera_info",),
        )
        images_present = bool(
            _color_image_topics(topics)
            or _depth_image_topics(topics)
            or _infra_image_topics(topics)
        )
        if not (camera_info and images_present):
            return False
        # Perception is suppressed when Autoware or RobotArm already covered the bag.
        if _detect_autoware(topics) or _detect_robotarm(topics):
            return False
        return True

    def _detect_control(topics: dict[str, dict]) -> bool:
        if not _has_control_signature(topics):
            return False
        if _has_nav2_signature(topics):
            return False
        if _detect_autoware(topics):
            return False
        if _detect_moveit(topics):
            return False
        return True

    def _nav2_topics(topics: dict[str, dict]) -> list[str]:
        return _select_topics(
            topics,
            type_markers=("odometry", "laserscan"),
            suffixes=("/odom", "/scan", "/cmd_vel", "/cmd_vel_smoothed"),
            contains=("navigate_to_pose", "local_costmap", "global_costmap"),
        )[:8]

    def _moveit_topics(topics: dict[str, dict]) -> list[str]:
        return _select_topics(
            topics,
            type_markers=("jointstate", "displaytrajectory", "planningscene"),
            suffixes=("/joint_states", "/display_planned_path"),
            contains=("move_action", "execute_trajectory", "follow_joint_trajectory"),
        )[:8]

    def _image_summary_topics(topics: dict[str, dict]) -> list[str]:
        return _select_topics(
            topics,
            type_markers=("image", "compressedimage", "camerainfo", "jointstate"),
            contains=("camera_info", "joint_states"),
        )[:8]

    def _all_topics(topics: dict[str, dict]) -> list[str]:
        return list(topics)[:8]

    from bagx.domain_findings import generate_domain_readiness_findings
    from bagx.eval_findings import _generate_nav2_findings_from_plugin

    plugins = [
        SimpleDomainPlugin(
            name="Nav2",
            detect_fn=_has_nav2_signature,
            representative_fn=_nav2_topics,
            findings_fn=_generate_nav2_findings_from_plugin,
        ),
        SimpleDomainPlugin(
            name="Autoware",
            detect_fn=_detect_autoware,
            representative_fn=lambda t: _autoware_topics(t)[:8],
            findings_fn=partial(generate_domain_readiness_findings, domain="Autoware"),
        ),
        SimpleDomainPlugin(
            name="MoveIt",
            detect_fn=_detect_moveit,
            representative_fn=_moveit_topics,
            findings_fn=partial(generate_domain_readiness_findings, domain="MoveIt"),
        ),
        SimpleDomainPlugin(
            name="Perception",
            detect_fn=_detect_perception,
            representative_fn=_image_summary_topics,
            findings_fn=partial(generate_domain_readiness_findings, domain="Perception"),
        ),
        SimpleDomainPlugin(
            name="RobotArm",
            detect_fn=_detect_robotarm,
            representative_fn=_image_summary_topics,
            findings_fn=partial(generate_domain_readiness_findings, domain="RobotArm"),
        ),
        SimpleDomainPlugin(
            name="Control",
            detect_fn=_detect_control,
            representative_fn=_all_topics,
            findings_fn=partial(generate_domain_readiness_findings, domain="Control"),
        ),
    ]
    for plugin in plugins:
        plugin._is_builtin = True
    return plugins


def _plugin_failure(name: str, hook: str, exc: Exception) -> "Finding":
    from bagx.findings import Evidence, Finding, finding_id

    logger.warning("Domain plugin %s failed in %s (%s)", name, hook, type(exc).__name__)
    return Finding(
        id=finding_id("plugin", name, f"{hook}_failed"),
        title=f"Domain plugin {name}: {hook} could not complete",
        severity="error",
        category="domain_detection",
        evidence=[
            Evidence(metric="plugin_hook", observed=hook),
            Evidence(metric="exception_type", observed=type(exc).__name__),
        ],
        suggested_action="Fix or remove this plugin before relying on the report; other checks still ran.",
        confidence="high",
    )


@dataclass
class _UnavailablePlugin:
    name: str
    failure: "Finding"

    def detect(self, topic_info: dict[str, dict]) -> bool:
        return False

    def representative_topics(self, topic_info: dict[str, dict]) -> list[str]:
        return []

    def generate_findings(self, report: "EvalReport") -> list["Finding"]:
        return [self.failure]


def _entry_point_plugins() -> list[DomainPlugin]:
    from bagx.findings import finding_id

    plugins: list[DomainPlugin] = []
    seen_ids = {finding_id(plugin.name) for plugin in _builtin_plugins()}
    try:
        entry_points = _md.entry_points(group=ENTRY_POINT_GROUP)
    except TypeError:
        # Older importlib.metadata API: entry_points() returns a dict.
        entry_points = _md.entry_points().get(ENTRY_POINT_GROUP, [])  # type: ignore[assignment]
    for entry_point in entry_points:
        try:
            obj: Any = entry_point.load()
            # Classes expose unbound hooks, so structural checks alone cannot
            # distinguish them from instances. Callable instances stay intact.
            instance = (
                obj()
                if inspect.isclass(obj) or (callable(obj) and not _looks_like_plugin(obj))
                else obj
            )
            if not _looks_like_plugin(instance):
                raise TypeError("entry point does not implement DomainPlugin")
            domain_id = finding_id(instance.name)
            if not domain_id or not domain_id[0].isalnum() or domain_id in seen_ids:
                raise ValueError("plugin name is empty or collides with another domain id")
            seen_ids.add(domain_id)
        except Exception as exc:
            plugins.append(
                _UnavailablePlugin(entry_point.name, _plugin_failure(entry_point.name, "load", exc))
            )
            continue
        plugins.append(instance)
    return plugins


def _looks_like_plugin(obj: Any) -> bool:
    return (
        isinstance(getattr(obj, "name", None), str)
        and bool(obj.name.strip())
        and callable(getattr(obj, "detect", None))
        and callable(getattr(obj, "representative_topics", None))
        and callable(getattr(obj, "generate_findings", None))
    )


def discover_domain_plugins() -> list[DomainPlugin]:
    """Return built-in plugins followed by entry-point installed plugins."""
    return [*_builtin_plugins(), *_entry_point_plugins()]


def plugin_source_label(plugin: DomainPlugin) -> str:
    """Return ``built-in`` or ``entry-point`` for CLI display."""
    if isinstance(plugin, _UnavailablePlugin):
        return "entry-point (unavailable)"
    return (
        "built-in"
        if isinstance(plugin, SimpleDomainPlugin) and plugin._is_builtin
        else "entry-point"
    )


def detect_domains(
    topic_info: dict[str, dict],
    plugins: list[DomainPlugin] | None = None,
    *,
    failures: list["Finding"] | None = None,
) -> list[DomainPlugin]:
    """Return the plugins whose ``detect`` returns True for ``topic_info``.

    Order is preserved from ``discover_domain_plugins`` (built-ins first, then
    entry-point plugins) which gives downstream code a stable iteration order.
    """
    if not topic_info:
        return []
    candidates = plugins if plugins is not None else discover_domain_plugins()
    detected = []
    for plugin in candidates:
        try:
            if plugin.detect(topic_info):
                detected.append(plugin)
        except Exception as exc:
            failure = _plugin_failure(plugin.name, "detect", exc)
            if failures is not None:
                failures.append(failure)
    return detected


def _plugin_load_findings(plugins: list[DomainPlugin]) -> list["Finding"]:
    return [plugin.failure for plugin in plugins if isinstance(plugin, _UnavailablePlugin)]


def _plugin_representative_topics(
    plugin: DomainPlugin,
    topics: dict[str, dict],
    failures: list["Finding"],
) -> list[str]:
    try:
        result = plugin.representative_topics(topics)
        if not isinstance(result, list) or any(not isinstance(topic, str) for topic in result):
            raise TypeError("representative_topics must return list[str]")
        return result
    except Exception as exc:
        failures.append(_plugin_failure(plugin.name, "representative_topics", exc))
        return []


def _plugin_readiness_findings(plugin: DomainPlugin, report: "EvalReport") -> list["Finding"]:
    from bagx.findings import Evidence, Finding, SEVERITY_ORDER, TimeRange

    try:
        results = plugin.generate_findings(report)
        if not isinstance(results, list) or any(not isinstance(item, Finding) for item in results):
            raise TypeError("generate_findings must return list[Finding]")
        for item in results:
            if (
                not re.fullmatch(r"[a-z0-9][a-z0-9_.-]*", item.id)
                or item.severity not in SEVERITY_ORDER
                or item.confidence not in {"low", "medium", "high"}
                or not isinstance(item.title, str)
                or not item.title
                or not isinstance(item.category, str)
                or not item.category
            ):
                raise ValueError("invalid finding id, severity or metadata")
            if (
                not isinstance(item.affected_topics, list)
                or any(not isinstance(topic, str) for topic in item.affected_topics)
                or not isinstance(item.evidence, list)
                or any(
                    not isinstance(e, Evidence) or not isinstance(e.metric, str)
                    for e in item.evidence
                )
                or any(e.unit is not None and not isinstance(e.unit, str) for e in item.evidence)
                or any(e.topic is not None and not isinstance(e.topic, str) for e in item.evidence)
                or (item.domain is not None and not isinstance(item.domain, str))
                or (
                    item.suggested_action is not None and not isinstance(item.suggested_action, str)
                )
            ):
                raise TypeError("invalid finding topic or evidence fields")
            if item.time_range is not None and (
                not isinstance(item.time_range, TimeRange)
                or type(item.time_range.start_ns) is not int
                or type(item.time_range.end_ns) is not int
            ):
                raise TypeError("finding time_range requires integer nanoseconds")
            json.dumps(item.to_dict(), allow_nan=False)
        return results
    except Exception as exc:
        return [_plugin_failure(plugin.name, "generate_findings", exc)]
