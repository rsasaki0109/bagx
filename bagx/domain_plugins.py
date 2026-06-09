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
import logging
from dataclasses import dataclass, field
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

    def detect(self, topic_info: dict[str, dict]) -> bool:
        return bool(self.detect_fn(topic_info))

    def representative_topics(self, topic_info: dict[str, dict]) -> list[str]:
        return list(self.representative_fn(topic_info))

    def generate_findings(self, report: "EvalReport") -> list["Finding"]:
        return list(self.findings_fn(report))


def _builtin_plugins() -> list[DomainPlugin]:
    """Return the built-in domain plugins.

    Imports are deferred so :mod:`bagx.eval` can import this module without
    creating a circular dependency.
    """
    from bagx.eval import (
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
        controller_action = _select_topics(topics, contains=("follow_joint_trajectory/_action/status",))
        evidence = sum(
            bool(m)
            for m in [joint, planned, scene, move_action, execute_action, controller_action]
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

    from bagx.eval_findings import _generate_nav2_findings_from_plugin

    return [
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
        ),
        SimpleDomainPlugin(
            name="MoveIt",
            detect_fn=_detect_moveit,
            representative_fn=_moveit_topics,
        ),
        SimpleDomainPlugin(
            name="Perception",
            detect_fn=_detect_perception,
            representative_fn=_image_summary_topics,
        ),
        SimpleDomainPlugin(
            name="RobotArm",
            detect_fn=_detect_robotarm,
            representative_fn=_image_summary_topics,
        ),
        SimpleDomainPlugin(
            name="Control",
            detect_fn=_detect_control,
            representative_fn=_all_topics,
        ),
    ]


def _entry_point_plugins() -> list[DomainPlugin]:
    plugins: list[DomainPlugin] = []
    try:
        entry_points = _md.entry_points(group=ENTRY_POINT_GROUP)
    except TypeError:
        # Older importlib.metadata API: entry_points() returns a dict.
        entry_points = _md.entry_points().get(ENTRY_POINT_GROUP, [])  # type: ignore[assignment]
    for entry_point in entry_points:
        try:
            obj: Any = entry_point.load()
        except Exception as exc:  # pragma: no cover - defensive
            logger.warning(
                "Failed to load bagx.domains plugin %s: %s", entry_point.name, exc
            )
            continue
        instance = obj() if callable(obj) and not _looks_like_plugin(obj) else obj
        if not _looks_like_plugin(instance):
            logger.warning(
                "bagx.domains entry point %r does not implement DomainPlugin",
                entry_point.name,
            )
            continue
        plugins.append(instance)
    return plugins


def _looks_like_plugin(obj: Any) -> bool:
    return (
        hasattr(obj, "name")
        and callable(getattr(obj, "detect", None))
        and callable(getattr(obj, "representative_topics", None))
        and callable(getattr(obj, "generate_findings", None))
    )


def discover_domain_plugins() -> list[DomainPlugin]:
    """Return built-in plugins followed by entry-point installed plugins."""
    return [*_builtin_plugins(), *_entry_point_plugins()]


def plugin_source_label(plugin: DomainPlugin) -> str:
    """Return ``built-in`` or ``entry-point`` for CLI display."""
    builtin_names = {item.name for item in _builtin_plugins()}
    return "built-in" if plugin.name in builtin_names else "entry-point"


def detect_domains(
    topic_info: dict[str, dict],
    plugins: list[DomainPlugin] | None = None,
) -> list[DomainPlugin]:
    """Return the plugins whose ``detect`` returns True for ``topic_info``.

    Order is preserved from ``discover_domain_plugins`` (built-ins first, then
    entry-point plugins) which gives downstream code a stable iteration order.
    """
    if not topic_info:
        return []
    candidates = plugins if plugins is not None else discover_domain_plugins()
    return [plugin for plugin in candidates if plugin.detect(topic_info)]
