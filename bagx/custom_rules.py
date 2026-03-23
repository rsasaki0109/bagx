"""User-defined evaluation rules for custom topic/message conventions."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from importlib import metadata, resources
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class TopicSelector:
    """Topic selection rule based on topic name/type metadata."""

    name: str | None = None
    name_contains: str | None = None
    prefix: str | None = None
    suffix: str | None = None
    type: str | None = None
    type_contains: str | None = None


@dataclass
class CustomRuleCheck:
    """One custom rule check within a domain."""

    kind: str
    label: str
    selector: TopicSelector | None = None
    input_selector: TopicSelector | None = None
    output_selector: TopicSelector | None = None
    min_rate_hz: float | None = None
    target_ms: float | None = None
    max_response_ms: float = 2000.0
    min_samples: int = 1


@dataclass
class CustomDomainRule:
    """A custom domain detected from user-specified topic conventions."""

    name: str
    min_matches: int = 1
    match_topics: list[TopicSelector] = field(default_factory=list)
    checks: list[CustomRuleCheck] = field(default_factory=list)


@dataclass
class CustomRuleSet:
    """Loaded custom rule set."""

    source_path: str
    domains: list[CustomDomainRule] = field(default_factory=list)


@dataclass
class CustomDomainResult:
    """Evaluation result for one matched custom domain."""

    name: str
    score: float
    matched_topics: list[str] = field(default_factory=list)
    recommendations: list[str] = field(default_factory=list)


@dataclass
class RulePlugin:
    """Discoverable custom-rule plugin."""

    name: str
    source: str
    description: str = ""
    builtin: bool = False


def load_custom_rule_set(spec: str) -> CustomRuleSet:
    """Load a custom rule set from a path, plugin name, or entry point."""
    source_label, payload = _resolve_rule_payload(spec)
    if isinstance(payload, CustomRuleSet):
        if not payload.source_path:
            payload.source_path = source_label
        return payload

    raw = _coerce_rule_document(payload, source_label)
    return _parse_rule_document(raw, source_label)


def discover_rule_plugins() -> list[RulePlugin]:
    """Return available built-in and externally discoverable rule plugins."""
    plugins: dict[str, RulePlugin] = {}

    for plugin in _iter_builtin_rule_plugins():
        plugins.setdefault(plugin.name, plugin)

    for plugin in _iter_env_rule_plugins():
        plugins.setdefault(plugin.name, plugin)

    for plugin in _iter_entry_point_rule_plugins():
        plugins.setdefault(plugin.name, plugin)

    return sorted(plugins.values(), key=lambda plugin: (plugin.name.lower(), plugin.source))


def evaluate_custom_rule_set(
    topic_info: dict[str, dict[str, Any]],
    topic_timestamps: dict[str, list[int]],
    rule_set: CustomRuleSet | None,
) -> list[CustomDomainResult]:
    """Evaluate all matching custom domains against topic metadata/timestamps."""
    if rule_set is None:
        return []

    results: list[CustomDomainResult] = []
    for domain in rule_set.domains:
        matched_topic_names: list[str] = []
        matched_selectors = 0
        for selector in domain.match_topics:
            matches = select_topics(topic_info, selector)
            if matches:
                matched_selectors += 1
                matched_topic_names.extend(matches)
        if matched_selectors < domain.min_matches:
            continue

        recs = [f"[bold cyan]{domain.name} custom rules matched[/bold cyan]"]
        scores: list[float] = []
        for check in domain.checks:
            check_score = _evaluate_check(
                check=check,
                topic_info=topic_info,
                topic_timestamps=topic_timestamps,
                recommendations=recs,
            )
            if check_score is not None:
                scores.append(check_score)

        score = float(np.mean(scores)) if scores else 100.0
        results.append(
            CustomDomainResult(
                name=domain.name,
                score=score,
                matched_topics=sorted(set(matched_topic_names)),
                recommendations=recs,
            )
        )

    return results


def select_topics(topic_info: dict[str, dict[str, Any]], selector: TopicSelector) -> list[str]:
    """Select topics matching a user-defined selector, sorted by rate desc."""
    matches: list[str] = []
    for topic_name, info in topic_info.items():
        if int(info.get("count", 0) or 0) <= 0:
            continue
        if _selector_matches(topic_name, str(info.get("type", "")), selector):
            matches.append(topic_name)
    return sorted(matches, key=lambda name: (-float(topic_info[name].get("rate_hz", 0.0) or 0.0), name))


def _resolve_rule_payload(spec: str) -> tuple[str, dict[str, Any] | CustomRuleSet]:
    path = Path(os.path.expandvars(spec)).expanduser()
    if path.exists():
        with open(path) as f:
            return str(path.resolve()), json.load(f)

    builtin_plugin = _load_builtin_rule_plugin(spec)
    if builtin_plugin is not None:
        return builtin_plugin

    env_plugin = _load_env_rule_plugin(spec)
    if env_plugin is not None:
        return env_plugin

    entry_point_plugin = _load_entry_point_rule_plugin(spec)
    if entry_point_plugin is not None:
        return entry_point_plugin

    known_plugins = ", ".join(plugin.name for plugin in discover_rule_plugins())
    if known_plugins:
        raise FileNotFoundError(
            f"Custom rule file or plugin not found: {spec}. Available plugins: {known_plugins}"
        )
    raise FileNotFoundError(f"Custom rule file or plugin not found: {spec}")


def _parse_rule_document(raw: dict[str, Any], source_label: str) -> CustomRuleSet:
    domains: list[CustomDomainRule] = []
    for domain in raw.get("domains", []):
        name = str(domain.get("name", "")).strip()
        if not name:
            raise ValueError(f"custom rule domain is missing a name in {source_label}")

        match_topics = [
            _parse_selector(item)
            for item in domain.get("match_topics", domain.get("topics", []))
        ]
        checks = [_parse_check(item) for item in domain.get("checks", [])]
        min_matches = int(domain.get("min_matches", len(match_topics) if match_topics else 0))
        domains.append(
            CustomDomainRule(
                name=name,
                min_matches=max(min_matches, 0),
                match_topics=match_topics,
                checks=checks,
            )
        )

    return CustomRuleSet(source_path=source_label, domains=domains)


def _coerce_rule_document(payload: Any, source_label: str) -> dict[str, Any]:
    if isinstance(payload, dict):
        return payload
    if isinstance(payload, (str, os.PathLike)):
        payload_path = Path(payload).expanduser()
        with open(payload_path) as f:
            return json.load(f)
    raise ValueError(
        f"Unsupported custom rule payload from {source_label}: expected dict, path, or CustomRuleSet"
    )


def _iter_builtin_rule_plugins() -> list[RulePlugin]:
    plugins: list[RulePlugin] = []
    for path, raw in _iter_builtin_rule_documents():
        name = str(raw.get("plugin_name", path.stem))
        plugins.append(
            RulePlugin(
                name=name,
                source=f"builtin:{path.stem}",
                description=str(raw.get("description", "")).strip(),
                builtin=True,
            )
        )
    return plugins


def _iter_builtin_rule_documents() -> list[tuple[Path, dict[str, Any]]]:
    documents: list[tuple[Path, dict[str, Any]]] = []
    try:
        root = resources.files("bagx").joinpath("rule_plugins")
    except (ModuleNotFoundError, FileNotFoundError):
        return documents

    try:
        entries = sorted(root.iterdir(), key=lambda item: item.name)
    except FileNotFoundError:
        return documents

    for entry in entries:
        if not entry.name.endswith(".json"):
            continue
        try:
            text = entry.read_text(encoding="utf-8")
            documents.append((Path(entry.name), json.loads(text)))
        except Exception:
            continue
    return documents


def _load_builtin_rule_plugin(name: str) -> tuple[str, dict[str, Any]] | None:
    normalized = name.strip().lower()
    for path, raw in _iter_builtin_rule_documents():
        plugin_name = str(raw.get("plugin_name", path.stem)).strip()
        if normalized in {path.stem.lower(), plugin_name.lower(), f"builtin:{path.stem.lower()}"}:
            return f"builtin:{path.stem}", raw
    return None


def _iter_env_rule_plugins() -> list[RulePlugin]:
    plugins: list[RulePlugin] = []
    for directory in _iter_rule_plugin_dirs():
        for path in sorted(directory.glob("*.json")):
            try:
                raw = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            name = str(raw.get("plugin_name", path.stem))
            plugins.append(
                RulePlugin(
                    name=name,
                    source=str(path),
                    description=str(raw.get("description", "")).strip(),
                )
            )
    return plugins


def _load_env_rule_plugin(name: str) -> tuple[str, dict[str, Any]] | None:
    normalized = name.strip().lower()
    for directory in _iter_rule_plugin_dirs():
        for path in sorted(directory.glob("*.json")):
            if path.stem.lower() != normalized:
                try:
                    raw = json.loads(path.read_text(encoding="utf-8"))
                except Exception:
                    continue
                plugin_name = str(raw.get("plugin_name", path.stem)).strip().lower()
                if plugin_name != normalized:
                    continue
                return str(path), raw
            raw = json.loads(path.read_text(encoding="utf-8"))
            return str(path), raw
    return None


def _iter_rule_plugin_dirs() -> list[Path]:
    raw = os.environ.get("BAGX_RULE_PLUGIN_PATH", "")
    if not raw:
        return []
    directories: list[Path] = []
    for entry in raw.split(os.pathsep):
        if not entry.strip():
            continue
        path = Path(os.path.expandvars(entry)).expanduser()
        if path.is_dir():
            directories.append(path)
    return directories


def _iter_entry_point_rule_plugins() -> list[RulePlugin]:
    plugins: list[RulePlugin] = []
    for entry_point in _iter_rule_entry_points():
        description = ""
        try:
            payload = _load_entry_point_payload(entry_point)
            if isinstance(payload, dict):
                description = str(payload.get("description", "")).strip()
        except Exception:
            description = ""
        plugins.append(
            RulePlugin(
                name=entry_point.name,
                source=f"entrypoint:{entry_point.value}",
                description=description,
            )
        )
    return plugins


def _load_entry_point_rule_plugin(name: str) -> tuple[str, dict[str, Any] | CustomRuleSet] | None:
    normalized = name.strip().lower()
    for entry_point in _iter_rule_entry_points():
        if entry_point.name.lower() != normalized:
            continue
        return f"entrypoint:{entry_point.value}", _load_entry_point_payload(entry_point)
    return None


def _iter_rule_entry_points() -> list[metadata.EntryPoint]:
    try:
        return sorted(metadata.entry_points(group="bagx.rules"), key=lambda item: item.name.lower())
    except TypeError:
        entry_points = metadata.entry_points()
        return sorted(entry_points.get("bagx.rules", []), key=lambda item: item.name.lower())


def _load_entry_point_payload(entry_point: metadata.EntryPoint) -> dict[str, Any] | CustomRuleSet:
    loaded = entry_point.load()
    payload = loaded() if callable(loaded) else loaded
    if isinstance(payload, CustomRuleSet):
        return payload
    return _coerce_rule_document(payload, f"entrypoint:{entry_point.value}")


def _parse_selector(data: dict[str, Any]) -> TopicSelector:
    if not isinstance(data, dict):
        raise ValueError(f"topic selector must be an object, got: {type(data)!r}")
    return TopicSelector(
        name=_optional_text(data.get("name")),
        name_contains=_optional_text(data.get("name_contains")),
        prefix=_optional_text(data.get("prefix")),
        suffix=_optional_text(data.get("suffix")),
        type=_optional_text(data.get("type")),
        type_contains=_optional_text(data.get("type_contains")),
    )


def _parse_check(data: dict[str, Any]) -> CustomRuleCheck:
    if not isinstance(data, dict):
        raise ValueError(f"custom rule check must be an object, got: {type(data)!r}")
    kind = str(data.get("kind", "")).strip()
    label = str(data.get("label", "")).strip()
    if not kind or not label:
        raise ValueError("custom rule check requires both 'kind' and 'label'")

    selector = _parse_selector(data["selector"]) if "selector" in data else None
    input_selector = _parse_selector(data["input"]) if "input" in data else None
    output_selector = _parse_selector(data["output"]) if "output" in data else None
    min_rate_hz = float(data["min_rate_hz"]) if "min_rate_hz" in data else None
    target_ms = float(data["target_ms"]) if "target_ms" in data else None
    max_response_ms = float(data.get("max_response_ms", 2000.0))
    min_samples = int(data.get("min_samples", 1))

    if kind in {"topic_exists", "topic_rate"} and selector is None:
        raise ValueError(f"{kind} check requires 'selector'")
    if kind == "topic_rate" and min_rate_hz is None:
        raise ValueError("topic_rate check requires 'min_rate_hz'")
    if kind == "latency":
        if input_selector is None or output_selector is None:
            raise ValueError("latency check requires both 'input' and 'output'")
        if target_ms is None:
            raise ValueError("latency check requires 'target_ms'")
    if kind not in {"topic_exists", "topic_rate", "latency"}:
        raise ValueError(f"unsupported custom rule check kind: {kind}")

    return CustomRuleCheck(
        kind=kind,
        label=label,
        selector=selector,
        input_selector=input_selector,
        output_selector=output_selector,
        min_rate_hz=min_rate_hz,
        target_ms=target_ms,
        max_response_ms=max_response_ms,
        min_samples=max(min_samples, 1),
    )


def _evaluate_check(
    *,
    check: CustomRuleCheck,
    topic_info: dict[str, dict[str, Any]],
    topic_timestamps: dict[str, list[int]],
    recommendations: list[str],
) -> float | None:
    if check.kind == "topic_exists":
        matches = select_topics(topic_info, check.selector) if check.selector else []
        if matches:
            recommendations.append(
                f"  [green]:heavy_check_mark:[/green] {check.label} ({matches[0]}) recorded — custom rule matched"
            )
            return 100.0
        recommendations.append(
            f"  [yellow]:warning:[/yellow] {check.label} missing — custom rule not satisfied"
        )
        return 0.0

    if check.kind == "topic_rate":
        matches = select_topics(topic_info, check.selector) if check.selector else []
        if not matches:
            recommendations.append(
                f"  [yellow]:warning:[/yellow] {check.label} missing — custom rule not satisfied"
            )
            return 0.0
        topic_name = matches[0]
        rate_hz = float(topic_info[topic_name].get("rate_hz", 0.0) or 0.0)
        score = _rate_goal_score(rate_hz, float(check.min_rate_hz or 0.0))
        if rate_hz >= float(check.min_rate_hz or 0.0):
            recommendations.append(
                f"  [green]:heavy_check_mark:[/green] {check.label} ({topic_name}) at {_format_rate_hz(rate_hz)}Hz — custom rule satisfied"
            )
        else:
            recommendations.append(
                f"  [yellow]:warning:[/yellow] {check.label} ({topic_name}) at {_format_rate_hz(rate_hz)}Hz — {float(check.min_rate_hz or 0.0):.0f}Hz+ expected by custom rule"
            )
        return score

    if check.kind == "latency":
        input_matches = select_topics(topic_info, check.input_selector) if check.input_selector else []
        output_matches = select_topics(topic_info, check.output_selector) if check.output_selector else []
        if not input_matches or not output_matches:
            recommendations.append(
                f"  [yellow]:warning:[/yellow] {check.label} latency could not be evaluated — required topics are missing"
            )
            return 0.0

        input_topic = input_matches[0]
        output_topic = output_matches[0]
        latencies_ms = _compute_next_response_latencies_ms(
            topic_timestamps.get(input_topic, []),
            topic_timestamps.get(output_topic, []),
            max_response_ms=check.max_response_ms,
        )
        if len(latencies_ms) < check.min_samples:
            recommendations.append(
                f"  [yellow]:warning:[/yellow] {check.label} latency could not be evaluated — not enough samples"
            )
            return 0.0

        median_lat = float(np.median(latencies_ms))
        p95_lat = float(np.percentile(latencies_ms, 95))
        score = _latency_goal_score(median_lat, float(check.target_ms or 1.0))
        sample_note = ""
        if len(latencies_ms) < 5:
            plural = "s" if len(latencies_ms) != 1 else ""
            sample_note = f" ({len(latencies_ms)} sample{plural})"
        if median_lat <= float(check.target_ms or 0.0):
            recommendations.append(
                f"  [green]:heavy_check_mark:[/green] Pipeline {check.label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note}"
            )
        elif median_lat <= float(check.target_ms or 0.0) * 4:
            recommendations.append(
                f"  [yellow]:warning:[/yellow] Pipeline {check.label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — slower than custom target"
            )
        else:
            recommendations.append(
                f"  [red]:x:[/red] Pipeline {check.label}: {median_lat:.0f}ms median, {p95_lat:.0f}ms P95{sample_note} — far slower than custom target"
            )
        return score

    raise ValueError(f"unsupported custom rule check kind: {check.kind}")


def _selector_matches(topic_name: str, topic_type: str, selector: TopicSelector) -> bool:
    lower_name = topic_name.lower()
    lower_type = topic_type.lower()

    if selector.name and lower_name != selector.name.lower():
        return False
    if selector.name_contains and selector.name_contains.lower() not in lower_name:
        return False
    if selector.prefix and not lower_name.startswith(selector.prefix.lower()):
        return False
    if selector.suffix and not lower_name.endswith(selector.suffix.lower()):
        return False
    if selector.type and lower_type != selector.type.lower():
        return False
    if selector.type_contains and selector.type_contains.lower() not in lower_type:
        return False
    return True


def _compute_next_response_latencies_ms(
    input_timestamps: list[int],
    output_timestamps: list[int],
    *,
    max_response_ms: float,
) -> list[float]:
    if not input_timestamps or not output_timestamps:
        return []

    arr_in = np.array(sorted(input_timestamps), dtype=np.int64)
    arr_out = np.array(sorted(output_timestamps), dtype=np.int64)
    latencies_ms: list[float] = []
    for t_in in arr_in:
        idx = int(np.searchsorted(arr_out, t_in, side="right"))
        if idx >= len(arr_out):
            continue
        latency_ms = float((arr_out[idx] - t_in) / 1e6)
        if 0 <= latency_ms <= max_response_ms:
            latencies_ms.append(latency_ms)
    return latencies_ms


def _rate_goal_score(rate_hz: float, target_hz: float) -> float:
    if rate_hz <= 0 or target_hz <= 0:
        return 0.0
    return float(max(0.0, min(100.0, (rate_hz / target_hz) * 100.0)))


def _latency_goal_score(median_ms: float, target_ms: float) -> float:
    if median_ms <= 0 or target_ms <= 0:
        return 0.0
    return float(max(0.0, min(100.0, (target_ms / median_ms) * 100.0)))


def _format_rate_hz(rate_hz: float) -> str:
    if rate_hz < 20 and abs(rate_hz - round(rate_hz)) >= 0.05:
        return f"{rate_hz:.1f}"
    return f"{rate_hz:.0f}"


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None
