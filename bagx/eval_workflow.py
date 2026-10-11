"""Action / service workflow outcome summaries for bagx eval."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Iterable

import numpy as np

from bagx.eval_topics import (
    _select_action_result_topics,
    _select_action_status_topics,
    _select_service_event_topics,
)

@dataclass
class WorkflowMetrics:
    topic: str
    kind: str
    observed_events: int = 0
    completed_events: int = 0
    success_events: int = 0
    failure_events: int = 0
    unknown_events: int = 0
    request_events: int = 0
    response_events: int = 0
    success_rate: float | None = None
    response_rate: float | None = None
    score: float | None = None
    failure_reasons: list[str] = field(default_factory=list)


def _matching_workflow_metrics(
    metrics: list[WorkflowMetrics],
    *,
    contains: tuple[str, ...],
) -> list[WorkflowMetrics]:
    lowered = tuple(fragment.lower() for fragment in contains)
    return [
        metric
        for metric in metrics
        if any(fragment in metric.topic.lower() for fragment in lowered)
    ]


def _summarize_workflow_metrics(
    workflow_messages: dict[str, list[dict[str, Any]]],
    topic_info: dict[str, dict[str, Any]],
) -> list[WorkflowMetrics]:
    metrics: list[WorkflowMetrics] = []
    for topic in _select_action_status_topics(topic_info):
        metrics.append(_summarize_action_status_topic(topic, workflow_messages.get(topic, [])))
    for topic in _select_action_result_topics(topic_info):
        metrics.append(_summarize_action_result_topic(topic, workflow_messages.get(topic, [])))
    for topic in _select_service_event_topics(topic_info):
        metrics.append(_summarize_service_event_topic(topic, workflow_messages.get(topic, [])))
    return metrics


def _summarize_action_status_topic(topic: str, messages: list[dict[str, Any]]) -> WorkflowMetrics:
    final_status_by_goal: dict[str, tuple[int, str | None]] = {}
    anonymous_counter = 0

    for message_index, data in enumerate(messages):
        for entry_index, entry in enumerate(_extract_goal_status_entries(data)):
            status = entry.get("status")
            if status is None:
                continue
            goal_id = entry.get("goal_id")
            if not goal_id:
                anonymous_counter += 1
                goal_id = f"{topic}:{message_index}:{entry_index}:{anonymous_counter}"
            final_status_by_goal[goal_id] = (status, entry.get("reason"))

    success = 0
    failure = 0
    unknown = 0
    failure_reasons: Counter[str] = Counter()
    for status, reason_hint in final_status_by_goal.values():
        state, reason = _classify_action_status(status)
        if state == "success":
            success += 1
        elif state == "failure":
            failure += 1
            failure_reason = reason_hint or reason
            if failure_reason:
                failure_reasons[failure_reason] += 1
        else:
            unknown += 1

    completed = success + failure
    success_rate = (success / completed) if completed else None
    return WorkflowMetrics(
        topic=topic,
        kind="action_status",
        observed_events=len(final_status_by_goal),
        completed_events=completed,
        success_events=success,
        failure_events=failure,
        unknown_events=unknown,
        success_rate=success_rate,
        score=(success_rate * 100.0) if success_rate is not None else None,
        failure_reasons=_top_failure_reasons(failure_reasons),
    )


def _summarize_action_result_topic(topic: str, messages: list[dict[str, Any]]) -> WorkflowMetrics:
    success = 0
    failure = 0
    unknown = 0
    failure_reasons: Counter[str] = Counter()

    for data in messages:
        state, reason = _classify_workflow_payload(data)
        if state == "success":
            success += 1
        elif state == "failure":
            failure += 1
            if reason:
                failure_reasons[reason] += 1
        else:
            unknown += 1

    completed = success + failure
    success_rate = (success / completed) if completed else None
    return WorkflowMetrics(
        topic=topic,
        kind="action_result",
        observed_events=len(messages),
        completed_events=completed,
        success_events=success,
        failure_events=failure,
        unknown_events=unknown,
        success_rate=success_rate,
        score=(success_rate * 100.0) if success_rate is not None else None,
        failure_reasons=_top_failure_reasons(failure_reasons),
    )


def _summarize_service_event_topic(topic: str, messages: list[dict[str, Any]]) -> WorkflowMetrics:
    request_events = 0
    response_events = 0
    success = 0
    failure = 0
    unknown = 0
    failure_reasons: Counter[str] = Counter()

    for data in messages:
        requests = _ensure_sequence(data.get("request"))
        responses = _ensure_sequence(data.get("response"))
        request_events += len(requests)
        response_events += len(responses)

        for response in responses:
            state, reason = _classify_workflow_payload(response if isinstance(response, dict) else {"value": response})
            if state == "success":
                success += 1
            elif state == "failure":
                failure += 1
                if reason:
                    failure_reasons[reason] += 1
            else:
                unknown += 1

    completed = success + failure
    success_rate = (success / completed) if completed else None
    response_rate = (response_events / request_events) if request_events else None
    score_candidates = [
        rate * 100.0
        for rate in (success_rate, response_rate)
        if rate is not None
    ]
    score = float(np.mean(score_candidates)) if score_candidates else None
    return WorkflowMetrics(
        topic=topic,
        kind="service_event",
        observed_events=len(messages),
        completed_events=completed,
        success_events=success,
        failure_events=failure,
        unknown_events=unknown,
        request_events=request_events,
        response_events=response_events,
        success_rate=success_rate,
        response_rate=response_rate,
        score=score,
        failure_reasons=_top_failure_reasons(failure_reasons),
    )


def _generate_workflow_recommendations(metrics: list[WorkflowMetrics]) -> list[str]:
    recs: list[str] = []
    for metric in sorted(metrics, key=lambda item: (item.kind, item.topic)):
        if metric.kind == "action_status":
            if metric.completed_events == 0 and metric.observed_events > 0:
                recs.append(
                    f"  [dim]:information_source:[/dim] Action lifecycle ({metric.topic}) was recorded, but no terminal completion status was observed"
                )
                continue
            if metric.completed_events == 0:
                continue
            detail = f"{metric.success_events}/{metric.completed_events} terminal goals succeeded"
            if metric.failure_reasons:
                detail += f" — failures include {', '.join(metric.failure_reasons)}"
            if metric.failure_events == 0:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Action completion ({metric.topic}): {detail}"
                )
            elif metric.success_rate is not None and metric.success_rate >= 0.5:
                recs.append(
                    f"  [yellow]:warning:[/yellow] Action completion ({metric.topic}): {detail}"
                )
            else:
                recs.append(
                    f"  [red]:x:[/red] Action completion ({metric.topic}): {detail}"
                )

        elif metric.kind == "action_result":
            if metric.completed_events == 0:
                if metric.observed_events > 0:
                    recs.append(
                        f"  [dim]:information_source:[/dim] Action result ({metric.topic}) was recorded, but no explicit success/failure field could be decoded"
                    )
                continue
            detail = f"{metric.success_events}/{metric.completed_events} results succeeded"
            if metric.failure_reasons:
                detail += f" — failures include {', '.join(metric.failure_reasons)}"
            if metric.failure_events == 0:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Action result ({metric.topic}): {detail}"
                )
            elif metric.success_rate is not None and metric.success_rate >= 0.5:
                recs.append(
                    f"  [yellow]:warning:[/yellow] Action result ({metric.topic}): {detail}"
                )
            else:
                recs.append(
                    f"  [red]:x:[/red] Action result ({metric.topic}): {detail}"
                )

        elif metric.kind == "service_event":
            if metric.request_events == 0:
                continue
            coverage = f"{metric.response_events}/{metric.request_events} requests have responses"
            if metric.completed_events > 0:
                coverage += f"; {metric.success_events}/{metric.completed_events} decoded responses succeeded"
            if metric.failure_reasons:
                coverage += f" — failures include {', '.join(metric.failure_reasons)}"
            if metric.response_rate == 1.0 and metric.failure_events == 0:
                recs.append(
                    f"  [green]:heavy_check_mark:[/green] Service responses ({metric.topic}): {coverage}"
                )
            elif metric.response_rate is not None and metric.response_rate >= 0.7:
                recs.append(
                    f"  [yellow]:warning:[/yellow] Service responses ({metric.topic}): {coverage}"
                )
            else:
                recs.append(
                    f"  [red]:x:[/red] Service responses ({metric.topic}): {coverage}"
                )
    return recs


def _extract_goal_status_entries(data: dict[str, Any]) -> list[dict[str, Any]]:
    entries = _ensure_sequence(data.get("status_list"))
    results: list[dict[str, Any]] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        status = _coerce_int(entry.get("status"))
        if status is None:
            continue
        results.append(
            {
                "goal_id": _extract_goal_id(entry),
                "status": status,
                "reason": _extract_reason_from_mapping(entry),
            }
        )
    return results


def _extract_goal_id(entry: dict[str, Any]) -> str | None:
    goal_info = entry.get("goal_info")
    if not isinstance(goal_info, dict):
        return None
    goal_id = goal_info.get("goal_id")
    if isinstance(goal_id, dict):
        uuid = goal_id.get("uuid")
        if isinstance(uuid, (list, tuple)):
            values = [value for value in uuid if isinstance(value, int)]
            if values:
                return "".join(f"{value:02x}" for value in values)
        if isinstance(uuid, (bytes, bytearray)):
            return uuid.hex()
    if isinstance(goal_id, str) and goal_id.strip():
        return goal_id.strip()
    return None


def _classify_action_status(status: int) -> tuple[str, str | None]:
    if status == 4:
        return "success", None
    if status == 5:
        return "failure", "canceled"
    if status == 6:
        return "failure", "aborted"
    return "unknown", None


def _classify_workflow_payload(payload: Any) -> tuple[str, str | None]:
    fields = list(_walk_scalar_fields(payload))
    bool_success = _find_bool_field(fields, {"success", "succeeded", "ok"})
    error_code = _find_failure_code(fields)
    status_code = _find_numeric_field(fields, {"status", "goal_status", "status_code"})
    reason = _find_reason_text(fields)
    code_detail = _format_failure_code_detail(error_code)

    if bool_success is False:
        return "failure", _merge_failure_detail(reason, code_detail) or "reported failure"
    if error_code is not None and error_code != 0:
        return "failure", _merge_failure_detail(reason, code_detail)
    if status_code is not None:
        status_state, status_reason = _classify_action_status(status_code)
        if status_state == "failure":
            return "failure", _merge_failure_detail(reason, code_detail) or status_reason
    if bool_success is True:
        return "success", None
    if status_code is not None:
        status_state, _ = _classify_action_status(status_code)
        if status_state == "success":
            return "success", None
    if error_code == 0:
        return "success", None
    return "unknown", None


def _walk_scalar_fields(payload: Any, prefix: str = "") -> Iterable[tuple[str, Any]]:
    if isinstance(payload, dict):
        for key, value in payload.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            yield from _walk_scalar_fields(value, path)
        return
    if isinstance(payload, (list, tuple)):
        for index, value in enumerate(payload):
            path = f"{prefix}[{index}]" if prefix else f"[{index}]"
            yield from _walk_scalar_fields(value, path)
        return
    yield prefix, payload


def _find_bool_field(fields: list[tuple[str, Any]], names: set[str]) -> bool | None:
    for path, value in sorted(fields, key=lambda item: _field_priority(item[0]), reverse=True):
        last = _last_field_token(path)
        if last not in names:
            continue
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)) and value in {0, 1}:
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in {"true", "success", "succeeded", "ok"}:
                return True
            if lowered in {"false", "failure", "failed", "error"}:
                return False
    return None


def _find_numeric_field(fields: list[tuple[str, Any]], names: set[str]) -> int | None:
    for path, value in sorted(fields, key=lambda item: _field_priority(item[0]), reverse=True):
        if _last_field_token(path) not in names:
            continue
        coerced = _coerce_int(value)
        if coerced is not None:
            return coerced
    return None


def _find_failure_code(fields: list[tuple[str, Any]]) -> int | None:
    candidate_tokens = {"error_code", "return_code", "result_code", "code", "val"}
    best_match: tuple[int, int] | None = None
    for path, value in fields:
        token = _last_field_token(path)
        if token not in candidate_tokens:
            continue
        if token == "val" and not _path_has_any(path, ("error_code", "return_code", "result_code")):
            continue
        if token == "code" and not _path_has_any(path, ("error", "fail", "abort", "cancel", "result")):
            continue
        coerced = _coerce_int(value)
        if coerced is None:
            continue
        priority = _field_priority(path)
        if best_match is None or priority > best_match[0]:
            best_match = (priority, coerced)
    return best_match[1] if best_match is not None else None


def _find_reason_text(fields: list[tuple[str, Any]]) -> str | None:
    preferred_names = {
        "error_string",
        "error_message",
        "failure_reason",
        "reason",
        "message",
        "details",
        "text",
        "description",
        "comment",
        "diagnostic_message",
    }
    best_match: tuple[int, str] | None = None
    for path, value in fields:
        if _last_field_token(path) not in preferred_names:
            continue
        if not isinstance(value, str):
            continue
        text = value.strip()
        if not text:
            continue
        if text.lower() in {"ok", "success", "succeeded", "done"}:
            continue
        priority = _field_priority(path)
        if best_match is None or priority > best_match[0]:
            best_match = (priority, text[:120])
    return best_match[1] if best_match is not None else None


def _extract_reason_from_mapping(data: dict[str, Any]) -> str | None:
    fields = list(_walk_scalar_fields(data))
    return _merge_failure_detail(
        _find_reason_text(fields),
        _format_failure_code_detail(_find_failure_code(fields)),
    )


def _merge_failure_detail(reason: str | None, code_detail: str | None) -> str | None:
    if reason and code_detail and code_detail not in reason:
        return f"{reason} ({code_detail})"
    return reason or code_detail


def _format_failure_code_detail(code: int | None) -> str | None:
    if code is None or code == 0:
        return None
    return f"error_code={code}"


def _field_priority(path: str) -> int:
    score = 0
    lowered = path.lower()
    if _path_has_any(lowered, ("error", "fail", "abort", "cancel")):
        score += 6
    if _path_has_any(lowered, ("response", "result")):
        score += 4
    if _path_has_any(lowered, ("status_list", "goal_info")):
        score += 2
    if _path_has_any(lowered, ("detail", "reason", "message", "text", "description")):
        score += 1
    return score


def _path_has_any(path: str, fragments: tuple[str, ...]) -> bool:
    lowered = path.lower()
    return any(fragment in lowered for fragment in fragments)


def _last_field_token(path: str) -> str:
    token = path.split(".")[-1]
    if "[" in token:
        token = token.split("[", 1)[0]
    return token.lower()


def _coerce_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        text = value.strip()
        if text.startswith(("+", "-")):
            sign = text[0]
            digits = text[1:]
            if digits.isdigit():
                return int(sign + digits)
        if text.isdigit():
            return int(text)
    return None


def _ensure_sequence(value: Any) -> list[Any]:
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    if isinstance(value, dict):
        return [value]
    return []


def _top_failure_reasons(counter: Counter[str]) -> list[str]:
    return [reason for reason, _count in counter.most_common(3)]
