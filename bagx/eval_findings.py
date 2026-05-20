"""Structured finding generation for eval reports.

Findings are stable, machine-readable summaries of readiness checks. They are
designed to be benchmark-gateable without depending on human-facing
recommendation text.

Finding id naming policy
------------------------
- Lowercase, dot-separated: ``<domain>.<area>.<qualifier>``.
  Examples: ``gnss.fix_rate.good``, ``imu.accel_noise.noisy``, ``nav2.detected``.
- Qualifiers encode the observed state (``good`` / ``gappy`` / ``noisy`` / ``low``
  / ``high`` / ``rate_low`` / ``missing_*``) so passing states are also
  benchmark-checkable.
- Ids are constructed with :func:`bagx.findings.finding_id` to keep them
  ASCII-safe and stable across releases. Topic-derived tokens go through
  :func:`bagx.findings.clean_topic_token`.

Severity policy
---------------
- ``info`` — readiness signal is healthy or merely informational.
- ``warning`` — readiness is degraded but the bag is still usable for the
  detected domain; user action recommended.
- ``error`` — readiness is unreliable; the bag should not be used as-is for
  the detected domain.
- ``critical`` — reserved for catastrophic readiness loss (e.g. mandatory
  topic missing on a safety-critical pipeline). Not emitted by the
  built-in checks today, but defined so downstream plugins can use it
  without a schema bump.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Iterable

import numpy as np

from bagx.findings import Evidence, Finding, clean_topic_token, finding_id

if TYPE_CHECKING:
    from bagx.custom_rules import CustomDomainResult
    from bagx.eval import EvalReport, WorkflowMetrics


def generate_findings(report: EvalReport, domain_names: set[str]) -> list[Finding]:
    """Generate structured findings for JSON reports and benchmark contracts."""
    findings: list[Finding] = []
    topics = report.topic_info
    is_non_slam_domain = bool(domain_names)

    for domain_name in sorted(domain_names):
        findings.append(
            Finding(
                id=finding_id(_domain_id(domain_name), "detected"),
                title=f"{domain_name} topics detected",
                severity="info",
                category="domain_detection",
                domain=_domain_id(domain_name),
                affected_topics=_domain_representative_topics(domain_name, topics),
                evidence=[Evidence(metric="domain_detected", observed=True, expected=True)],
                confidence="high",
            )
        )

    findings.extend(_generate_gnss_findings(report, is_non_slam_domain))
    findings.extend(_generate_imu_findings(report, is_non_slam_domain))
    findings.extend(_generate_sync_findings(report, is_non_slam_domain))
    findings.extend(_generate_nav2_findings(report, domain_names))
    findings.extend(_generate_custom_domain_findings(report.custom_domains))
    findings.extend(_generate_workflow_findings(report.workflow_metrics))

    return findings


def _generate_gnss_findings(report: EvalReport, is_non_slam_domain: bool) -> list[Finding]:
    findings: list[Finding] = []
    if report.gnss:
        g = report.gnss
        if g.fix_rate >= 0.95:
            severity = "info"
            suffix = "good"
            title = "GNSS fix rate is suitable as a reference"
            action = None
        elif g.fix_rate >= 0.5:
            severity = "warning"
            suffix = "gappy"
            title = "GNSS fix rate has gaps"
            action = "Check GNSS visibility before using this bag as a ground-truth reference."
        else:
            severity = "error"
            suffix = "unreliable"
            title = "GNSS fix rate is unreliable"
            action = "Do not use this GNSS stream as ground truth without another reference."
        findings.append(
            Finding(
                id=finding_id("gnss", "fix_rate", suffix),
                title=title,
                severity=severity,
                category="sensor_quality",
                domain="slam" if not is_non_slam_domain else None,
                evidence=[
                    Evidence(metric="fix_rate", observed=round(g.fix_rate, 4), expected=">=0.95"),
                    Evidence(metric="fix_count", observed=g.fix_count),
                    Evidence(metric="no_fix_count", observed=g.no_fix_count),
                ],
                suggested_action=action,
                confidence="high",
            )
        )
        if not math.isnan(g.hdop_mean) and g.hdop_mean > 5.0:
            findings.append(
                Finding(
                    id=finding_id("gnss", "hdop", "high"),
                    title="GNSS HDOP is high",
                    severity="warning",
                    category="sensor_quality",
                    domain="slam" if not is_non_slam_domain else None,
                    evidence=[
                        Evidence(metric="hdop_mean", observed=round(g.hdop_mean, 3), expected="<=5.0"),
                        Evidence(metric="hdop_max", observed=round(g.hdop_max, 3)),
                    ],
                    suggested_action="Treat GNSS position accuracy as degraded for this bag.",
                    confidence="medium",
                )
            )
    elif not is_non_slam_domain:
        findings.append(
            Finding(
                id=finding_id("gnss", "missing"),
                title="No GNSS data recorded",
                severity="info",
                category="topic_presence",
                domain="slam",
                evidence=[Evidence(metric="topic_present", observed=False, expected=True)],
                suggested_action="Use an external reference if ground truth is required.",
                confidence="high",
            )
        )
    return findings


def _generate_imu_findings(report: EvalReport, is_non_slam_domain: bool) -> list[Finding]:
    findings: list[Finding] = []
    if report.imu:
        m = report.imu
        accel_noise = _mean_valid([m.accel_noise_x, m.accel_noise_y, m.accel_noise_z])
        gyro_noise = _mean_valid([m.gyro_noise_x, m.gyro_noise_y, m.gyro_noise_z])
        domain = None if is_non_slam_domain else "slam"
        affected_topics = [report.imu_topic] if report.imu_topic else []

        if not math.isnan(accel_noise):
            if accel_noise < 0.2:
                severity = "info"
                suffix = "good"
                title = "IMU accelerometer noise is usable"
                action = None
            else:
                severity = "warning"
                suffix = "noisy"
                title = "IMU accelerometer noise is high"
                action = (
                    "Check IMU calibration."
                    if is_non_slam_domain
                    else "Consider LiDAR-only odometry or lower IMU trust in LIO."
                )
            findings.append(
                Finding(
                    id=finding_id("imu", "accel_noise", suffix),
                    title=title,
                    severity=severity,
                    category="sensor_quality",
                    domain=domain,
                    affected_topics=affected_topics,
                    evidence=[
                        Evidence(
                            metric="accel_noise_mean",
                            observed=round(accel_noise, 6),
                            expected="<0.2",
                            unit="m/s^2",
                            topic=report.imu_topic or None,
                        )
                    ],
                    suggested_action=action,
                    confidence="medium",
                )
            )

        if not math.isnan(gyro_noise):
            severity = "info" if gyro_noise < 0.01 else "warning"
            suffix = "good" if gyro_noise < 0.01 else "noisy"
            findings.append(
                Finding(
                    id=finding_id("imu", "gyro_noise", suffix),
                    title="IMU gyro noise is usable" if severity == "info" else "IMU gyro noise is high",
                    severity=severity,
                    category="sensor_quality",
                    domain=domain,
                    affected_topics=affected_topics,
                    evidence=[
                        Evidence(
                            metric="gyro_noise_mean",
                            observed=round(gyro_noise, 8),
                            expected="<0.01",
                            unit="rad/s",
                            topic=report.imu_topic or None,
                        )
                    ],
                    suggested_action=None if severity == "info" else "Consider lowering IMU integration weight.",
                    confidence="medium",
                )
            )

        if not math.isnan(m.frequency_hz) and m.frequency_hz < 100:
            findings.append(
                Finding(
                    id=finding_id("imu", "rate", "low"),
                    title="IMU rate is low",
                    severity="warning",
                    category="rate_quality",
                    domain=domain,
                    affected_topics=affected_topics,
                    evidence=[
                        Evidence(
                            metric="rate_hz",
                            observed=round(m.frequency_hz, 3),
                            expected=">=100",
                            unit="Hz",
                            topic=report.imu_topic or None,
                        )
                    ],
                    suggested_action="Record a faster IMU stream for tightly-coupled fusion.",
                    confidence="high",
                )
            )
    elif not is_non_slam_domain:
        findings.append(
            Finding(
                id=finding_id("imu", "missing"),
                title="No IMU data recorded",
                severity="info",
                category="topic_presence",
                domain="slam",
                evidence=[Evidence(metric="topic_present", observed=False, expected=True)],
                suggested_action="Use LiDAR-only odometry or record IMU if LIO is required.",
                confidence="high",
            )
        )
    return findings


def _generate_sync_findings(report: EvalReport, is_non_slam_domain: bool) -> list[Finding]:
    if not report.sync or not report.sync.mean_delay_ms:
        return []
    worst_delay = max(report.sync.mean_delay_ms)
    worst_idx = report.sync.mean_delay_ms.index(worst_delay)
    topic_a, topic_b = report.sync.topic_pairs[worst_idx]
    if worst_delay < 20:
        return [
            Finding(
                id=finding_id("sync", "delay", "good"),
                title="Sensor sync delay is acceptable",
                severity="info",
                category="sync_quality",
                domain=None if is_non_slam_domain else "slam",
                affected_topics=[topic_a, topic_b],
                evidence=[
                    Evidence(
                        metric="mean_delay_ms",
                        observed=round(worst_delay, 3),
                        expected="<20",
                        unit="ms",
                    )
                ],
                confidence="medium",
            )
        ]
    if is_non_slam_domain:
        return []
    return [
        Finding(
            id=finding_id("sync", "delay", "high"),
            title="Sensor sync delay is high",
            severity="warning",
            category="sync_quality",
            domain="slam",
            affected_topics=[topic_a, topic_b],
            evidence=[
                Evidence(
                    metric="mean_delay_ms",
                    observed=round(worst_delay, 3),
                    expected="<20",
                    unit="ms",
                )
            ],
            suggested_action="Enable deskew or timestamp compensation for tightly-coupled fusion.",
            confidence="medium",
        )
    ]


def _generate_nav2_findings(report: EvalReport, domain_names: set[str]) -> list[Finding]:
    if "Nav2" not in domain_names:
        return []

    topics = report.topic_info
    findings: list[Finding] = []
    odom_topics = _select_topics(
        topics,
        type_markers=("odometry",),
        suffixes=("/odom",),
        contains=("odometry/filtered", "/odometry/"),
    )
    scan_topics = _select_topics(
        topics,
        type_markers=("laserscan",),
        suffixes=("/scan",),
        contains=("/scan_",),
    )
    cmd_vel_topics = _select_topics(
        topics,
        type_markers=("twist", "twiststamped"),
        suffixes=("/cmd_vel", "/cmd_vel_smoothed"),
        contains=("cmd_vel",),
    )
    plan_topics = _select_topics(
        topics,
        suffixes=("/plan", "/global_plan"),
        contains=("planner_server/plan", "/plan", "trajectory"),
    )
    navigate_topics = _select_topics(
        topics,
        contains=("navigate_to_pose/_action/status", "navigate_to_pose/_action/feedback"),
    )

    for topic_name, expected_hz, label in [
        (odom_topics[0] if odom_topics else "", 20.0, "odometry"),
        (scan_topics[0] if scan_topics else "", 10.0, "scan"),
    ]:
        if not topic_name:
            continue
        rate = float(topics[topic_name].get("rate_hz", 0.0) or 0.0)
        if rate < expected_hz:
            findings.append(
                Finding(
                    id=finding_id("nav2", label, "rate_low"),
                    title=f"Nav2 {label} rate is low",
                    severity="warning",
                    category="rate_quality",
                    domain="nav2",
                    affected_topics=[topic_name],
                    evidence=[
                        Evidence(
                            metric="rate_hz",
                            observed=round(rate, 3),
                            expected=f">={expected_hz:g}",
                            unit="Hz",
                            topic=topic_name,
                        )
                    ],
                    suggested_action=f"Record {label} near {expected_hz:g}Hz or higher for Nav2 readiness.",
                    confidence="high",
                )
            )

    if not cmd_vel_topics:
        findings.append(
            Finding(
                id=finding_id("nav2", "missing_cmd_vel"),
                title="cmd_vel topic is missing",
                severity="warning",
                category="topic_presence",
                domain="nav2",
                affected_topics=["/cmd_vel"],
                evidence=[Evidence(metric="topic_present", observed=False, expected=True, topic="/cmd_vel")],
                suggested_action="Record /cmd_vel when evaluating control-loop readiness.",
                confidence="high",
            )
        )
    elif 0 < float(topics[cmd_vel_topics[0]].get("rate_hz", 0.0) or 0.0) < 10:
        topic_name = cmd_vel_topics[0]
        rate = float(topics[topic_name].get("rate_hz", 0.0) or 0.0)
        findings.append(
            Finding(
                id=finding_id("nav2", "cmd_vel", "rate_low"),
                title="cmd_vel rate is low",
                severity="warning",
                category="rate_quality",
                domain="nav2",
                affected_topics=[topic_name],
                evidence=[
                    Evidence(
                        metric="rate_hz",
                        observed=round(rate, 3),
                        expected=">=10",
                        unit="Hz",
                        topic=topic_name,
                    )
                ],
                suggested_action="Record a faster command stream for control-loop readiness.",
                confidence="high",
            )
        )

    if not plan_topics:
        findings.append(
            Finding(
                id=finding_id("nav2", "missing_global_plan"),
                title="Global plan topic is missing",
                severity="warning",
                category="topic_presence",
                domain="nav2",
                affected_topics=["/plan"],
                evidence=[Evidence(metric="topic_present", observed=False, expected=True, topic="/plan")],
                suggested_action="Record planner output to inspect Nav2 planning behavior.",
                confidence="high",
            )
        )

    if not navigate_topics:
        findings.append(
            Finding(
                id=finding_id("nav2", "missing_navigate_to_pose"),
                title="NavigateToPose action topic is missing",
                severity="warning",
                category="workflow_observability",
                domain="nav2",
                affected_topics=["/navigate_to_pose/_action/status"],
                evidence=[
                    Evidence(
                        metric="action_topic_present",
                        observed=False,
                        expected=True,
                        topic="/navigate_to_pose/_action/status",
                    )
                ],
                suggested_action="Record NavigateToPose action topics for goal-level debugging.",
                confidence="high",
            )
        )

    return findings


def _generate_custom_domain_findings(custom_domains: list["CustomDomainResult"]) -> list[Finding]:
    findings: list[Finding] = []
    for domain in custom_domains:
        domain_id = _domain_id(domain.name)
        severity = "info" if domain.score >= 90 else "warning" if domain.score >= 50 else "error"
        findings.append(
            Finding(
                id=finding_id("custom", domain_id, "evaluated"),
                title=f"{domain.name} custom rules evaluated",
                severity=severity,
                category="custom_rules",
                domain=domain_id,
                affected_topics=list(domain.matched_topics),
                evidence=[
                    Evidence(metric="score", observed=round(domain.score, 3), expected=">=90"),
                    Evidence(metric="matched_topic_count", observed=len(domain.matched_topics)),
                ],
                suggested_action=None if severity == "info" else "Review the custom rule findings and missing/slow topics.",
                confidence="high",
            )
        )
    return findings


def _generate_workflow_findings(metrics: list["WorkflowMetrics"]) -> list[Finding]:
    findings: list[Finding] = []
    for metric in metrics:
        topic_token = clean_topic_token(metric.topic)
        if metric.kind == "action_status" and metric.failure_events > 0:
            findings.append(
                Finding(
                    id=finding_id("workflow", topic_token, "action_failures"),
                    title="Action terminal failures were recorded",
                    severity="warning",
                    category="workflow_outcome",
                    affected_topics=[metric.topic],
                    evidence=[
                        Evidence(metric="success_events", observed=metric.success_events, topic=metric.topic),
                        Evidence(metric="failure_events", observed=metric.failure_events, topic=metric.topic),
                        Evidence(metric="success_rate", observed=metric.success_rate, topic=metric.topic),
                    ],
                    suggested_action="Inspect action failure reasons and upstream planner/controller state.",
                    confidence="medium",
                )
            )
        if metric.kind == "action_result" and metric.failure_events > 0:
            findings.append(
                Finding(
                    id=finding_id("workflow", topic_token, "result_failures"),
                    title="Action result failures were recorded",
                    severity="warning",
                    category="workflow_outcome",
                    affected_topics=[metric.topic],
                    evidence=[
                        Evidence(metric="success_events", observed=metric.success_events, topic=metric.topic),
                        Evidence(metric="failure_events", observed=metric.failure_events, topic=metric.topic),
                        Evidence(metric="failure_reasons", observed=metric.failure_reasons, topic=metric.topic),
                    ],
                    suggested_action="Use the recorded result failures as debugging anchors.",
                    confidence="medium",
                )
            )
        if metric.kind == "service_event" and metric.response_rate is not None and metric.response_rate < 1.0:
            findings.append(
                Finding(
                    id=finding_id("workflow", topic_token, "missing_service_responses"),
                    title="Some service requests have no recorded response",
                    severity="warning",
                    category="workflow_outcome",
                    affected_topics=[metric.topic],
                    evidence=[
                        Evidence(metric="request_events", observed=metric.request_events, topic=metric.topic),
                        Evidence(metric="response_events", observed=metric.response_events, topic=metric.topic),
                        Evidence(metric="response_rate", observed=metric.response_rate, topic=metric.topic),
                    ],
                    suggested_action="Check whether service responses were dropped or not recorded.",
                    confidence="medium",
                )
            )
    return findings


def _domain_representative_topics(domain_name: str, topics: dict[str, dict]) -> list[str]:
    domain = _domain_id(domain_name)
    if domain == "nav2":
        return _select_topics(
            topics,
            type_markers=("odometry", "laserscan"),
            suffixes=("/odom", "/scan", "/cmd_vel", "/cmd_vel_smoothed"),
            contains=("navigate_to_pose", "local_costmap", "global_costmap"),
        )[:8]
    if domain == "autoware":
        prefixes = ("/sensing/", "/perception/", "/planning/", "/control/", "/localization/", "/vehicle/")
        return [name for name in topics if any(name.startswith(prefix) for prefix in prefixes)][:8]
    if domain == "moveit":
        return _select_topics(
            topics,
            type_markers=("jointstate", "displaytrajectory", "planningscene"),
            suffixes=("/joint_states", "/display_planned_path"),
            contains=("move_action", "execute_trajectory", "follow_joint_trajectory"),
        )[:8]
    if domain in {"perception", "robotarm"}:
        return _select_topics(
            topics,
            type_markers=("image", "compressedimage", "camerainfo", "jointstate"),
            contains=("camera_info", "joint_states"),
        )[:8]
    return list(topics)[:8]


def _select_topics(
    topics: dict[str, dict],
    *,
    type_markers: tuple[str, ...] = (),
    prefixes: tuple[str, ...] = (),
    suffixes: tuple[str, ...] = (),
    contains: tuple[str, ...] = (),
) -> list[str]:
    matches = []
    normalized_type_markers = tuple(marker.lower() for marker in type_markers)
    normalized_prefixes = tuple(prefix.lower() for prefix in prefixes)
    normalized_suffixes = tuple(suffix.lower() for suffix in suffixes)
    normalized_contains = tuple(fragment.lower() for fragment in contains)

    for name, info in topics.items():
        if int(info.get("count", 0) or 0) <= 0:
            continue
        lower_name = name.lower()
        lower_type = str(info.get("type", "")).lower()

        if normalized_type_markers and not any(marker in lower_type for marker in normalized_type_markers):
            continue

        has_name_filters = bool(normalized_prefixes or normalized_suffixes or normalized_contains)
        if has_name_filters:
            name_match = (
                any(lower_name.startswith(prefix) for prefix in normalized_prefixes)
                or any(lower_name == suffix or lower_name.endswith(suffix) for suffix in normalized_suffixes)
                or any(fragment in lower_name for fragment in normalized_contains)
            )
            if not name_match:
                continue

        matches.append(name)

    return sorted(matches, key=lambda name: (-topics[name]["rate_hz"], name))


def _domain_id(domain_name: str) -> str:
    return finding_id("", domain_name)


def _mean_valid(values: Iterable[float]) -> float:
    valid = [value for value in values if not math.isnan(value)]
    return float(np.mean(valid)) if valid else float("nan")
