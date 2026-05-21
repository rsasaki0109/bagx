"""Tests for custom rule discovery and plugin loading."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from bagx.custom_rules import (
    discover_rule_plugins,
    evaluate_custom_rule_set,
    load_custom_rule_set,
)
from bagx.custom_rules import CustomDomainResult
from bagx.eval_findings import _generate_custom_check_findings, _generate_custom_domain_findings


class TestRuleDiscovery:
    def test_builtin_plugin_is_discoverable(self):
        plugins = discover_rule_plugins()

        assert any(plugin.name == "warehouse_bot" for plugin in plugins)

    def test_environment_plugin_directory_is_supported(self, monkeypatch, tmp_path: Path):
        plugin_dir = tmp_path / "plugins"
        plugin_dir.mkdir()
        plugin_path = plugin_dir / "field_robot.json"
        plugin_path.write_text(json.dumps({
            "plugin_name": "field_robot",
            "description": "Field robot checks",
            "domains": [
                {
                    "name": "FieldRobot",
                    "match_topics": [{"name_contains": "wheel_odom"}],
                    "checks": [],
                }
            ],
        }, indent=2))
        monkeypatch.setenv("BAGX_RULE_PLUGIN_PATH", str(plugin_dir))

        plugins = discover_rule_plugins()
        rule_set = load_custom_rule_set("field_robot")

        assert any(plugin.name == "field_robot" for plugin in plugins)
        assert rule_set.domains[0].name == "FieldRobot"

    def test_invalid_rule_document_reports_schema_errors(self, tmp_path: Path):
        path = tmp_path / "bad_rules.json"
        path.write_text(json.dumps({
            "domains": [
                {
                    "name": "BadBot",
                    "match_topics": [{"name_contains": 42, "unknown": "x"}],
                    "checks": [
                        {"kind": "min_hertz", "label": "Wheel odom", "selector": {}},
                        {"kind": "topic_rate", "label": "Controller command", "selector": {}},
                        {
                            "kind": "latency",
                            "label": "path to command",
                            "input": {"name_contains": "path"},
                            "target_ms": "fast",
                        },
                    ],
                }
            ]
        }, indent=2))

        with pytest.raises(ValueError) as exc_info:
            load_custom_rule_set(str(path))

        message = str(exc_info.value)
        assert "Invalid custom rules" in message
        assert 'domains[0].checks[0].kind: unknown check kind "min_hertz"' in message
        assert "domains[0].checks[1].min_rate_hz: required for topic_rate" in message
        assert "domains[0].checks[2].output: required for latency" in message
        assert "domains[0].checks[2].target_ms: must be a number" in message


class TestPerCheckEvaluation:
    """Per-check finding emission (v0.4.0 Priority 2)."""

    def _rule_set(self, tmp_path: Path, checks: list[dict]) -> Path:
        path = tmp_path / "rules.json"
        path.write_text(json.dumps({
            "domains": [
                {
                    "name": "TestBot",
                    "match_topics": [{"name_contains": "robot"}],
                    "checks": checks,
                }
            ]
        }))
        return path

    def test_topic_exists_pass_produces_pass_finding(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {"kind": "topic_exists", "label": "Wheel odometry", "selector": {"name_contains": "wheel_odom"}},
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/wheel_odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 50.0},
        }
        results = evaluate_custom_rule_set(topic_info, {}, rule_set)
        assert len(results) == 1
        assert len(results[0].checks) == 1
        check = results[0].checks[0]
        assert check.status == "pass"
        assert check.matched_topics == ["/robot/wheel_odom"]

        findings = _generate_custom_check_findings(results[0], "testbot")
        assert any(f.id == "custom.testbot.wheel_odometry.pass" for f in findings)
        passing = next(f for f in findings if f.id.endswith(".pass"))
        assert passing.severity == "info"

    def test_topic_exists_fail_uses_default_warning(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {"kind": "topic_exists", "label": "Wheel odometry", "selector": {"name_contains": "missing"}},
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/wheel_odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 50.0},
        }
        results = evaluate_custom_rule_set(topic_info, {}, rule_set)
        findings = _generate_custom_check_findings(results[0], "testbot")
        fail = next(f for f in findings if f.id.endswith(".fail"))
        assert fail.severity == "warning"
        assert fail.id == "custom.testbot.wheel_odometry.fail"

    def test_severity_override_error(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {"kind": "topic_exists", "label": "Mandatory beacon", "selector": {"name_contains": "missing"}, "severity": "error"},
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/wheel_odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 50.0},
        }
        results = evaluate_custom_rule_set(topic_info, {}, rule_set)
        assert results[0].checks[0].severity == "error"
        findings = _generate_custom_check_findings(results[0], "testbot")
        fail = next(f for f in findings if f.id.endswith(".fail"))
        assert fail.severity == "error"

    def test_topic_rate_fail_includes_rate_evidence(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {"kind": "topic_rate", "label": "Wheel odom rate", "selector": {"name_contains": "wheel_odom"}, "min_rate_hz": 100},
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/wheel_odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 50.0},
        }
        results = evaluate_custom_rule_set(topic_info, {}, rule_set)
        check = results[0].checks[0]
        assert check.status == "fail"
        assert any(e["metric"] == "rate_hz" and e["observed"] == 50.0 for e in check.evidence)

    def test_latency_skipped_when_no_samples(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {
                "kind": "latency",
                "label": "Path follow",
                "input": {"name_contains": "path"},
                "output": {"name_contains": "cmd_vel"},
                "target_ms": 100,
            },
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/path": {"type": "nav_msgs/msg/Path", "count": 10, "rate_hz": 1.0},
            "/robot/cmd_vel": {"type": "geometry_msgs/msg/Twist", "count": 10, "rate_hz": 1.0},
        }
        # No timestamps after input → no latency samples.
        results = evaluate_custom_rule_set(
            topic_info,
            {"/robot/path": [10_000_000_000], "/robot/cmd_vel": [5_000_000_000]},
            rule_set,
        )
        check = results[0].checks[0]
        assert check.status == "skipped"

        findings = _generate_custom_check_findings(results[0], "testbot")
        skipped = next(f for f in findings if f.id.endswith(".skipped"))
        assert skipped.severity == "warning"

    def test_invalid_severity_rejected(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {"kind": "topic_exists", "label": "X", "selector": {"name_contains": "x"}, "severity": "fatal"},
        ])
        with pytest.raises(ValueError) as exc_info:
            load_custom_rule_set(str(path))
        assert "severity" in str(exc_info.value)

    def test_aggregate_finding_still_emitted(self, tmp_path: Path):
        path = self._rule_set(tmp_path, [
            {"kind": "topic_exists", "label": "Wheel odom", "selector": {"name_contains": "wheel_odom"}},
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/wheel_odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 50.0},
        }
        results = evaluate_custom_rule_set(topic_info, {}, rule_set)
        findings = _generate_custom_domain_findings(results)
        ids = [f.id for f in findings]
        assert "custom.testbot.evaluated" in ids
        assert any(fid.startswith("custom.testbot.") and fid.endswith(".pass") for fid in ids)

    def test_check_results_serialize_in_eval_report(self, tmp_path: Path):
        """custom_domains[].checks survives report.to_dict() for JSON output."""
        path = self._rule_set(tmp_path, [
            {"kind": "topic_exists", "label": "Wheel odom", "selector": {"name_contains": "wheel_odom"}},
        ])
        rule_set = load_custom_rule_set(str(path))
        topic_info = {
            "/robot/wheel_odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 50.0},
        }
        results = evaluate_custom_rule_set(topic_info, {}, rule_set)
        assert isinstance(results[0], CustomDomainResult)
        from dataclasses import asdict
        serialized = asdict(results[0])
        assert serialized["checks"][0]["status"] == "pass"
        assert serialized["checks"][0]["label"] == "Wheel odom"
