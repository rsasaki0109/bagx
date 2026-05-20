"""Tests for custom rule discovery and plugin loading."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from bagx.custom_rules import discover_rule_plugins, load_custom_rule_set


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
