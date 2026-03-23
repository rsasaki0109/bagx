"""Tests for custom rule discovery and plugin loading."""

from __future__ import annotations

import json
from pathlib import Path

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
