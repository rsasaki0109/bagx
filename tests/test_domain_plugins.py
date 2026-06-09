"""Tests for the domain plugin SDK."""

from __future__ import annotations

from pathlib import Path

import pytest

from bagx.domain_plugins import (
    SimpleDomainPlugin,
    detect_domains,
    discover_domain_plugins,
)
from bagx.eval import evaluate_bag


def test_discover_domain_plugins_includes_builtins():
    plugins = discover_domain_plugins()
    names = {plugin.name for plugin in plugins}
    assert {"Nav2", "Autoware", "MoveIt", "Perception", "RobotArm", "Control"} <= names


def test_detect_domains_nav2_bag(nav2_bag: Path):
    report = evaluate_bag(str(nav2_bag))
    detected = detect_domains(report.topic_info)
    assert [plugin.name for plugin in detected] == ["Nav2"]


def test_detect_domains_moveit_bag(moveit_bag: Path):
    report = evaluate_bag(str(moveit_bag))
    detected = detect_domains(report.topic_info)
    assert "MoveIt" in {plugin.name for plugin in detected}


def test_simple_domain_plugin_hooks():
    topics = {
        "/rtk/fix": {"type": "sensor_msgs/msg/NavSatFix", "count": 10, "rate_hz": 5.0},
    }

    def detect(topic_info: dict) -> bool:
        return any("rtk" in name.lower() for name in topic_info)

    plugin = SimpleDomainPlugin(name="RTK", detect_fn=detect)
    assert plugin.detect(topics)
    assert plugin.representative_topics(topics) == []
    assert plugin.generate_findings(None) == []  # type: ignore[arg-type]


def test_detect_domains_respects_plugin_order():
    topics = {
        "/odom": {"type": "nav_msgs/msg/Odometry", "count": 100, "rate_hz": 20.0},
        "/scan": {"type": "sensor_msgs/msg/LaserScan", "count": 50, "rate_hz": 10.0},
        "/plan": {"type": "nav_msgs/msg/Path", "count": 10, "rate_hz": 1.0},
    }
    detected = detect_domains(topics)
    assert detected[0].name == "Nav2"


def test_entry_point_plugin_loading(monkeypatch: pytest.MonkeyPatch):
    class FakePlugin:
        name = "FakeStack"

        def detect(self, topic_info: dict) -> bool:
            return "/fake" in topic_info

        def representative_topics(self, topic_info: dict) -> list[str]:
            return ["/fake"]

        def generate_findings(self, report) -> list:
            return []

    class FakeEntryPoint:
        name = "fake_stack"
        value = "fake.module:FakePlugin"

        def load(self):
            return FakePlugin

    monkeypatch.setattr(
        "bagx.domain_plugins._entry_point_plugins",
        lambda: [FakePlugin()],
    )
    topics = {"/fake": {"type": "std_msgs/msg/String", "count": 1, "rate_hz": 1.0}}
    detected = detect_domains(topics)
    assert detected[-1].name == "FakeStack"
