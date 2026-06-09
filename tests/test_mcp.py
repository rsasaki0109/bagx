"""Tests for bagx MCP tools and bag query helpers."""

from __future__ import annotations

from pathlib import Path

import pytest

from bagx.ask import list_bag_topics, query_messages
from bagx.mcp_server import (
    HAS_MCP,
    compare_bags_report,
    detect_anomalies_report,
    eval_bag_report,
    list_topics_report,
    query_messages_report,
)
from tests.conftest import _create_db3, build_navsatfix_cdr

pytestmark = pytest.mark.skipif(not HAS_MCP, reason="mcp not installed")

BASE_NS = 1_700_000_000_000_000_000


@pytest.fixture
def gnss_bag(tmp_path: Path) -> Path:
    topics = [{"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"}]
    messages = []
    for i in range(20):
        ts = BASE_NS + i * 100_000_000
        messages.append(
            {
                "topic": "/gnss",
                "timestamp_ns": ts,
                "data": build_navsatfix_cdr(
                    stamp_sec=ts // 1_000_000_000,
                    stamp_nanosec=ts % 1_000_000_000,
                    status=0,
                    latitude=35.0 + i * 0.001,
                    longitude=139.0,
                    altitude=10.0,
                ),
            }
        )
    return _create_db3(tmp_path / "gnss.db3", topics, messages)


def test_list_bag_topics(gnss_bag: Path) -> None:
    report = list_bag_topics(str(gnss_bag))
    assert report["message_count"] == 20
    assert report["topics"][0]["name"] == "/gnss"


def test_query_messages_with_time_window(gnss_bag: Path) -> None:
    start = BASE_NS + 500_000_000
    end = BASE_NS + 1_500_000_000
    rows = query_messages(str(gnss_bag), "/gnss", start_ns=start, end_ns=end, limit=10)
    assert 1 <= len(rows) <= 10
    assert all(start <= row["timestamp_ns"] <= end for row in rows)


def test_query_messages_unknown_topic(gnss_bag: Path) -> None:
    with pytest.raises(ValueError, match="Topic not found"):
        query_messages(str(gnss_bag), "/missing")


def test_eval_bag_report(gnss_bag: Path) -> None:
    report = eval_bag_report(str(gnss_bag))
    assert report["report_type"] == "eval"
    assert "findings" in report
    assert report["overall_score"] >= 0


def test_list_topics_report(gnss_bag: Path) -> None:
    report = list_topics_report(str(gnss_bag))
    assert report["topics"][0]["type"] == "sensor_msgs/msg/NavSatFix"


def test_detect_anomalies_report(gnss_bag: Path) -> None:
    report = detect_anomalies_report(str(gnss_bag))
    assert report["report_type"] == "anomaly"
    assert "anomalies" in report


def test_compare_bags_report(gnss_bag: Path, tmp_path: Path) -> None:
    other = gnss_bag.parent / "gnss_copy.db3"
    other.write_bytes(gnss_bag.read_bytes())
    report = compare_bags_report(str(gnss_bag), str(other))
    assert report["report_type"] == "compare"
    assert report["winner"] in {"A", "B", "tie"}


def test_query_messages_report(gnss_bag: Path) -> None:
    report = query_messages_report(str(gnss_bag), "/gnss", limit=3)
    assert report["count"] == 3
    assert len(report["messages"]) == 3


def test_create_mcp_server_registers_tools() -> None:
    from bagx.mcp_server import create_mcp_server

    server = create_mcp_server()
    tool_names = {tool.name for tool in server._tool_manager.list_tools()}  # noqa: SLF001
    assert tool_names == {
        "eval_bag",
        "list_topics",
        "detect_anomalies",
        "compare_bags",
        "query_messages",
    }
