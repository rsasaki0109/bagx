"""MCP server exposing bagx analysis tools over stdio.

No LLM API keys are required — tools return structured bag data for the
host agent (Claude Code, Claude Desktop, etc.) to reason about.
"""

from __future__ import annotations

from typing import Any

try:
    from mcp.server.fastmcp import FastMCP

    HAS_MCP = True
except ImportError:
    HAS_MCP = False
    FastMCP = None  # type: ignore[misc, assignment]


def _require_mcp() -> None:
    if not HAS_MCP:
        raise ImportError(
            "MCP support is not installed. Install with: pip install bagx[mcp]"
        )


def eval_bag_report(path: str) -> dict[str, Any]:
    """Run quality eval and return findings-focused JSON."""
    from bagx.eval import evaluate_bag

    report = evaluate_bag(path)
    data = report.to_dict()
    return {
        "bag_path": data["bag_path"],
        "overall_score": data["overall_score"],
        "domain_score": data.get("domain_score"),
        "duration_sec": data["duration_sec"],
        "findings": data.get("findings", []),
        "recommendations": data.get("recommendations", []),
        "gnss": data.get("gnss"),
        "imu": data.get("imu"),
        "sync": data.get("sync"),
        "schema_version": data.get("schema_version"),
        "report_type": data.get("report_type"),
        "bagx_version": data.get("bagx_version"),
    }


def list_topics_report(path: str) -> dict[str, Any]:
    """List topics and bag metadata."""
    from bagx.ask import list_bag_topics

    return list_bag_topics(path)


def detect_anomalies_report(path: str, topic: str | None = None) -> dict[str, Any]:
    """Run anomaly detection; optional single-topic filter."""
    from bagx.anomaly import detect_anomalies

    topics = [topic] if topic else None
    return detect_anomalies(path, topics=topics).to_dict()


def compare_bags_report(path_a: str, path_b: str) -> dict[str, Any]:
    """Compare two bags by quality metrics."""
    from bagx.compare import compare_bags

    return compare_bags(path_a, path_b).to_dict()


def query_messages_report(
    path: str,
    topic: str,
    start_ns: int | None = None,
    end_ns: int | None = None,
    limit: int = 50,
) -> dict[str, Any]:
    """Fetch decoded messages for one topic."""
    from bagx.ask import query_messages

    messages = query_messages(
        path,
        topic,
        start_ns=start_ns,
        end_ns=end_ns,
        limit=limit,
    )
    return {
        "bag_path": path,
        "topic": topic,
        "start_ns": start_ns,
        "end_ns": end_ns,
        "limit": limit,
        "count": len(messages),
        "messages": messages,
    }


def create_mcp_server() -> FastMCP:
    """Build the FastMCP server with all bagx tools registered."""
    _require_mcp()
    mcp = FastMCP(
        "bagx",
        instructions=(
            "Analyze ROS1/ROS2 rosbag files: eval quality, list topics, "
            "detect anomalies, compare bags, and query decoded messages. "
            "No LLM keys needed — return structured JSON for the host agent."
        ),
    )

    @mcp.tool()
    def eval_bag(path: str) -> dict[str, Any]:
        """Evaluate bag quality and return findings JSON (scores + recommendations)."""
        return eval_bag_report(path)

    @mcp.tool()
    def list_topics(path: str) -> dict[str, Any]:
        """List topics, types, message counts, and bag duration."""
        return list_topics_report(path)

    @mcp.tool()
    def detect_anomalies(path: str, topic: str | None = None) -> dict[str, Any]:
        """Detect GNSS/IMU/rate anomalies; optionally restrict to one topic."""
        return detect_anomalies_report(path, topic=topic)

    @mcp.tool()
    def compare_bags(path_a: str, path_b: str) -> dict[str, Any]:
        """Compare two bags on overall/GNSS/IMU/sync/domain metrics."""
        return compare_bags_report(path_a, path_b)

    @mcp.tool()
    def query_messages(
        path: str,
        topic: str,
        start_ns: int | None = None,
        end_ns: int | None = None,
        limit: int = 50,
    ) -> dict[str, Any]:
        """Return decoded messages for a topic, optionally within [start_ns, end_ns]."""
        return query_messages_report(
            path,
            topic,
            start_ns=start_ns,
            end_ns=end_ns,
            limit=limit,
        )

    return mcp


def run_mcp_server() -> None:
    """Start the bagx MCP server on stdio (blocks until stopped)."""
    create_mcp_server().run(transport="stdio")


if __name__ == "__main__":
    run_mcp_server()
