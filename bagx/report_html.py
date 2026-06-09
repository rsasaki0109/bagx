"""Convert bagx eval JSON reports into self-contained HTML."""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any

from bagx import __version__

SEVERITY_COLORS = {
    "info": "#2563eb",
    "warning": "#d97706",
    "error": "#dc2626",
    "critical": "#7f1d1d",
}
SEVERITY_BADGE = {
    "info": "✔",
    "warning": "⚠",
    "error": "❌",
    "critical": "❌",
}
REPO_URL = "https://github.com/rsasaki0109/bagx"


def load_eval_report(path: str | Path) -> dict[str, Any]:
    """Load an eval JSON report from disk."""
    with Path(path).open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if payload.get("report_type") not in {None, "eval"}:
        raise ValueError(f"Expected eval report JSON, got report_type={payload.get('report_type')!r}")
    return payload


def eval_json_to_html(report: dict[str, Any]) -> str:
    """Render a self-contained HTML document from an eval JSON dict."""
    bag_path = html.escape(str(report.get("bag_path", "unknown")))
    duration = float(report.get("duration_sec") or 0.0)
    overall = report.get("overall_score")
    overall_text = "—" if overall is None else f"{float(overall):.1f}"
    domains = _detected_domains(report)
    domain_text = ", ".join(html.escape(name) for name in domains) or "SLAM / general"
    bagx_version = html.escape(str(report.get("bagx_version", __version__)))
    schema_version = html.escape(str(report.get("schema_version", "unknown")))

    sections = [
        _render_header(bag_path, overall_text, domain_text, duration, report),
        _render_timeline_section(report),
        _render_rate_table(report),
        _render_findings_section(report),
        _render_footer(bagx_version, schema_version),
    ]
    body = "\n".join(sections)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>bagx eval report — {bag_path}</title>
  <style>
{_base_css()}
  </style>
</head>
<body>
{body}
</body>
</html>
"""


def write_eval_html(report: dict[str, Any], output_path: str | Path) -> Path:
    """Write HTML rendered from ``report`` to ``output_path``."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(eval_json_to_html(report), encoding="utf-8")
    return path


def _base_css() -> str:
    return """
    :root {
      --bg: #f8fafc;
      --card: #ffffff;
      --text: #0f172a;
      --muted: #64748b;
      --border: #e2e8f0;
      --accent: #4f46e5;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, sans-serif;
      background: var(--bg);
      color: var(--text);
      line-height: 1.5;
    }
    .wrap { max-width: 1100px; margin: 0 auto; padding: 24px; }
    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 20px;
      margin-bottom: 20px;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.05);
    }
    h1, h2 { margin: 0 0 12px; }
    h2 { font-size: 1.1rem; }
    .score {
      font-size: 3rem;
      font-weight: 700;
      color: var(--accent);
      line-height: 1;
    }
    .meta { color: var(--muted); font-size: 0.95rem; }
    .grid {
      display: grid;
      grid-template-columns: 180px 1fr;
      gap: 12px 20px;
      margin-top: 16px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.92rem;
    }
    th, td {
      border-bottom: 1px solid var(--border);
      padding: 8px 10px;
      text-align: left;
      vertical-align: top;
    }
    th { color: var(--muted); font-weight: 600; }
    .finding {
      border-left: 4px solid var(--border);
      padding: 10px 12px;
      margin-bottom: 10px;
      background: #fcfdff;
      border-radius: 0 8px 8px 0;
    }
    .finding-id {
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 0.85rem;
      color: var(--muted);
    }
    .badge { font-weight: 700; }
    .timeline-wrap { overflow-x: auto; }
    footer {
      color: var(--muted);
      font-size: 0.9rem;
      text-align: center;
      padding: 12px 0 28px;
    }
    footer a { color: var(--accent); text-decoration: none; }
    """


def _render_header(
    bag_path: str,
    overall_text: str,
    domain_text: str,
    duration: float,
    report: dict[str, Any],
) -> str:
    messages = int(report.get("total_messages") or 0)
    topics = int(report.get("topic_count") or 0)
    return f"""
  <div class="wrap">
    <section class="card">
      <h1>bagx eval report</h1>
      <div class="meta">{bag_path}</div>
      <div class="grid">
        <div class="meta">Overall score</div>
        <div class="score">{overall_text}<span style="font-size:1.2rem;color:var(--muted)">/100</span></div>
        <div class="meta">Detected stack</div>
        <div>{domain_text}</div>
        <div class="meta">Bag metadata</div>
        <div>{duration:.1f}s · {messages:,} messages · {topics} topics</div>
      </div>
    </section>"""


def _render_timeline_section(report: dict[str, Any]) -> str:
    svg = _render_timeline_svg(report)
    if not svg:
        return """
    <section class="card">
      <h2>Topic timeline</h2>
      <p class="meta">No temporal findings with time ranges in this report.</p>
    </section>"""
    return f"""
    <section class="card">
      <h2>Topic timeline</h2>
      <p class="meta">Colored bands show temporal findings (gap, anomaly, GNSS loss, …).</p>
      <div class="timeline-wrap">{svg}</div>
    </section>"""


def _render_rate_table(report: dict[str, Any]) -> str:
    topic_info: dict[str, dict[str, Any]] = report.get("topic_info") or {}
    rows = []
    for name in sorted(topic_info):
        info = topic_info[name]
        rate = float(info.get("rate_hz") or 0.0)
        topic_type = html.escape(str(info.get("type", "")))
        status, status_class = _rate_status(name, rate, report)
        rows.append(
            "<tr>"
            f"<td><code>{html.escape(name)}</code></td>"
            f"<td>{topic_type}</td>"
            f"<td>{rate:.1f}</td>"
            f'<td class="badge" style="color:{status_class}">{status}</td>'
            "</tr>"
        )
    if not rows:
        rows.append('<tr><td colspan="4" class="meta">No topics recorded.</td></tr>')
    return f"""
    <section class="card">
      <h2>Topic rates</h2>
      <table>
        <thead><tr><th>Topic</th><th>Type</th><th>Hz</th><th>Status</th></tr></thead>
        <tbody>{"".join(rows)}</tbody>
      </table>
    </section>"""


def _render_findings_section(report: dict[str, Any]) -> str:
    findings = report.get("findings") or []
    if not findings:
        return """
    <section class="card">
      <h2>Findings</h2>
      <p class="meta">No structured findings in this report.</p>
    </section>"""
    blocks = []
    for finding in sorted(findings, key=lambda item: (item.get("severity", ""), item.get("id", ""))):
        severity = str(finding.get("severity", "info"))
        color = SEVERITY_COLORS.get(severity, "#64748b")
        title = html.escape(str(finding.get("title", "")))
        finding_id = html.escape(str(finding.get("id", "")))
        span = _format_time_range(finding.get("time_range"))
        span_html = f'<div class="meta">{html.escape(span)}</div>' if span else ""
        blocks.append(
            f'<div class="finding" style="border-left-color:{color}">'
            f'<div class="finding-id">{finding_id}</div>'
            f"<div><strong>{title}</strong></div>"
            f"{span_html}"
            "</div>"
        )
    return f"""
    <section class="card">
      <h2>Findings</h2>
      {"".join(blocks)}
    </section>"""


def _render_footer(bagx_version: str, schema_version: str) -> str:
    return f"""
    <footer>
      generated by <a href="{REPO_URL}">bagx</a> {bagx_version}
      · schema {schema_version}
    </footer>
  </div>"""


def _detected_domains(report: dict[str, Any]) -> list[str]:
    domains = []
    for finding in report.get("findings") or []:
        if finding.get("category") == "domain_detection" and str(finding.get("id", "")).endswith(".detected"):
            title = str(finding.get("title", ""))
            if title.endswith(" topics detected"):
                domains.append(title[: -len(" topics detected")])
    return sorted(domains)


def _rate_status(topic: str, rate_hz: float, report: dict[str, Any]) -> tuple[str, str]:
    worst = None
    for finding in report.get("findings") or []:
        affected = finding.get("affected_topics") or []
        if topic not in affected:
            continue
        category = finding.get("category", "")
        if category not in {"rate_quality", "topic_presence", "sensor_quality"}:
            continue
        severity = str(finding.get("severity", "info"))
        if severity in {"error", "critical"}:
            return SEVERITY_BADGE.get(severity, "❌"), SEVERITY_COLORS.get(severity, "#dc2626")
        if severity == "warning" and worst != "error":
            worst = "warning"
    if worst == "warning":
        return "⚠", SEVERITY_COLORS["warning"]
    if rate_hz <= 0:
        return "❌", SEVERITY_COLORS["error"]
    return "✔", SEVERITY_COLORS["info"]


def _format_time_range(time_range: dict[str, Any] | None) -> str:
    if not isinstance(time_range, dict):
        return ""
    start_ns = time_range.get("start_ns")
    end_ns = time_range.get("end_ns")
    if start_ns is None or end_ns is None:
        return ""
    start_s = int(start_ns) / 1e9
    end_s = int(end_ns) / 1e9
    if start_s == end_s:
        return f"t={start_s:.2f}s"
    return f"t={start_s:.2f}s – {end_s:.2f}s"


def _bag_time_bounds(report: dict[str, Any]) -> tuple[float, float]:
    duration = float(report.get("duration_sec") or 0.0)
    starts: list[int] = []
    ends: list[int] = []
    for finding in report.get("findings") or []:
        tr = finding.get("time_range")
        if isinstance(tr, dict) and "start_ns" in tr and "end_ns" in tr:
            starts.append(int(tr["start_ns"]))
            ends.append(int(tr["end_ns"]))
    if starts:
        start_s = min(starts) / 1e9
        end_s = max(max(ends) / 1e9, start_s + duration)
        return start_s, max(end_s, start_s + 0.001)
    return 0.0, max(duration, 0.001)


def _render_timeline_svg(report: dict[str, Any]) -> str:
    topic_info: dict[str, dict[str, Any]] = report.get("topic_info") or {}
    temporal = [
        finding
        for finding in (report.get("findings") or [])
        if isinstance(finding.get("time_range"), dict)
    ]
    if not temporal:
        return ""

    topics = sorted(topic_info.keys())
    if not topics:
        topics = sorted(
            {
                topic
                for finding in temporal
                for topic in (finding.get("affected_topics") or [])
            }
        )
    if not topics:
        return ""

    start_s, end_s = _bag_time_bounds(report)
    span = end_s - start_s
    left_pad = 180
    right_pad = 20
    row_height = 22
    top_pad = 24
    width = 920
    height = top_pad + row_height * len(topics) + 28
    plot_width = width - left_pad - right_pad

    def x_pos(seconds: float) -> float:
        return left_pad + ((seconds - start_s) / span) * plot_width

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-label="Topic timeline">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>',
        f'<line x1="{left_pad}" y1="{top_pad - 6}" x2="{width - right_pad}" y2="{top_pad - 6}" stroke="#cbd5e1"/>',
    ]

    for index, topic in enumerate(topics):
        y = top_pad + index * row_height
        label = html.escape(topic if len(topic) <= 28 else "…" + topic[-27:])
        lines.append(f'<text x="8" y="{y + 14}" fill="#334155" font-size="11">{label}</text>')
        lines.append(
            f'<rect x="{left_pad}" y="{y + 3}" width="{plot_width}" height="12" '
            f'fill="#f1f5f9" stroke="#e2e8f0" stroke-width="1"/>'
        )

    for finding in temporal:
        tr = finding["time_range"]
        t0 = int(tr["start_ns"]) / 1e9
        t1 = int(tr["end_ns"]) / 1e9
        x0 = x_pos(t0)
        x1 = x_pos(max(t1, t0 + 0.001))
        band_width = max(x1 - x0, 2.0)
        color = SEVERITY_COLORS.get(str(finding.get("severity", "info")), "#64748b")
        affected = finding.get("affected_topics") or []
        target_topics = [topic for topic in topics if topic in affected] or topics[:1]
        for topic in target_topics:
            try:
                index = topics.index(topic)
            except ValueError:
                continue
            y = top_pad + index * row_height + 3
            title = html.escape(str(finding.get("id", "")))
            lines.append(
                f'<rect x="{x0:.2f}" y="{y}" width="{band_width:.2f}" height="12" '
                f'fill="{color}" opacity="0.75"><title>{title}</title></rect>'
            )

    axis_y = top_pad + row_height * len(topics) + 8
    lines.append(f'<text x="{left_pad}" y="{axis_y}" fill="#64748b" font-size="10">{start_s:.1f}s</text>')
    lines.append(
        f'<text x="{width - right_pad - 24}" y="{axis_y}" fill="#64748b" font-size="10">{end_s:.1f}s</text>'
    )
    lines.append("</svg>")
    return "\n".join(lines)
