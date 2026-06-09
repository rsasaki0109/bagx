"""Reference domain plugin for RTK / high-precision GNSS stacks.

Copy this module into your own package and register it via pyproject.toml::

    [project.entry-points."bagx.domains"]
    rtk_gnss = "your_package.rtk_gnss:RtkGnssPlugin"

The plugin detects bags that carry RTK or dual-antenna GNSS topics and emits
findings that complement bagx's built-in GNSS quality checks.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from bagx.domain_plugins import SimpleDomainPlugin
from bagx.findings import Evidence, Finding, finding_id

if TYPE_CHECKING:
    from bagx.eval import EvalReport


class RtkGnssPlugin:
    """Example third-party domain plugin."""

    name = "RTK GNSS"

    def detect(self, topic_info: dict[str, dict]) -> bool:
        for name, info in topic_info.items():
            lower = name.lower()
            type_name = str(info.get("type", "")).lower()
            if "navsatfix" not in type_name:
                continue
            if any(token in lower for token in ("rtk", "gnss", "gps", "fix")):
                return True
        return False

    def representative_topics(self, topic_info: dict[str, dict]) -> list[str]:
        topics = []
        for name, info in topic_info.items():
            if "navsatfix" in str(info.get("type", "")).lower():
                topics.append(name)
        return sorted(topics)[:8]

    def generate_findings(self, report: EvalReport) -> list[Finding]:
        if not report.gnss:
            return [
                Finding(
                    id=finding_id("rtk_gnss", "missing_gnss_metrics"),
                    title="RTK GNSS topics detected but GNSS metrics are unavailable",
                    severity="warning",
                    category="sensor_quality",
                    domain="rtk_gnss",
                    affected_topics=self.representative_topics(report.topic_info),
                    evidence=[Evidence(metric="gnss_metrics_present", observed=False, expected=True)],
                    suggested_action="Ensure NavSatFix messages deserialize correctly in the bag.",
                    confidence="medium",
                )
            ]

        g = report.gnss
        if g.fix_rate >= 0.95:
            return []

        return [
            Finding(
                id=finding_id("rtk_gnss", "fix_rate", "degraded"),
                title="RTK GNSS fix rate is degraded for precision navigation",
                severity="warning",
                category="sensor_quality",
                domain="rtk_gnss",
                affected_topics=self.representative_topics(report.topic_info),
                evidence=[
                    Evidence(metric="fix_rate", observed=round(g.fix_rate, 4), expected=">=0.95"),
                ],
                suggested_action="Inspect RTK base-station link, antenna placement, and outage windows.",
                confidence="high",
            )
        ]


def build_plugin() -> SimpleDomainPlugin | RtkGnssPlugin:
    """Factory helper for entry-point registration."""
    return RtkGnssPlugin()
