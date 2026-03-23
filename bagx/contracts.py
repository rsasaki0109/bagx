"""Shared report contract metadata."""

from __future__ import annotations

from bagx import __version__

REPORT_SCHEMA_VERSION = "1.0.0"


def report_metadata(report_type: str) -> dict[str, str]:
    """Return common metadata for machine-readable JSON reports."""
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "bagx_version": __version__,
        "report_type": report_type,
    }
