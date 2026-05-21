"""Shared report contract metadata."""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

from bagx import __version__

REPORT_SCHEMA_VERSION = "1.1.0"

_SCHEMA_DIR = Path(__file__).resolve().parent / "schema"


def report_metadata(report_type: str) -> dict[str, str]:
    """Return common metadata for machine-readable JSON reports."""
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "bagx_version": __version__,
        "report_type": report_type,
    }


def findings_schema_path() -> Path:
    """Path to the JSON Schema describing a single Finding object."""
    return _SCHEMA_DIR / "findings.schema.json"


@lru_cache(maxsize=1)
def findings_schema() -> dict[str, Any]:
    """Return the parsed Finding JSON Schema (cached)."""
    with findings_schema_path().open() as f:
        return json.load(f)
