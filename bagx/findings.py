"""Structured findings for machine-readable readiness reports."""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass, field
from typing import Any, Literal

import numpy as np


Severity = Literal["info", "warning", "error", "critical"]
Confidence = Literal["low", "medium", "high"]

SEVERITY_ORDER: dict[str, int] = {"info": 0, "warning": 1, "error": 2, "critical": 3}


def severity_at_least(severity: str, minimum: str) -> bool:
    """Return True when severity is at or above the minimum threshold."""
    if minimum not in SEVERITY_ORDER:
        raise ValueError(f"Unknown severity: {minimum!r}")
    return SEVERITY_ORDER.get(severity, -1) >= SEVERITY_ORDER[minimum]


@dataclass
class Evidence:
    """One measured or observed fact behind a finding."""

    metric: str
    observed: Any
    expected: Any | None = None
    unit: str | None = None
    topic: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return _clean_nan(asdict(self))


@dataclass
class Finding:
    """A structured readiness result that can be rendered or tested."""

    id: str
    title: str
    severity: Severity
    category: str
    domain: str | None = None
    affected_topics: list[str] = field(default_factory=list)
    evidence: list[Evidence] = field(default_factory=list)
    suggested_action: str | None = None
    confidence: Confidence = "medium"

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["evidence"] = [item.to_dict() for item in self.evidence]
        return _clean_nan(data)


def finding_id(prefix: str, *parts: str) -> str:
    """Build a stable lowercase finding id from free-form parts."""
    tokens = [prefix, *parts]
    raw = ".".join(token for token in tokens if token)
    return re.sub(r"[^a-z0-9_.-]+", "_", raw.lower()).strip("._")


def clean_topic_token(topic: str) -> str:
    """Convert a topic name into an id-safe token."""
    cleaned = topic.strip("/").replace("/", ".")
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", cleaned).strip("._") or "topic"


def _clean_nan(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: _clean_nan(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_clean_nan(v) for v in obj]
    if isinstance(obj, tuple):
        return tuple(_clean_nan(v) for v in obj)
    if isinstance(obj, np.generic):
        return _clean_nan(obj.item())
    if isinstance(obj, float) and math.isnan(obj):
        return None
    return obj
