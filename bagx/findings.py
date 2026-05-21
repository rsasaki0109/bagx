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


@dataclass(frozen=True)
class TimeRange:
    """Inclusive time interval in absolute ROS time (nanoseconds).

    ``start_ns == end_ns`` represents a point event. Both endpoints are
    expressed in absolute ROS time so reports remain meaningful when
    compared across bag boundaries; relative-second views are a rendering
    concern, not a storage one.
    """

    start_ns: int
    end_ns: int

    def __post_init__(self) -> None:
        if self.start_ns < 0 or self.end_ns < 0:
            raise ValueError(
                f"TimeRange endpoints must be non-negative ns "
                f"(got start={self.start_ns}, end={self.end_ns})"
            )
        if self.end_ns < self.start_ns:
            raise ValueError(
                f"TimeRange end_ns ({self.end_ns}) precedes start_ns ({self.start_ns})"
            )

    @property
    def duration_ns(self) -> int:
        return self.end_ns - self.start_ns

    @property
    def duration_sec(self) -> float:
        return self.duration_ns / 1e9

    def overlaps(self, other: "TimeRange") -> bool:
        return self.start_ns <= other.end_ns and other.start_ns <= self.end_ns

    def to_dict(self) -> dict[str, int]:
        return {"start_ns": self.start_ns, "end_ns": self.end_ns}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TimeRange":
        return cls(start_ns=int(data["start_ns"]), end_ns=int(data["end_ns"]))


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
    time_range: TimeRange | None = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["evidence"] = [item.to_dict() for item in self.evidence]
        data["time_range"] = self.time_range.to_dict() if self.time_range else None
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
