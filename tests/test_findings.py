"""Unit tests for the Finding / TimeRange dataclasses."""

from __future__ import annotations

import pytest

from bagx.findings import Evidence, Finding, TimeRange


class TestTimeRange:
    def test_basic_construction(self):
        tr = TimeRange(start_ns=1_000_000_000, end_ns=2_500_000_000)
        assert tr.start_ns == 1_000_000_000
        assert tr.end_ns == 2_500_000_000
        assert tr.duration_ns == 1_500_000_000
        assert tr.duration_sec == pytest.approx(1.5)

    def test_point_event_allowed(self):
        tr = TimeRange(start_ns=42, end_ns=42)
        assert tr.duration_ns == 0
        assert tr.duration_sec == 0.0

    def test_rejects_inverted_endpoints(self):
        with pytest.raises(ValueError, match="precedes"):
            TimeRange(start_ns=500, end_ns=100)

    def test_rejects_negative_start(self):
        with pytest.raises(ValueError, match="non-negative"):
            TimeRange(start_ns=-1, end_ns=10)

    def test_rejects_negative_end(self):
        with pytest.raises(ValueError, match="non-negative"):
            TimeRange(start_ns=0, end_ns=-1)

    def test_overlaps_true_when_intersecting(self):
        a = TimeRange(start_ns=0, end_ns=100)
        b = TimeRange(start_ns=50, end_ns=150)
        assert a.overlaps(b)
        assert b.overlaps(a)

    def test_overlaps_true_when_touching(self):
        a = TimeRange(start_ns=0, end_ns=100)
        b = TimeRange(start_ns=100, end_ns=200)
        assert a.overlaps(b)

    def test_overlaps_false_when_disjoint(self):
        a = TimeRange(start_ns=0, end_ns=100)
        b = TimeRange(start_ns=200, end_ns=300)
        assert not a.overlaps(b)

    def test_to_dict_round_trip(self):
        tr = TimeRange(start_ns=10, end_ns=20)
        assert tr.to_dict() == {"start_ns": 10, "end_ns": 20}
        assert TimeRange.from_dict(tr.to_dict()) == tr

    def test_frozen(self):
        tr = TimeRange(start_ns=10, end_ns=20)
        with pytest.raises((AttributeError, Exception)):
            tr.start_ns = 5  # type: ignore[misc]


class TestFindingWithTimeRange:
    def test_finding_without_time_range_serializes_null(self):
        f = Finding(
            id="nav2.detected",
            title="Nav2 topics detected",
            severity="info",
            category="domain_detection",
        )
        data = f.to_dict()
        assert "time_range" in data
        assert data["time_range"] is None

    def test_finding_with_time_range_serializes(self):
        f = Finding(
            id="anomaly.gnss.fix_lost.gps",
            title="GNSS fix lost",
            severity="warning",
            category="sensor_quality",
            time_range=TimeRange(start_ns=1_000_000_000, end_ns=2_500_000_000),
        )
        data = f.to_dict()
        assert data["time_range"] == {"start_ns": 1_000_000_000, "end_ns": 2_500_000_000}

    def test_finding_keeps_other_fields_intact(self):
        f = Finding(
            id="test.x",
            title="test",
            severity="error",
            category="rate_quality",
            domain="slam",
            affected_topics=["/imu/data"],
            evidence=[Evidence(metric="rate_hz", observed=200.0, unit="Hz")],
            suggested_action="check IMU driver",
            confidence="high",
            time_range=TimeRange(start_ns=0, end_ns=100),
        )
        data = f.to_dict()
        assert data["id"] == "test.x"
        assert data["severity"] == "error"
        assert data["domain"] == "slam"
        assert data["affected_topics"] == ["/imu/data"]
        assert data["evidence"] == [{
            "metric": "rate_hz",
            "observed": 200.0,
            "expected": None,
            "unit": "Hz",
            "topic": None,
        }]
        assert data["suggested_action"] == "check IMU driver"
        assert data["confidence"] == "high"
        assert data["time_range"] == {"start_ns": 0, "end_ns": 100}
