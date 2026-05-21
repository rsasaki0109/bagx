"""Validate findings.schema.json and real eval output against it."""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest

from bagx.contracts import findings_schema, findings_schema_path
from bagx.eval import evaluate_bag


def test_findings_schema_file_is_shipped():
    path = findings_schema_path()
    assert path.exists(), f"findings.schema.json missing at {path}"
    assert path.is_file()


def test_findings_schema_is_valid_jsonschema():
    schema = findings_schema()
    jsonschema.Draft202012Validator.check_schema(schema)


def test_findings_schema_top_level_metadata():
    schema = findings_schema()
    assert schema["$schema"].startswith("https://json-schema.org/draft/2020-12")
    assert schema["title"] == "bagx Finding"
    assert "severity" in schema["properties"]
    assert set(schema["properties"]["severity"]["enum"]) == {
        "info",
        "warning",
        "error",
        "critical",
    }


@pytest.mark.parametrize(
    "fixture_name",
    ["multi_bag", "nav2_bag", "moveit_bag", "perception_bag"],
)
def test_real_findings_validate(fixture_name: str, request: pytest.FixtureRequest):
    bag: Path = request.getfixturevalue(fixture_name)
    report = evaluate_bag(str(bag))
    findings = report.to_dict()["findings"]
    schema = findings_schema()
    validator = jsonschema.Draft202012Validator(schema)
    for finding in findings:
        errors = list(validator.iter_errors(finding))
        assert not errors, (
            f"finding {finding.get('id')!r} from {fixture_name} failed schema: "
            f"{[e.message for e in errors]}"
        )


def test_schema_rejects_invalid_severity():
    schema = findings_schema()
    invalid = {
        "id": "test.bad",
        "title": "test",
        "severity": "fatal",
        "category": "test",
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(invalid))
    assert errors, "schema should reject severity=fatal"


def test_schema_rejects_bad_id_pattern():
    schema = findings_schema()
    invalid = {
        "id": "Bad ID With Spaces",
        "title": "test",
        "severity": "info",
        "category": "test",
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(invalid))
    assert errors, "schema should reject id with spaces and uppercase"


def test_schema_file_parses_as_json():
    with findings_schema_path().open() as f:
        parsed = json.load(f)
    assert "$id" in parsed


def test_schema_accepts_time_range_object():
    schema = findings_schema()
    finding = {
        "id": "anomaly.gnss.fix_lost.gps",
        "title": "GNSS fix lost",
        "severity": "warning",
        "category": "sensor_quality",
        "time_range": {"start_ns": 1_000_000_000, "end_ns": 2_500_000_000},
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(finding))
    assert not errors, [e.message for e in errors]


def test_schema_accepts_time_range_null():
    schema = findings_schema()
    finding = {
        "id": "nav2.detected",
        "title": "Nav2 topics detected",
        "severity": "info",
        "category": "domain_detection",
        "time_range": None,
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(finding))
    assert not errors, [e.message for e in errors]


def test_schema_accepts_finding_without_time_range_key():
    """Pre-1.3.0 reports omit time_range entirely — must still validate."""
    schema = findings_schema()
    finding = {
        "id": "nav2.detected",
        "title": "Nav2 topics detected",
        "severity": "info",
        "category": "domain_detection",
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(finding))
    assert not errors, [e.message for e in errors]


def test_schema_rejects_time_range_with_negative_ns():
    schema = findings_schema()
    finding = {
        "id": "test.x",
        "title": "test",
        "severity": "info",
        "category": "test",
        "time_range": {"start_ns": -1, "end_ns": 100},
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(finding))
    assert errors, "schema should reject negative time_range endpoints"


def test_schema_rejects_time_range_with_extra_fields():
    schema = findings_schema()
    finding = {
        "id": "test.x",
        "title": "test",
        "severity": "info",
        "category": "test",
        "time_range": {"start_ns": 0, "end_ns": 100, "label": "stuff"},
    }
    validator = jsonschema.Draft202012Validator(schema)
    errors = list(validator.iter_errors(finding))
    assert errors, "schema should reject extra fields inside time_range"
