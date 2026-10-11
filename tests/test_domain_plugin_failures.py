"""Installed plugin failures must remain visible to JSON consumers and CI gates."""

import json

import jsonschema
import pytest
from typer.testing import CliRunner

from bagx.cli import app
from bagx.contracts import findings_schema
from bagx.domain_plugins import (
    SimpleDomainPlugin,
    detect_domains,
    discover_domain_plugins,
    plugin_source_label,
)
from bagx.eval import evaluate_bag
from bagx.findings import Evidence, Finding, TimeRange


class EntryPoint:
    name = "external_stack"

    def __init__(self, obj=None, error=None):
        self.obj, self.error = obj, error

    def load(self):
        if self.error:
            raise self.error
        return self.obj


class ExternalPlugin:
    name = "ExternalStack"

    def detect(self, topics):
        return True

    def representative_topics(self, topics):
        return list(topics)[:1]

    def generate_findings(self, report):
        return [Finding("external.ready", "External readiness completed", "info", "custom_rules")]


def install(monkeypatch, *entries):
    monkeypatch.setattr("bagx.domain_plugins._md.entry_points", lambda **kwargs: entries)


@pytest.mark.parametrize("kind", ["class", "factory", "instance", "callable_instance"])
def test_real_entry_point_loader_accepts_supported_objects(monkeypatch, gnss_bag, kind):
    class CallablePlugin(ExternalPlugin):
        def __call__(self):
            raise AssertionError("An existing callable plugin must not be instantiated again")

    obj = {
        "class": ExternalPlugin,
        "factory": lambda: ExternalPlugin(),
        "instance": ExternalPlugin(),
        "callable_instance": CallablePlugin(),
    }[kind]
    install(monkeypatch, EntryPoint(obj))
    report = evaluate_bag(str(gnss_bag))
    assert "external.ready" in {f.id for f in report.findings}
    assert not any(f.id.startswith("plugin.") for f in report.findings)
    for finding in report.findings:
        jsonschema.validate(finding.to_dict(), findings_schema())


@pytest.mark.parametrize("failure", ["import", "constructor", "invalid", "empty_name", "collision"])
def test_load_failures_are_reported_without_stopping_other_checks(monkeypatch, gnss_bag, failure):
    class BadConstructor(ExternalPlugin):
        def __init__(self):
            raise RuntimeError("must not expose this exception text in JSON")

    class EmptyName(ExternalPlugin):
        name = " "

    class Collision(ExternalPlugin):
        name = "NAV2"

    entry = {
        "import": EntryPoint(error=ImportError("missing dependency")),
        "constructor": EntryPoint(BadConstructor),
        "invalid": EntryPoint(object()),
        "empty_name": EntryPoint(EmptyName),
        "collision": EntryPoint(Collision),
    }[failure]
    install(monkeypatch, entry)
    plugins = discover_domain_plugins()
    assert plugin_source_label(plugins[-1]) == "entry-point (unavailable)"
    report = evaluate_bag(str(gnss_bag))
    finding = next(f for f in report.findings if f.id == "plugin.external_stack.load_failed")
    assert finding.severity == "error"
    assert any("Plugin:" in r for r in report.to_dict()["recommendations"])
    assert "gnss.fix_rate.gappy" in {f.id for f in report.findings}
    assert not any(f.id == "externalstack.detected" for f in report.findings)
    assert "must not expose" not in json.dumps(report.to_dict())
    jsonschema.validate(finding.to_dict(), findings_schema())


@pytest.mark.parametrize("hook", ["detect", "representative_topics", "generate_findings"])
def test_hook_exceptions_produce_error_findings(monkeypatch, gnss_bag, hook):
    class Broken(ExternalPlugin):
        pass

    def fail(*args):
        raise RuntimeError("hook failed")

    setattr(Broken, hook, fail)
    install(monkeypatch, EntryPoint(Broken))
    report = evaluate_bag(str(gnss_bag))
    failure = next(f for f in report.findings if f.id == f"plugin.externalstack.{hook}_failed")
    assert failure.severity == "error"
    assert "gnss.fix_rate.gappy" in {f.id for f in report.findings}
    jsonschema.validate(
        report.to_dict()["findings"][report.findings.index(failure)], findings_schema()
    )


@pytest.mark.parametrize(
    "value",
    [
        None,
        "ready",
        [None],
        [Finding("bad", "bad", "invalid", "custom_rules")],
        [Finding("bad id", "bad", "info", "custom_rules")],
        [Finding("bad", "bad", "info", "custom_rules", affected_topics="/gps")],
        [
            Finding(
                "bad", "bad", "info", "custom_rules", evidence=[Evidence(metric=42, observed=True)]
            )
        ],
        [
            Finding(
                "bad",
                "bad",
                "info",
                "custom_rules",
                evidence=[Evidence(metric="ok", observed=float("inf"))],
            )
        ],
        [Finding("bad", "bad", "info", "custom_rules", time_range=TimeRange(1.5, 2.5))],
    ],
)
def test_invalid_findings_are_not_allowed_to_silently_pass_a_gate(monkeypatch, gnss_bag, value):
    plugin = SimpleDomainPlugin(
        "InvalidResult", lambda topics: True, findings_fn=lambda report: value
    )
    # Use a raw implementation: SimpleDomainPlugin deliberately adapts iterables.
    plugin.generate_findings = lambda report: value
    install(monkeypatch, EntryPoint(plugin))
    report = evaluate_bag(str(gnss_bag))
    failures = [
        f for f in report.findings if f.id == "plugin.invalidresult.generate_findings_failed"
    ]
    assert len(failures) == 1 and failures[0].severity == "error"
    json.dumps(report.to_dict(), allow_nan=False)


def test_duplicate_normalized_plugin_names_are_not_both_detected(monkeypatch):
    a, b = ExternalPlugin(), ExternalPlugin()
    a.name, b.name = "External Stack", "external_stack"
    install(monkeypatch, EntryPoint(a), EntryPoint(b))
    plugins = discover_domain_plugins()
    detected = detect_domains(
        {"/gps": {"type": "sensor_msgs/msg/NavSatFix", "count": 1, "rate_hz": 1}}, plugins
    )
    assert [p.name for p in detected] == ["External Stack"]
    assert plugin_source_label(plugins[-1]) == "entry-point (unavailable)"


def test_broken_plugin_fails_benchmark_severity_gate(monkeypatch, gnss_bag, tmp_path):
    install(monkeypatch, EntryPoint(error=ImportError("missing plugin")))
    manifest = tmp_path / "suite.json"
    manifest.write_text(
        json.dumps({"cases": [{"name": "gnss", "bag_path": str(gnss_bag), "expect": {}}]})
    )
    result = CliRunner().invoke(app, ["benchmark", str(manifest), "--exit-on", "error"])
    assert result.exit_code == 1, result.output
    assert "worst finding severity" in result.output


def test_string_representative_topics_is_reported_as_an_error(monkeypatch, gnss_bag):
    plugin = SimpleDomainPlugin(
        "BadTopics", lambda topics: True, representative_fn=lambda topics: "/gps"
    )
    install(monkeypatch, EntryPoint(plugin))
    result = evaluate_bag(str(gnss_bag))
    assert "plugin.badtopics.representative_topics_failed" in {f.id for f in result.findings}
    detected = next(f for f in result.findings if f.id == "badtopics.detected")
    assert detected.affected_topics == []


def test_packaged_example_class_is_discovered_from_actual_metadata(monkeypatch, gnss_bag, tmp_path):
    dist = tmp_path / "example_domain-0.0.0.dist-info"
    dist.mkdir()
    (dist / "METADATA").write_text("Metadata-Version: 2.1\nName: example-domain\nVersion: 0.0.0\n")
    (dist / "entry_points.txt").write_text(
        "[bagx.domains]\nrtk_gnss = examples.domains.rtk_gnss:RtkGnssPlugin\n"
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    report = evaluate_bag(str(gnss_bag))
    assert "rtk_gnss.detected" in {f.id for f in report.findings}
    assert not any(f.id.startswith("plugin.rtk_gnss.") for f in report.findings)
