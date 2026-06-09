# domain-plugins — Extending Stack Detection

bagx ships built-in domain detection for Nav2, Autoware, MoveIt, Perception,
RobotArm, and Control. You can add your own stack without patching bagx by
registering a **domain plugin**.

A domain plugin answers three questions for a bag:

1. **detect** — does this bag belong to my stack?
2. **representative_topics** — which topics best summarise the stack?
3. **generate_findings** — what structured findings should the eval report add?

Plugins are discovered from the `bagx.domains` Python entry-point group. Built-in
domains use the same protocol internally.

## CLI usage

```bash
bagx domains list
bagx eval my_bag.db3 --json report.json
```

When a plugin matches, eval emits a `*.detected` finding plus any findings
returned by `generate_findings`.

## Minimal plugin

```python
from bagx.domain_plugins import SimpleDomainPlugin
from bagx.findings import Evidence, Finding, finding_id

def detect(topic_info: dict) -> bool:
    return any(name.endswith("/rtk/fix") for name in topic_info)

def findings(report) -> list[Finding]:
    return [
        Finding(
            id=finding_id("rtk_gnss", "stack_present"),
            title="RTK GNSS stack detected",
            severity="info",
            category="domain_detection",
            domain="rtk_gnss",
            evidence=[Evidence(metric="domain_detected", observed=True, expected=True)],
            confidence="high",
        )
    ]

plugin = SimpleDomainPlugin(
    name="RTK GNSS",
    detect_fn=detect,
    findings_fn=findings,
)
```

## Packaging

Register the plugin in your package `pyproject.toml`:

```toml
[project.entry-points."bagx.domains"]
rtk_gnss = "your_package.rtk_gnss:build_plugin"
```

The entry point may return:

- a class instance implementing the `DomainPlugin` protocol, or
- a zero-argument factory callable that returns one.

See `examples/domains/rtk_gnss.py` for a fuller reference implementation.

## Finding id policy

Plugin display names are normalised into domain ids via
`bagx.findings.finding_id`. Use lowercase dot-separated ids such as
`rtk_gnss.fix_rate.degraded` so benchmark `expected_findings` contracts stay
stable.

## Relationship to custom rules

- **Domain plugins** — stack detection and readiness findings tied to a
  framework (Nav2-like, RTK GNSS, agricultural robot, …).
- **Custom rules** (`bagx eval --rules`) — topic/rate/latency checks for
  proprietary message stacks without writing Python.

Use both when you need framework-level findings *and* proprietary topic
contracts.
