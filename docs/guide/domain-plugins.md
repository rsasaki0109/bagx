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

The entry point can expose:

- a zero-argument class implementing the `DomainPlugin` protocol,
- an instance implementing that protocol (including callable instances), or
- a zero-argument factory that returns an instance.

For example, the reference class can be registered directly:

```toml
[project.entry-points."bagx.domains"]
rtk_gnss = "your_package.rtk_gnss:RtkGnssPlugin"
```

Install your package in the environment that runs bagx, then use `bagx domains
list` to verify discovery. Plugin names must be nonempty and have distinct
normalised domain ids. Names colliding with built-ins or an earlier plugin are
reported as unavailable rather than overriding detection.

See `examples/domains/rtk_gnss.py` for a fuller reference implementation.

## Readiness hooks and failures

`representative_topics` returns a `list[str]`; `generate_findings` returns a
`list[Finding]` with valid stable ids, severities and JSON-serializable evidence.
`SimpleDomainPlugin` adapts iterable results to lists; a single topic string is
not a valid representative-topic result. A callable plugin instance is used as
it stands; bagx calls only classes and factory functions to construct plugins.

Plugin failures do not prevent the remaining checks from running. Eval emits
an **error** finding in `domain_detection`:

| Finding id | Failure |
|------------|---------|
| `plugin.<name_token>.load_failed` | Import, construction, invalid protocol/name or name collision |
| `plugin.<name_token>.detect_failed` | Stack detection hook failed |
| `plugin.<name_token>.representative_topics_failed` | Topic summary hook failed or returned the wrong shape |
| `plugin.<name_token>.generate_findings_failed` | Readiness hook failed, returned the wrong shape, invalid id/severity, or unserializable evidence |

Error evidence identifies the hook and exception type. Failed imports / factories
appear as `entry-point (unavailable)` in `bagx domains list`; they do not emit a
false `*.detected` finding. Normal eval output includes `ERROR Plugin:` lines.
Scores still reflect checks that ran, so **use a severity gate** to reject
incomplete evaluation:

```bash
bagx benchmark suite.json --exit-on error
bagx eval recording.db3 --findings-only --severity-min error --json errors.json
```

## Built-in readiness checks

The built-in hooks now expose structured readiness beyond stack detection:

| Domain | Rate targets | Additional recording checks |
|--------|--------------|-----------------------------|
| Autoware | Cameras 15 Hz, LiDAR 10 Hz, vehicle status 20 Hz, planning 5 Hz, control 10 Hz | An observed `/planning/` or `/control/` stage requires its counterpart; sensing-only bags require neither |
| MoveIt | Joint states 100 Hz | Planned path; execution action status or feedback when a path is present |
| Perception | Each image stream 15 Hz | Camera calibration topics |
| RobotArm | Primary joint states 100 Hz, color/depth images 15 Hz | Camera calibration topics |
| Control | Primary odometry 20 Hz, joint states 100 Hz, command 10 Hz | Planning output when commands are present |

Rate finding ids are
`<domain>.<signal>.<topic_token>.{rate_good,rate_low,rate_unavailable}`. Signals
are `camera`, `lidar`, `vehicle_status`, `planning`, `control`, `joint_states`,
`image`, `odometry` or `command`, as appropriate for the domain. Good rates are
informational; low, nonfinite, negative or implausibly high (>100 kHz) rates are
warnings. These are practical targets, not proof that a stack will run correctly.
Secondary image / LiDAR streams are checked independently.

Missing-topic ids are `autoware.missing_{planning,control}`,
`moveit.missing_{planned_path,execution}`, `robotarm.missing_camera_info`,
`perception.missing_camera_info` and `control.missing_planning`. Missing execution
uses `workflow_observability`; other missing topics use `topic_presence`.
Nav2 retains its existing ids and behavior. These findings do not change scores
or report keys, but new warnings can change severity-gate outcomes.

### Public validation

The official NVIDIA NGC `r2bdataset2024` version 1 files were evaluated against
the prior code, with full publisher-listed file sizes and message counts checked.
Both recordings retain all legacy JSON after excluding the new domain findings:

| Recording | Messages | Domain / overall scores | New readiness observations |
|-----------|----------|-------------------------|----------------------------|
| `r2b_robotarm` | 7,916 | 99.733 / 95.987 | Joint states 476.1 Hz and color images 27.5 Hz are good; depth images 14.8 Hz produce `robotarm.image.camera_1.aligned_depth_to_color.image_raw.rate_low` |
| `r2b_whitetunnel` | 17,632 | 100 / 91.712 | All eight compressed-image streams reach 30.3–31.2 Hz and emit informational `perception.image.<topic_token>.rate_good` findings |

These are unreleased-code results. `benchmarks/non_slam_suite.json` records the
new expected ids for these public cases, alongside existing score thresholds.
Source URLs are listed in the [public bag fetch script](https://github.com/rsasaki0109/bagx/blob/main/scripts/fetch_scoreboard_bags.sh).
Run the selected contracts after obtaining those recordings:

```bash
export BAGX_REALBAGS=/path/to/public-bags
bagx benchmark benchmarks/non_slam_suite.json --case nvidia-r2b-robotarm --case nvidia-r2b-whitetunnel
```

For acquisition-timestamp interpretation, see the [Whitetunnel sync measurements](sync.md#public-multi-sensor-validation).

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
