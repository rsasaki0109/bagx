# custom-rules — Extending bagx for Custom Message Stacks

If your robot uses custom messages, `bagx` can still evaluate the bag without decoding those payloads.

The custom rule engine works on:

- topic names
- topic types
- message rates
- timestamp-based latency between topics

This is enough for many internal stacks where the important question is "did we record the right topics at the right rates, and did the control loop react quickly enough?"

Rules can be loaded from:

- a JSON file path
- a built-in plugin name such as `warehouse_bot`
- a JSON file discovered via `BAGX_RULE_PLUGIN_PATH`
- an installed Python package that exposes a `bagx.rules` entry point

## CLI usage

```bash
bagx rules list
bagx eval warehouse_bag.db3 --rules warehouse_bot
bagx eval warehouse_bag.db3 --rules examples/custom_rules/warehouse_bot.json
bagx benchmark benchmark.json --rules examples/custom_rules/warehouse_bot.json
```

`benchmark` also supports `rules_path` inside the manifest, either at the top level or per case.

Use `bagx rules list` to see built-in and installed plugins.

Invalid rule files are validated before evaluation. bagx reports all common schema
errors it can find, including unknown check kinds, missing selectors, missing
thresholds, and non-numeric latency/rate values.

## Rule format

```json
{
  "domains": [
    {
      "name": "WarehouseBot",
      "min_matches": 2,
      "match_topics": [
        {"name_contains": "wheel_odom", "type_contains": "WheelOdometry"},
        {"name_contains": "controller_cmd", "type_contains": "ControllerCommand"},
        {"name_contains": "mission_path", "type_contains": "MissionPath"}
      ],
      "checks": [
        {
          "kind": "topic_rate",
          "label": "Wheel odometry",
          "selector": {"name_contains": "wheel_odom"},
          "min_rate_hz": 20
        },
        {
          "kind": "topic_exists",
          "label": "Mission result",
          "selector": {"suffix": "/result"}
        },
        {
          "kind": "latency",
          "label": "mission path → controller",
          "input": {"name_contains": "mission_path"},
          "output": {"name_contains": "controller_cmd"},
          "target_ms": 100,
          "max_response_ms": 1000
        }
      ]
    }
  ]
}
```

See [examples/custom_rules/warehouse_bot.json](bagx/examples/custom_rules/warehouse_bot.json) for a complete example.

## Plugin distribution

For a team-internal stack, the easiest options are:

- ship a JSON file and point `BAGX_RULE_PLUGIN_PATH` at its directory
- package your rules in Python and expose a `bagx.rules` entry point that returns a JSON dict, a path, or a `CustomRuleSet`

That lets other users run:

```bash
bagx eval my_bag.db3 --rules your_plugin_name
```

## Selector fields

A selector picks which topics a rule applies to. All fields are strings and
optional, but a selector with no fields matches nothing.

| Field | Match against | Notes |
| --- | --- | --- |
| `name` | exact topic name | e.g. `"/cmd_vel"` |
| `name_contains` | substring of topic name | most common form |
| `prefix` | topic name prefix | |
| `suffix` | topic name suffix | |
| `type` | exact message type string | e.g. `"sensor_msgs/msg/Imu"` |
| `type_contains` | substring of message type | |

Unknown fields raise a schema error at load time.

## Check schema

Every check requires `kind` and `label`. Additional fields depend on `kind`.

| `kind` | Required | Optional | Purpose |
| --- | --- | --- | --- |
| `topic_exists` | `selector` | `min_samples`, `severity` | Topic with at least one message must exist. |
| `topic_rate` | `selector`, `min_rate_hz` | `min_samples`, `severity` | Topic must publish above the rate. |
| `latency` | `input`, `output`, `target_ms` | `max_response_ms`, `min_samples`, `severity` | Median delay from input → output must be ≤ `target_ms`. |

All numeric fields (`min_rate_hz`, `target_ms`, `max_response_ms`) must be
numbers; `min_samples` must be an integer. Invalid documents fail loading
with a list of all detected schema errors at once, not one at a time.

### Per-check severity

Each check accepts an optional `severity` field that sets the finding
severity emitted **when the check fails**. Defaults to `warning`. Allowed
values are `info`, `warning`, `error`, and `critical`.

```json
{
  "kind": "topic_rate",
  "label": "Wheel odometry",
  "selector": {"name_contains": "wheel_odom"},
  "min_rate_hz": 20,
  "severity": "error"
}
```

Use `error` for checks that must hold for the bag to be usable at all, and
keep `warning` for checks that flag degradation worth investigating.
A passing check always emits an `info` finding regardless of the configured
severity.

## Per-check findings

Each check emits an individual structured finding alongside the per-domain
aggregate. Use the per-check ids in
[`expected_findings`](benchmark.md#expected_findings) and
[`forbidden_findings`](benchmark.md#forbidden_findings) to gate a single
check rather than the whole custom domain.

Id format:

```
custom.<domain_id>.<label_token>.{pass,fail,skipped}
```

- `<domain_id>` is the domain `name` lowercased with non-alphanumerics
  replaced.
- `<label_token>` is derived from the check `label` the same way.
- The last token reflects evaluation outcome: `pass`, `fail`, or `skipped`
  (latency without enough samples falls into `skipped`).

The aggregate `custom.<domain_id>.evaluated` finding is still emitted and
remains a good high-level signal; reach for per-check ids when you need to
gate individual checks.

## Typical use cases

- internal vehicle stacks with custom odometry / command / mission messages
- custom manipulation stacks that expose action results but not MoveIt topics
- proprietary planners/controllers where topic naming is consistent but message decoding is unavailable
