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

See [examples/custom_rules/warehouse_bot.json](/workspace/ai_coding_ws/bagx/examples/custom_rules/warehouse_bot.json) for a complete example.

## Plugin distribution

For a team-internal stack, the easiest options are:

- ship a JSON file and point `BAGX_RULE_PLUGIN_PATH` at its directory
- package your rules in Python and expose a `bagx.rules` entry point that returns a JSON dict, a path, or a `CustomRuleSet`

That lets other users run:

```bash
bagx eval my_bag.db3 --rules your_plugin_name
```

## Supported selectors

- `name`
- `name_contains`
- `prefix`
- `suffix`
- `type`
- `type_contains`

## Supported checks

- `topic_exists`
- `topic_rate`
- `latency`

## Typical use cases

- internal vehicle stacks with custom odometry / command / mission messages
- custom manipulation stacks that expose action results but not MoveIt topics
- proprietary planners/controllers where topic naming is consistent but message decoding is unavailable
