# Versioning and compatibility

bagx is used as a CI gate, so its outputs are a contract. This page defines what
counts as a breaking change and how changes are announced.

## What is covered

| Surface | Examples |
|---------|----------|
| CLI | Command names, option names, exit codes (`--exit-on`, `--fail-on`, benchmark failures) |
| JSON reports | Keys and types in `eval` / `benchmark` / `diff` / `anomaly` / `tf` JSON, `schema_version` |
| Finding ids | `<domain>.<area>.<qualifier>` ids, `severity` and `category` values |
| Findings schema | `bagx/schema/findings.schema.json` |
| Python API | `bagx.eval.evaluate_bag`, `EvalReport` / `EvalConfig` fields, `bagx.reader.BagReader`, `bagx.tf_tree.analyze_tf`, the domain plugin protocol, custom rule file format |

Names that start with `_` are internal and can change at any time. Some are
re-exported for plugins (`bagx.eval._select_topics`); those follow the deprecation
rule below.

## Version numbers

bagx uses `MAJOR.MINOR.PATCH`.

- **PATCH** (0.6.0 → 0.6.1): bug fixes only. No new options, no removed keys.
  A fix may change a score or a finding when the old value was wrong; the changelog
  lists every such change under **Fixed**.
- **MINOR** (0.6 → 0.7): new commands, options, findings or report keys. Before 1.0,
  a minor release may also remove or rename something, but only after the
  deprecation period below.
- **MAJOR** (1.0 → 2.0): after 1.0, removals and renames happen only here.

## Deprecation

1. The old name keeps working for at least one minor release and emits a
   `DeprecationWarning` (Python) or a warning line on stderr (CLI).
2. The changelog lists it under **Deprecated**, with the replacement.
3. It is removed in a later minor release (before 1.0) or the next major (after 1.0),
   listed under **Removed**.

Finding ids get the same treatment. A renamed id is emitted under both names for the
deprecation period, so benchmark manifests and `bagx diff` baselines keep matching.

## Report `schema_version`

JSON reports carry their own `schema_version`, independent of the package version:

- **minor** bump: additive changes (new keys, new finding categories). Consumers that
  ignore unknown keys keep working.
- **major** bump: a key is removed or its type or meaning changes.

## Scores

Readiness scores feed CI thresholds (`min_overall_score`), so any change to how a score
is computed is called out in the changelog with the affected domains. Checks that only
add findings and recommendations (TF tree, message content) are designed not to change
scores.

## Supported environments

- Python: the versions in the CI matrix (currently 3.10–3.13).
- ROS: works without ROS. With ROS 2, CI covers Humble and Jazzy through rosbag2_py.
- Optional extras (`mcap`, `ros1`, `mcp`, `llm`) are pinned below the next major
  version once a breaking upstream release is seen (e.g. `mcp<2`).
