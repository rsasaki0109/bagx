# benchmark — Benchmark Suites

Run a manifest-driven suite of curated bag checks.

This is useful when you want public rosbag datasets and internal gold bags to act like a reproducible regression suite, not ad hoc manual checks.

## Usage

```bash
# Run all cases in a suite
bagx benchmark benchmarks/open_data_suite.json
bagx benchmark benchmarks/non_slam_suite.json
bagx benchmark warehouse_benchmark.json --rules warehouse_bot

# Export a machine-readable report
bagx benchmark benchmarks/open_data_suite.json --json benchmark-report.json

# Run only selected cases
bagx benchmark benchmarks/open_data_suite.json --case nvidia-r2b-robotarm

# Fail if any referenced bag is missing
bagx benchmark benchmarks/open_data_suite.json --fail-on-missing
```

## Manifest format

The manifest is JSON and supports environment-variable expansion in `bag_path`.
It also supports optional `rules_path` values to apply custom message rules. `rules_path` can be either a JSON file path or a plugin name.

```json
{
  "suite_name": "open-data-dogfood",
  "rules_path": "warehouse_bot",
  "cases": [
    {
      "name": "nvidia-r2b-galileo2",
      "bag_path": "${BAGX_REALBAGS}/r2b_galileo2",
      "report_type": "eval",
      "expect": {
        "min_overall_score": 90,
        "required_domains": ["Perception"],
        "required_recommendations": [
          "Perception topics detected",
          "Camera calibration topics are recorded"
        ],
        "forbidden_recommendations": ["No GNSS data", "No IMU data"]
      }
    }
  ]
}
```

The repository ships two ready-made suites:

- `benchmarks/open_data_suite.json`: public Autoware + NVIDIA bags
- `benchmarks/non_slam_suite.json`: perception/manipulation plus optional local Nav2 / MoveIt dogfood bags

For proprietary stacks, pair a benchmark manifest with a custom rules plugin or file and keep your expectations in `required_domains`, `required_recommendations`, and `min_topic_rates`.

## Supported expectations

- `min_overall_score`
- `max_overall_score`
- `min_domain_score`
- `required_domains`
- `required_recommendations`
- `forbidden_recommendations`
- `expected_findings`
- `min_topic_rates`
- `required_topics`

### expected_findings

`expected_findings` checks structured readiness findings by stable id instead of matching
human-facing recommendation text. See [eval — Structured findings](eval.md#structured-findings)
for the available ids and severity policy.

```json
{
  "expect": {
    "expected_findings": [
      {
        "id": "nav2.detected",
        "severity": "info",
        "domain": "nav2",
        "category": "domain_detection"
      }
    ]
  }
}
```

Each item can be either a bare string (id only) or an object. When an object is
given, `severity`, `domain`, `category`, and `affected_topics` are checked
individually and produce separate `expected_finding_*` sub-checks in the report.

For **temporal findings** (with a `time_range`), an `time_range_overlap`
constraint scopes the match to a window of the bag — pass when the finding's
time_range overlaps the window:

```json
{
  "id": "anomaly.gnss.fix_lost.gnss",
  "time_range_overlap": {"start_ns": 120000000000, "end_ns": 145000000000}
}
```

Useful for "GNSS must be lost only during the parked phase" style
expectations. Requires the eval report to include temporal findings — run
`bagx eval --include-anomaly` upstream.

### forbidden_findings

The inverse of `expected_findings` — fail when listed ids appear. Each item can
be either a bare string (id only) or an object with a `severity_min` scope:

```json
{
  "expect": {
    "forbidden_findings": [
      "nav2.missing_global_plan",
      {"id": "sync.delay.high", "severity_min": "error"}
    ]
  }
}
```

With `severity_min`, the finding is only forbidden when its severity is at or
above the threshold — letting an info-level appearance through while gating on
the worse cases.

`forbidden_findings` also accepts `time_range_overlap` (same shape as in
`expected_findings`). Both qualifiers compose: the rule fires only when a
matching finding has severity ≥ `severity_min` **and** overlaps the
constraint window — useful for "no fix-lost during autonomy phase".

### max_severity

A per-category ceiling. Any finding whose severity exceeds the ceiling fails
its case:

```json
{
  "expect": {
    "max_severity": {
      "sensor_quality": "warning",
      "topic_presence": "info"
    }
  }
}
```

### --exit-on

`bagx benchmark suite.json --exit-on warning` returns a non-zero exit code when
any case's worst finding severity reaches the threshold. Use this together with
the case-level `passed`/`failed` status to gate CI on both manifest expectations
and structural severity.

## JSON contract

Benchmark JSON reports (schema_version 1.3.0+) include:

- `schema_version`
- `report_type`
- `bagx_version`
- `worst_severity` at suite level and on each case
- `finding_ids` for each evaluated case

This makes it practical to gate regressions in CI or compare reports across releases.
