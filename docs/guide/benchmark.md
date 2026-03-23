# benchmark — Benchmark Suites

Run a manifest-driven suite of curated bag checks.

This is useful when you want public rosbag datasets and internal gold bags to act like a reproducible regression suite, not ad hoc manual checks.

## Usage

```bash
# Run all cases in a suite
bagx benchmark benchmarks/open_data_suite.json
bagx benchmark benchmarks/non_slam_suite.json
bagx benchmark warehouse_benchmark.json --rules examples/custom_rules/warehouse_bot.json

# Export a machine-readable report
bagx benchmark benchmarks/open_data_suite.json --json benchmark-report.json

# Run only selected cases
bagx benchmark benchmarks/open_data_suite.json --case nvidia-r2b-robotarm

# Fail if any referenced bag is missing
bagx benchmark benchmarks/open_data_suite.json --fail-on-missing
```

## Manifest format

The manifest is JSON and supports environment-variable expansion in `bag_path`.
It also supports optional `rules_path` values to apply custom message rules.

```json
{
  "suite_name": "open-data-dogfood",
  "rules_path": "examples/custom_rules/warehouse_bot.json",
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

For proprietary stacks, pair a benchmark manifest with a custom rules file and keep your expectations in `required_domains`, `required_recommendations`, and `min_topic_rates`.

## Supported expectations

- `min_overall_score`
- `max_overall_score`
- `min_domain_score`
- `required_domains`
- `required_recommendations`
- `forbidden_recommendations`
- `min_topic_rates`
- `required_topics`

## JSON contract

Benchmark JSON reports include:

- `schema_version`
- `report_type`
- `bagx_version`

This makes it practical to gate regressions in CI or compare reports across releases.
