# benchmark — Benchmark Suites

Run a manifest-driven suite of curated bag checks.

This is useful when you want public rosbag datasets and internal gold bags to act like a reproducible regression suite, not ad hoc manual checks.

## Usage

```bash
# Run all cases in a suite
bagx benchmark benchmarks/open_data_suite.json

# Export a machine-readable report
bagx benchmark benchmarks/open_data_suite.json --json benchmark-report.json

# Run only selected cases
bagx benchmark benchmarks/open_data_suite.json --case nvidia-r2b-robotarm

# Fail if any referenced bag is missing
bagx benchmark benchmarks/open_data_suite.json --fail-on-missing
```

## Manifest format

The manifest is JSON and supports environment-variable expansion in `bag_path`.

```json
{
  "suite_name": "open-data-dogfood",
  "cases": [
    {
      "name": "nvidia-r2b-robotarm",
      "bag_path": "${BAGX_REALBAGS}/r2b_robotarm",
      "report_type": "eval",
      "expect": {
        "min_overall_score": 90,
        "required_domains": ["RobotArm"],
        "required_recommendations": [
          "Robot arm perception/manipulation topics detected"
        ],
        "min_topic_rates": {
          "/joint_states": 100
        }
      }
    }
  ]
}
```

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
