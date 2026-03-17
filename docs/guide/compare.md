# compare — Bag Comparison

Compare quality metrics of two bag files side-by-side.

## Usage

```bash
bagx compare A.db3 B.db3
bagx compare A.db3 B.db3 --json diff.json
```

## Output

For each metric, shows:

- **A value** and **B value**
- **Diff** (absolute and percentage)
- **Verdict**: `improved`, `degraded`, or `unchanged` (threshold: 1%)

Determines an overall **winner** (A, B, or tie) based on the count of improved vs degraded metrics.

## Example

```
$ bagx compare good_run.db3 bad_run.db3

Bag Comparison
  A: good_run.db3
  B: bad_run.db3
┏━━━━━━━━━━━━━━━━┳━━━━━━━┳━━━━━━━┳━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━┓
┃ Metric         ┃     A ┃     B ┃            Diff ┃   Verdict ┃
┡━━━━━━━━━━━━━━━━╇━━━━━━━╇━━━━━━━╇━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━┩
│ GNSS Fix Rate  │ 0.950 │ 0.720 │ -0.230 (-24.2%) │  degraded │
│ IMU Score      │  84.5 │  91.2 │   +6.7 (+7.9%)  │  improved │
│ Overall Score  │  89.0 │  72.0 │ -17.0 (-19.1%)  │  degraded │
└────────────────┴───────┴───────┴─────────────────┴───────────┘

Result: A is better (2 metrics where B degraded)
```
