# diff — Compare two eval reports

`bagx diff` compares two `bagx eval --json` reports by stable finding id
(`<domain>.<area>.<qualifier>`). Use it in CI to gate PRs against a baseline
report from `main`, or at release time to track readiness regressions.

## Usage

```bash
bagx diff baseline.json current.json
bagx diff baseline.json current.json --format markdown
bagx diff baseline.json current.json --format json --output diff.json
bagx diff baseline.json current.json --exit-on warning
bagx diff baseline.json current.json --include-same
```

## Change kinds

Each finding id is classified into one of:

| Kind | Meaning |
| ---- | ------- |
| `NEW` | id only in `current` — a finding that was not previously emitted |
| `GONE` | id only in `baseline` — a finding that used to be emitted but no longer is |
| `WORSE` | same id, current severity is higher than baseline |
| `BETTER` | same id, current severity is lower than baseline |
| `SAME` | same id, same severity (hidden by default; use `--include-same`) |

The order of importance in the text rendering is **worse → new → gone → better → same**.

## Evidence drift

When a finding stays the same severity (`SAME`), `bagx diff` still surfaces
*evidence drift* — meaningful changes in numeric observed values within the
finding. The default drift threshold is 10% relative change. Non-numeric
evidence is reported when the string representation differs.

## Temporal findings

Findings with a `time_range` (e.g. `anomaly.gnss.fix_lost.<topic>` from
`bagx eval --include-anomaly`) are matched segment-by-segment within the
same id:

- **Overlapping ranges** count as the *same* segment — a slightly-shifted
  fix-lost window compares as `WORSE` / `BETTER` / `SAME` rather than
  `GONE`+`NEW`.
- **Disjoint ranges** count as *different* segments — a new fault window
  shows up as a separate `NEW` change next to the existing one.

The `text` and `markdown` renderers add a segment column / line showing
`t=<start>-<end>s` for the affected window. Pre-1.3 reports without
`time_range` continue to join by id alone, so existing CI pipelines keep
working unchanged.

## Output formats

- `text` (default): coloured terminal output, one line per change.
- `json`: includes the full diff structure plus `schema_version`,
  `report_type: "diff"`, and `bagx_version`, suitable for further machine
  processing.
- `markdown`: a PR-comment friendly table. Use `--output diff.md` to write
  directly to a file.

## Exit codes

- `0` — no regressions at or above `--exit-on` severity (default behaviour)
- `1` — at least one `NEW` or `WORSE` finding at the requested severity
- `2` — invalid CLI arguments (unknown format, unknown severity)

`--exit-on warning` fails the diff when any `NEW` or `WORSE` finding has
severity ≥ warning. `BETTER` and `GONE` never cause a non-zero exit — they
represent improvements.

## CI example

```bash
# baseline is the eval report on main (cached as a GitHub Actions artifact)
bagx eval bag.db3 --json current.json
bagx diff baseline.json current.json --format markdown --output diff.md --exit-on warning
```

Post `diff.md` as a PR comment, and let `--exit-on warning` block the merge
when readiness regresses.

A ready-to-use GitHub Actions workflow lives at
[`examples/github_actions/bagx-pr-check.yml`](https://github.com/rsasaki0109/bagx/blob/main/examples/github_actions/bagx-pr-check.yml).
It evaluates the PR bag, restores the previous baseline from cache, runs
`bagx diff`, and posts the markdown result as a sticky PR comment.
