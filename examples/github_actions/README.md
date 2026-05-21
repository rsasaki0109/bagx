# bagx GitHub Action templates

Workflow templates that pair `bagx eval`, `bagx diff`, and `bagx benchmark`
with GitHub PR automation. Copy any file from this directory into your repo's
`.github/workflows/` directory to enable it.

## bagx-pr-check.yml

Runs on every PR that touches a `.db3` or `.mcap` file. Evaluates the bag,
compares it against a cached baseline from `main`, and posts the markdown
diff as a sticky PR comment. Fails the check when readiness regresses at or
above the configured severity floor (default `warning`).

### Setup

1. Copy `bagx-pr-check.yml` to `.github/workflows/bagx-pr-check.yml`.
2. Set `BAG_PATH` to the bag location in your repo (or replace the `bagx eval`
   step with one that fetches the bag from object storage / LFS).
3. Add a sibling workflow that primes the baseline cache on push to `main`:

```yaml
name: bagx prime baseline
on:
  push:
    branches: [main]

jobs:
  prime:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.12"
      - run: pip install "bagx>=0.3.0"
      - run: bagx eval bag/recording.db3 --json baseline.json
      - uses: actions/cache/save@v4
        with:
          path: baseline.json
          key: bagx-eval-baseline-${{ github.sha }}
```

The PR workflow's `actions/cache` step looks up `bagx-eval-baseline-<merge-base-sha>`
so any PR targeting that main commit shares the same baseline.

### Tuning

- `SEVERITY_FLOOR`: change to `error` for stricter gating, or `info` to
  surface every regression. Used by both the terminal log
  (`--severity-min`) and the diff exit code (`--exit-on`).
- `pull-requests: write` permission is required for the sticky comment.
- The artifact upload retains 90 days of historical eval reports by default,
  useful for retroactively comparing against any past PR.
