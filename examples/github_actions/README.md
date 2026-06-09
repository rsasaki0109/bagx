# bagx GitHub Action templates

## Recommended: bagx-action

Use the published composite action instead of copy-pasting shell steps:

```yaml
- uses: rsasaki0109/bagx-action@v1
  with:
    suite: benchmarks/my_suite.json
    fail-on: warning
```

See [bagx-action on GitHub](https://github.com/rsasaki0109/bagx-action) and
[docs/guide/github-action.md](https://rsasaki0109.github.io/bagx/guide/github-action/).

### Templates in this directory

| File | Purpose |
| --- | --- |
| `bagx-action-pr.yml` | PR readiness diff with sticky comment |
| `bagx-action-baseline.yml` | Prime baseline cache on push to `main` |
| `bagx-pr-check.yml` | Legacy hand-written workflow (still works) |

### PR check setup

1. Copy `bagx-action-baseline.yml` → `.github/workflows/bagx-baseline.yml`
2. Copy `bagx-action-pr.yml` → `.github/workflows/bagx-pr-check.yml`
3. Set `bags: bag/recording.db3` to your bag path

The baseline workflow caches `.bagx-action/baseline.json` keyed on the commit SHA.
PR workflows restore that cache using the merge-base SHA and post a markdown diff.

### Tuning

- `fail-on`: change to `error` for stricter gating, or `info` to surface every regression
- `pull-requests: write` permission is required for sticky PR comments
- For shields.io badges, set `badge-gist-id` and a gist-capable `badge-token` secret
