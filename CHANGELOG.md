# Changelog

All notable changes to bagx are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow the policy in
[Versioning and compatibility](https://rsasaki0109.github.io/bagx/guide/versioning/).

## [Unreleased]

### Added
- Built-in domain plugin readiness findings for Autoware, MoveIt, Perception,
  RobotArm and Control. Rates use the existing practical targets; each checked
  stream emits `<domain>.<signal>.<topic_token>.{rate_good,rate_low,rate_unavailable}`.
  Missing planning/control counterparts, planned paths, execution visibility and
  camera calibration topics emit documented `*.missing_*` findings. Nav2 ids,
  scores, report keys and versions are unchanged; new warnings can affect CI
  severity gates. Sensing-only Autoware recordings do not require planning/control.
  Public r2b RobotArm (7,916 messages) adds a depth-image rate warning at 14.8 Hz;
  r2b Whitetunnel (17,632 messages) adds eight healthy image-rate findings at
  30.3–31.2 Hz. Their default legacy JSON and scores match the prior code.
- Opt-in `bagx eval --sync-pairing imu` / `EvalConfig(sync_pairing="imu")`:
  pair the best-scoring active IMU with every eligible sensor; without an IMU,
  use all adjacent sensor pairs. Secondary LiDARs and right cameras are no
  longer excluded by the five-pair limit in this mode. Default pairing remains
  `adjacent`; batch, benchmark and scoreboard retain their existing scores.
- Python 3.13 support, with the full test / benchmark smoke suite added to CI
  alongside Python 3.10–3.12 and ROS 2 Humble / Jazzy.
- `bagx eval` header-stamp sync diagnostics for its existing sensor pairs:
  header nearest-neighbor mean/P95 plus relative median `stamp − record_time`
  offsets reveal constant timestamp offsets hidden by dense periodic streams.
  New `sync_quality` findings are
  `sync.header_stamp.<topic_a_token>.<topic_b_token>.measured`, `.offset`,
  `.delay_high`, `.clock_mismatch` and `.unavailable`. Offsets include recording /
  transport latency and are not a physical sensor-delay measurement. Missing,
  zero, sparse, repeated or regressing stamps and incompatible clocks are
  explicitly reported. These add findings / recommendations without changing
  scores, report keys or versions; severity-based CI gates can see new warnings.
- `bagx tf`: TF tree validation (multiple parents, cycles, disconnected trees, transforms on
  both `/tf` and `/tf_static`, pauses and early stops, zero or regressing stamps, clock
  mismatches between topics, sensor `frame_id`s missing from the tree, `--require-frames`,
  `--fail-on`). The same checks run inside `bagx eval` (`tf.*` findings, `TF:`
  recommendations, `report.tf`) and as the MCP tool `check_tf`. They do not change scores.
- Message content checks in `bagx eval` (`msg.*` findings, `Data:` recommendations): scans
  with no or few valid ranges, invalid scan config, beam count changes, uncalibrated or
  zero-size `camera_info`, `camera_info` vs image size mismatch, truncated images,
  resolution or encoding changes, `joint_states` shared by publishers with different joint
  sets, name/position length mismatch, NaN joints, and empty `frame_id`. These do not
  change scores.
- CDR fallback parsers (no ROS needed) for `sensor_msgs` LaserScan, CameraInfo, Image,
  CompressedImage and JointState.
- Header-stamp matched pipeline latency: when a stage's output carries its input's stamp,
  latency is measured exactly per message and reported with `(header-stamp matched)`.
- Scoreboard manifest integrity: `provenance`, `availability`, bag fingerprints and
  `scripts/generate_scoreboard.py --check`.
- CI runs the test suite on ROS 2 Humble and Jazzy, including rosbag2_py vs SQLite backend
  parity tests.

### Changed
- Choosing `--sync-pairing imu` can change sync / overall scores and related
  findings for SLAM, Autoware and perception bags. Nav2 / MoveIt sensor sync
  remains disabled. For the regression fixture of two 10 Hz cameras offset
  by 50 ms plus a 200 Hz IMU, sync is 70.96 (`adjacent`) → 100 (`imu`);
  this is a change in compared pairs, not evidence of hardware alignment.
  Default public scoreboard rows are unchanged (MID-360 overall 98.7 / sync
  100, NTU VIRAL overall 84.0 / sync 100). Public NTU VIRAL `eee_03` validation
  (389,804 messages) expands 5 → 8 pairs, adding the right camera and both
  node2 streams; overall stays 83.9861 → 83.9861 and sync 100 → 100. The added
  pairs' header-neighbor means are 1.2751–1.3208 ms, with relative stamp/record
  offsets −1.4112–+0.1189 ms. Default legacy findings / recommendations match
  main after excluding the added header diagnostics. Keep the pairing mode fixed in
  comparisons / CI gates; alternate-mode scores need their own bag evaluation
  and must not replace the default scoreboard results.
- Public r2b Whitetunnel validation expands `imu` pairing from 5 → 9 pairs.
  Overall remains 91.712 → 91.712 and sync 100 → 100; right-stereo left/right
  streams show header-neighbor means 2.488 ms despite relative stamp/record
  offsets −41.289 / −42.152 ms, producing `offset` warnings. These are timestamp /
  transport observations, not measured physical sensor delays. r2b RobotArm
  stays at overall 95.987 / sync 92.241 in both pairing modes.
- Report `schema_version` 1.3.0 → 1.4.0. This is additive: eval JSON gains `tf` and new
  finding categories `tf_tree` and `message_content`.
- `eval.py` is split into `eval_topics`, `eval_workflow`, `eval_latency` and `eval_domains`.
  This is a code move with no behavior change, and the old `bagx.eval` imports still work.
- Scoreboard: TurtleBot3 Walker and Franka Panda rows are marked `source_unavailable`; r2b
  rows point at NVIDIA NGC. Livox MID-360 is pinned to the AIST Zenodo bag (the same bag the
  0.4.0 score came from) and NTU VIRAL to `eee_03` on DR-NTU. Newer College (download
  needs a request form) is unscored until a sequence is pinned. Ouster OS0-32 is
  `source_unavailable`, because its recording was never tied to a public file. Every
  available public bag is rescored after the sync fix below, and rows scored on unreleased
  code record `bagx_version` as `X.Y.Z+dev`.

### Fixed
- README public-bag examples now match the cached unreleased scoreboard:
  Autoware bag4 77.9 → 82.8, r2b Galileo 90.5 → 90.9, r2b Whitetunnel 91.3 → 91.7
  overall. This corrects stale documentation; no scores are recomputed differently.
- Domain entry-point classes are instantiated correctly, and factory exceptions
  no longer stop the whole eval. Broken imports, constructors or hooks and
  invalid plugin results become `plugin.<name_token>.<hook>_failed` **error**
  findings (`domain_detection`) and `ERROR Plugin:` recommendations. Other checks
  still run, and benchmark severity gates reject incomplete evaluation. Invalid
  or colliding plugin names are reported as unavailable rather than duplicating
  or overriding built-in detection. Healthy-plugin scores and existing findings
  are unchanged; nine framework fixture JSON reports match the prior code after
  excluding new readiness findings.
- Eval JSON serialization no longer recursively copies internal timestamp /
  header-stamp buffers before discarding them. Public JSON, scores and retained
  buffers are unchanged; large reports avoid this unnecessary allocation. On
  the public NTU VIRAL `eee_03` report, traced peak allocations during `to_dict`
  drop from 27.54 MB to 0.10 MB (not total evaluation memory).
- Sync score (`bagx eval`, all SLAM / Autoware / perception bags with two or more sensor
  streams): each pair walked whichever topic sorted first by name and took the nearest
  message on the other. For a dense/sparse pair such as 200 Hz IMU ↔ 10 Hz LiDAR this
  measured a quarter of the LiDAR period (~25 ms) however well the streams were aligned,
  and raised a false "enable per-point deskew" warning. The pair now walks the sparser
  topic, so sync scores rise on bags with mixed rates (Livox MID-360 70 → 100, Autoware bag4
  67.7 → 87.4) and the tuned FAST-LIO comment about deskew disappears where it was spurious.
  `bagx scenario` `sync_degraded` uses the same rule.
- `sensor_msgs` Temperature, MagneticField, FluidPressure, RelativeHumidity, Illuminance and
  BatteryState topics no longer count as sync candidates because their name contains `imu`
  or `camera` (e.g. `/imu/temperature` was paired with a camera on NTU VIRAL).
- NavSatFix CDR parser misread `status` / `service` unless `len(frame_id) + 1` was a
  multiple of 4 (e.g. `gnss_link`), which skewed the GNSS fix rate on the SQLite fallback.
- On ROS 2 Jazzy every rclpy-decoded message failed to convert (`_check_fields` slot), so
  the ROS backend silently fell back to the CDR parser and custom message types lost their
  content.
- `pip install bagx[mcp]` pulled mcp 2.x, which broke `bagx mcp`; mcp is now `<2`.
- pyarrow 26 needs NumPy 2 at import time, so `bagx export` failed on ROS 2 systems with
  NumPy 1.x; pyarrow is now `<26`.
- Ruff rule selection is pinned, so CI no longer breaks when ruff changes its defaults.

## [0.6.0] - 2026-06-10

### Added
- ROS 1 `.bag` support via `pip install bagx[ros1]` (verified on TUM VI and EuRoC).
- `bagx mcp`: stdio MCP server (`eval_bag`, `list_topics`, `detect_anomalies`,
  `compare_bags`, `query_messages`).
- Public scoreboard: 26 datasets scored, with reproduction commands.
- `bagx-action` composite GitHub Action and CI dogfooding.

## [0.5.0] - 2026-06-10

### Added
- `bagx eval --tune` generates FAST-LIO / LIO-SAM / GLIM / KISS-ICP configs from a bag.
- `bagx eval --html` self-contained visual reports; `bagx report` converts saved JSON.

## [0.4.0] - 2026-06-10

### Added
- `Finding.time_range` for temporal findings across eval / anomaly / diff / benchmark.
- Per-check custom rule findings with optional severity.
- Domain plugin SDK for stack detection (`bagx.domains` entry points).
- `bagx eval --badge` shields.io readiness badges.

## [0.3.0] - 2026-05-21

### Added
- `bagx diff`: compare two eval reports by stable finding id (`text` / `markdown` / `json`,
  `--exit-on`).
- `bagx eval --findings-only` and `--severity-min`.
- Benchmark `forbidden_findings`, per-category `max_severity`, and `bagx benchmark --exit-on`.
- `bagx/schema/findings.schema.json` ships with the wheel.
- GitHub Actions PR-check example workflow.

### Changed
- Eval and benchmark JSON `schema_version` 1.1.0 → 1.2.0. Benchmark reports add
  `worst_severity`.

## [0.2.0] - 2026-05-21

### Added
- Structured `findings` in `eval --json` with stable `<domain>.<area>.<qualifier>` ids.
- Benchmark `expected_findings`.
- Custom rules from JSON files, `BAGX_RULE_PLUGIN_PATH` or `bagx.rules` entry points;
  `bagx rules list`.
- Action / service workflow outcome observability and a generic control-loop latency check.
- Built-in `perception` and `robotarm` domains.
- Versioned report contract (`schema_version`, `report_type`, `bagx_version`).

## [0.1.3] - 2026-03-24

### Added
- Pipeline latency between Nav2 / Autoware / MoveIt stages.
- Actionable recommendations at the end of `bagx eval`.

### Fixed
- IMU frequency with bursty timestamps (28306 Hz → 388 Hz).
- Anomaly output summarised instead of listing thousands of events.
- No false `sync_degraded` from `/rosout`; clean errors on corrupt or empty bags.

## [0.1.2] - 2026-03-23

### Fixed
- Meta topics (`/rosout`, `/parameter_events`, `/tf_static`, …) are excluded from scenario
  sync and dropout detection.

## [0.1.1] - 2026-03-18

### Changed
- IMU noise via first-order differencing (= Allan deviation at τ = dt); multi-IMU
  evaluation; full Allan variance on static data.

### Fixed
- Anomaly false positives on real data (31,972 → 640 on Newer College).

## [0.1.0] - 2026-03-18

Initial release: `eval`, `compare`, `sync`, `export`, `anomaly`, `scenario`, `ask`, `scene`,
`batch`, `info`. Works without ROS 2 via the SQLite + CDR fallback; supports `.mcap`,
multi-segment bags and both CDR endiannesses.

[Unreleased]: https://github.com/rsasaki0109/bagx/compare/v0.6.0...HEAD
[0.6.0]: https://github.com/rsasaki0109/bagx/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/rsasaki0109/bagx/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/rsasaki0109/bagx/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/rsasaki0109/bagx/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/rsasaki0109/bagx/compare/v0.1.3...v0.2.0
[0.1.3]: https://github.com/rsasaki0109/bagx/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/rsasaki0109/bagx/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/rsasaki0109/bagx/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/rsasaki0109/bagx/releases/tag/v0.1.0
