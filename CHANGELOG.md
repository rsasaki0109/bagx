# Changelog

All notable changes to bagx are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow the policy in
[Versioning and compatibility](https://rsasaki0109.github.io/bagx/guide/versioning/).

## [Unreleased]

### Added
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
