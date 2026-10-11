# Contributing to bagx

Thanks for helping. Bug reports with a bag (or a small excerpt) are the most valuable
contribution: most bagx fixes so far came from running it on real recordings.

## Reporting a problem

Open an issue with the **Bug report** template. The most useful things to include:

- `bagx --version`, Python version, OS, and ROS distro (or "no ROS")
- Bag format (`.db3`, `.mcap`, ROS 1 `.bag`) and `bagx info <bag>` output
- The command you ran and the output (`--json` is best)
- If you can share it, a link to the bag or a trimmed excerpt
  (`ros2 bag convert` / `mcap filter` can cut it down)

A score or finding you think is wrong counts as a bug. Say what you expected and why.

## Development setup

```bash
git clone https://github.com/rsasaki0109/bagx && cd bagx
pip install -e ".[dev,mcap,ros1,mcp]"
python3 -m pytest tests/        # full suite, no ROS needed
ruff check .                    # same lint as CI
```

Test bags are generated in `tests/conftest.py` directly as SQLite, with CDR builders such as
`build_imu_cdr` and `build_laser_scan_cdr`. Add a fixture there rather than committing binary
bags.

### Running against ROS 2

CI also runs the suite on ROS 2 Humble and Jazzy, where `.db3` files are read through
rosbag2_py. `tests/test_backend_parity.py` only runs there and checks that both backends give
the same eval result. To reproduce on Ubuntu 24.04:

```bash
sudo apt install ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-sensor-msgs ros-jazzy-nav-msgs ros-jazzy-tf2-msgs ros-jazzy-rclpy
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv-ros && .venv-ros/bin/pip install -e ".[dev,mcap]"
# ROS installs its own pytest plugins, which break collection of this suite
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-ros/bin/python -m pytest tests/
```

## Making a change

- Keep a PR to one topic, and add or update tests with it.
- If you add a CDR parser, test it against real serialization (see
  `test_cdr_parsers_match_rclpy_serialization_for_any_frame_length`). Alignment bugs only
  show up for some `frame_id` lengths.
- Heavy messages (point clouds, images) must not be held in memory; keep timestamps and small
  counters only.
- A change to how a score is computed must be listed in `CHANGELOG.md` with the affected
  domains, because users gate CI on scores.
- Follow [Versioning and compatibility](docs/guide/versioning.md) for anything that renames
  or removes CLI options, JSON keys or finding ids.
- Add a line under `## [Unreleased]` in `CHANGELOG.md`.

## Adding a stack or rules without changing bagx

Many requests can be handled outside the core:

- Custom topic, type, rate and latency checks: [custom rules](docs/guide/custom-rules.md)
  (JSON files or `bagx.rules` entry points)
- Detecting a new stack: [domain plugins](docs/guide/domain-plugins.md)
  (`bagx.domains` entry points)

## Adding a public dataset to the scoreboard

Add a case to `benchmarks/scoreboard.json` with `provenance` (`public` or `simulator`), a
`source_url` and a `download_hint`. Score it with
`python scripts/generate_scoreboard.py --refresh --write-manifest`, then run
`python scripts/generate_scoreboard.py --check`.
