# Getting Started

## Installation

```bash
pip install bagx
```

### Optional extras

```bash
pip install bagx[mcap]   # .mcap file support
pip install bagx[llm]    # LLM-powered ask command (Anthropic / OpenAI)
```

### From source

```bash
git clone https://github.com/rsasaki0109/bagx.git
cd bagx
pip install -e ".[dev]"
```

## ROS2 dependency

bagx works **with or without ROS2**:

- **With ROS2**: Full message deserialization via `rosbag2_py`
- **Without ROS2**: `.db3` files are read via SQLite with built-in CDR parsers for common types (NavSatFix, Imu, PoseStamped, Odometry, PointCloud2, TFMessage)

## First use

```bash
# Show bag info
bagx info your_bag.db3

# Evaluate quality
bagx eval your_bag.db3

# Export to JSON for inspection
bagx eval your_bag.db3 --json report.json

# Run a benchmark suite
bagx benchmark benchmarks/open_data_suite.json
```

## CLI options

```bash
bagx --version          # Show version
bagx --verbose eval ... # Debug logging
bagx --quiet eval ...   # Errors only
bagx --help             # Show all commands
```

## Supported formats

| Format | Extension | Requirement |
|--------|-----------|-------------|
| SQLite | `.db3` | None (built-in) |
| MCAP | `.mcap` | `pip install bagx[mcap]` |
| Directory | folder with `.db3`/`.mcap` | Auto-detected |
| Multi-segment | folder with multiple files | Aggregated automatically |
