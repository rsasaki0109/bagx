# mcp — MCP Server

Expose bagx analysis to LLM agents (Claude Code, Claude Desktop, Cursor, etc.)
via the [Model Context Protocol](https://modelcontextprotocol.io).

Unlike `bagx ask`, **no LLM API key is required**. bagx returns structured JSON;
the host agent does the reasoning.

## Install

```bash
pip install bagx[mcp]
```

## Start the server

```bash
bagx mcp
```

This speaks MCP over **stdio** (default for local agent integrations).

## Tools

| Tool | Description |
|------|-------------|
| `eval_bag(path)` | Quality eval → overall score, findings, recommendations |
| `list_topics(path)` | Topic names, types, counts, bag duration |
| `detect_anomalies(path, topic?)` | GNSS/IMU/rate anomaly report |
| `compare_bags(path_a, path_b)` | Side-by-side metric comparison |
| `query_messages(path, topic, start_ns?, end_ns?, limit?)` | Decoded messages for one topic |

## Claude Code

```bash
claude mcp add bagx -- bagx mcp
```

Or add manually to `.mcp.json`:

```json
{
  "mcpServers": {
    "bagx": {
      "command": "bagx",
      "args": ["mcp"]
    }
  }
}
```

Use the full path to `bagx` if it is not on `PATH` inside Claude Code.

## Claude Desktop

Edit `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "bagx": {
      "command": "bagx",
      "args": ["mcp"]
    }
  }
}
```

macOS config path: `~/Library/Application Support/Claude/claude_desktop_config.json`

## Example agent workflow

1. `list_topics` — discover `/imu`, `/scan`, `/odom`
2. `eval_bag` — get readiness score and domain recommendations
3. `detect_anomalies` — find IMU spikes or GNSS fix loss
4. `query_messages` — inspect decoded fields around a suspect timestamp

## ROS1 bags

Install ROS1 reader support alongside MCP:

```bash
pip install bagx[mcp,ros1]
```

Then point tools at `.bag` files the same way as `.db3` / MCAP directories.
