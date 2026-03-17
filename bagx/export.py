"""Export rosbag data to AI/analytics-friendly formats (JSON, Parquet)."""

from __future__ import annotations

import json
import logging
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
from rich.console import Console

from bagx.reader import BagReader
from bagx.schema import flatten_message

logger = logging.getLogger(__name__)


def export_bag(
    bag_path: str,
    output_dir: str,
    fmt: str = "parquet",
    topics: list[str] | None = None,
    flatten: bool = True,
    ai_mode: bool = False,
) -> dict[str, Path]:
    """Export bag data to structured files, one per topic.

    Args:
        bag_path: Path to the bag file.
        output_dir: Output directory.
        fmt: Output format ("parquet" or "json").
        topics: Topics to export (None = all).
        flatten: Flatten nested message fields.
        ai_mode: Enable AI-friendly transformations (normalized timestamps, etc.).

    Returns:
        Dict mapping topic names to output file paths.

    Raises:
        ValueError: If fmt is not 'parquet' or 'json'.
    """
    if fmt not in ("parquet", "json"):
        raise ValueError(f"Unsupported format: {fmt}. Use 'parquet' or 'json'.")

    reader = BagReader(bag_path)
    summary = reader.summary()
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if topics is None:
        topics = list(summary.topics.keys())

    # Collect messages by topic
    topic_messages: dict[str, list[dict]] = {t: [] for t in topics}
    start_time_ns = summary.start_time_ns

    for msg in reader.read_messages(topics=topics):
        if msg.topic not in topic_messages:
            continue

        row = {}

        # Timestamp normalization
        if ai_mode:
            # Relative timestamp from bag start, in float seconds
            row["timestamp_sec"] = (msg.timestamp_ns - start_time_ns) / 1e9
        else:
            row["timestamp_sec"] = msg.timestamp_ns / 1e9

        row["timestamp_ns"] = msg.timestamp_ns

        # Message data
        if flatten:
            flat = flatten_message(msg.data)
            # If flattening produced no fields (e.g., unknown msg type with only _raw_size),
            # include raw metadata so the row isn't empty
            if not flat:
                raw_size = msg.data.get("_raw_size")
                if raw_size is not None:
                    row["_raw_size"] = raw_size
                msg_type = msg.data.get("_msg_type")
                if msg_type is not None:
                    row["_msg_type"] = msg_type
            else:
                row.update(flat)
        else:
            row["data"] = msg.data

        topic_messages[msg.topic].append(row)

    # Write output files
    output_files: dict[str, Path] = {}

    for topic_name, rows in topic_messages.items():
        # Skip topics with no messages
        if not rows:
            continue

        # Warn if all messages are stub data
        if all("_raw_size" in r and len(r) <= 3 for r in rows):  # timestamp_ns, timestamp_sec, _raw_size
            logger.warning("Topic %s contains only stub data (no decoded fields)", topic_name)

        # Sanitize topic name for filename
        safe_name = topic_name.strip("/").replace("/", "_")

        if fmt == "parquet":
            path = out_dir / f"{safe_name}.parquet"
            _write_parquet(rows, path, flatten)
            output_files[topic_name] = path

        elif fmt == "json":
            path = out_dir / f"{safe_name}.json"
            _write_json(rows, path)
            output_files[topic_name] = path

    return output_files


def _write_parquet(rows: list[dict], path: Path, flatten: bool) -> None:
    """Write rows to a Parquet file."""
    # Collect all column names
    columns: dict[str, list] = {}
    all_keys: list[str] = []

    # First pass: gather keys
    for row in rows[:100]:
        for key in row:
            if key not in columns:
                all_keys.append(key)
                columns[key] = []

    # Reset
    columns = {k: [] for k in all_keys}

    # Second pass: collect values
    for row in rows:
        for key in all_keys:
            val = row.get(key)
            # Convert non-serializable types
            if isinstance(val, (list, tuple)):
                val = _try_numeric_list(val)
            elif isinstance(val, dict):
                val = json.dumps(val)
            elif isinstance(val, np.ndarray):
                val = val.tolist()
            columns[key].append(val)

    # Build DataFrame and write
    df = pd.DataFrame(columns)
    table = pa.Table.from_pandas(df)
    pq.write_table(table, str(path))


def _write_json(rows: list[dict], path: Path) -> None:
    """Write rows to a JSON file."""
    cleaned = []
    for row in rows:
        cleaned.append(_json_serialize(row))

    with open(path, "w") as f:
        json.dump(cleaned, f, indent=2)


def _json_serialize(obj):
    """Recursively make objects JSON-serializable."""
    if isinstance(obj, dict):
        return {k: _json_serialize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_serialize(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, bytes):
        return f"<bytes:{len(obj)}>"
    return obj


def _try_numeric_list(val):
    """Try to keep list values as-is if they're numeric, else JSON-encode."""
    if not val:
        return val
    if all(isinstance(v, (int, float)) for v in val):
        return val
    return json.dumps(val)


def print_export_summary(
    output_files: dict[str, Path], console: Console | None = None
) -> None:
    """Print export results."""
    if console is None:
        console = Console()

    console.print(f"\n[bold]Exported {len(output_files)} topic(s):[/bold]")
    for topic, path in output_files.items():
        size_kb = path.stat().st_size / 1024
        console.print(f"  {topic} → [cyan]{path}[/cyan] ({size_kb:.1f} KB)")
    console.print()
