"""Schema inference and normalization for rosbag messages."""

from __future__ import annotations

from typing import Any

import numpy as np
import pyarrow as pa


def infer_arrow_type(value: Any) -> pa.DataType:
    """Infer a PyArrow type from a Python value."""
    if isinstance(value, bool):
        return pa.bool_()
    if isinstance(value, int):
        return pa.int64()
    if isinstance(value, float):
        return pa.float64()
    if isinstance(value, str):
        return pa.string()
    if isinstance(value, bytes):
        return pa.binary()
    if isinstance(value, (list, tuple)):
        if len(value) == 0:
            return pa.list_(pa.float64())
        return pa.list_(infer_arrow_type(value[0]))
    if isinstance(value, np.ndarray):
        return pa.list_(pa.from_numpy_dtype(value.dtype))
    if isinstance(value, dict):
        fields = []
        for k, v in value.items():
            fields.append(pa.field(k, infer_arrow_type(v)))
        return pa.struct(fields)
    return pa.string()


def infer_schema(messages: list[dict]) -> pa.Schema:
    """Infer a PyArrow schema from a list of message dicts.

    Scans the first N messages to build a merged schema.
    """
    field_types: dict[str, pa.DataType] = {}

    sample = messages[:100]
    for msg in sample:
        for key, value in msg.items():
            if key.startswith("_"):
                continue
            if key not in field_types and value is not None:
                field_types[key] = infer_arrow_type(value)

    fields = [pa.field("timestamp_sec", pa.float64())]
    for name, dtype in sorted(field_types.items()):
        fields.append(pa.field(name, dtype))

    return pa.schema(fields)


def flatten_message(data: dict, prefix: str = "") -> dict:
    """Flatten a nested message dict into dot-separated keys."""
    result = {}
    for key, value in data.items():
        if key.startswith("_"):
            continue
        full_key = f"{prefix}{key}" if not prefix else f"{prefix}.{key}"
        if isinstance(value, dict):
            result.update(flatten_message(value, full_key))
        else:
            result[full_key] = value
    return result


def normalize_timestamp(stamp_sec: int, stamp_nanosec: int) -> float:
    """Convert ROS stamp to float seconds."""
    return stamp_sec + stamp_nanosec / 1e9
