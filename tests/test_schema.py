"""Tests for bagx.schema module."""

import pyarrow as pa

from bagx.schema import flatten_message, infer_arrow_type, infer_schema, normalize_timestamp


class TestFlattenMessage:
    def test_flat_dict(self):
        data = {"x": 1.0, "y": 2.0, "z": 3.0}
        result = flatten_message(data)
        assert result == {"x": 1.0, "y": 2.0, "z": 3.0}

    def test_nested_dict(self):
        data = {"position": {"x": 1.0, "y": 2.0}, "name": "test"}
        result = flatten_message(data)
        assert result == {"position.x": 1.0, "position.y": 2.0, "name": "test"}

    def test_deep_nested(self):
        data = {"a": {"b": {"c": 42}}}
        result = flatten_message(data)
        assert result == {"a.b.c": 42}

    def test_skip_underscore_keys(self):
        data = {"x": 1.0, "_raw_size": 100, "_msg_type": "foo"}
        result = flatten_message(data)
        assert result == {"x": 1.0}

    def test_empty_dict(self):
        assert flatten_message({}) == {}


class TestInferArrowType:
    def test_int(self):
        assert infer_arrow_type(42) == pa.int64()

    def test_float(self):
        assert infer_arrow_type(3.14) == pa.float64()

    def test_string(self):
        assert infer_arrow_type("hello") == pa.string()

    def test_bool(self):
        assert infer_arrow_type(True) == pa.bool_()

    def test_list(self):
        result = infer_arrow_type([1.0, 2.0])
        assert result == pa.list_(pa.float64())

    def test_empty_list(self):
        result = infer_arrow_type([])
        assert result == pa.list_(pa.float64())


class TestInferSchema:
    def test_basic(self):
        messages = [
            {"latitude": 35.0, "longitude": 139.0, "altitude": 40.0},
            {"latitude": 35.1, "longitude": 139.1, "altitude": 41.0},
        ]
        schema = infer_schema(messages)

        assert "timestamp_sec" in schema.names
        assert "latitude" in schema.names
        assert "longitude" in schema.names

    def test_skips_underscore(self):
        messages = [{"x": 1.0, "_raw_size": 100}]
        schema = infer_schema(messages)
        assert "_raw_size" not in schema.names


class TestNormalizeTimestamp:
    def test_basic(self):
        result = normalize_timestamp(1700000000, 500000000)
        assert result == 1700000000.5

    def test_zero(self):
        assert normalize_timestamp(0, 0) == 0.0
