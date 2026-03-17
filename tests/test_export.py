"""Tests for bagx.export module."""

import json
from pathlib import Path

import pandas as pd
import pyarrow.parquet as pq
import pytest

from bagx.export import export_bag


class TestExportParquet:
    def test_export_default(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(gnss_bag), str(out_dir))

        assert len(files) == 1
        assert "/gnss" in files
        assert files["/gnss"].suffix == ".parquet"
        assert files["/gnss"].exists()

    def test_parquet_content(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(gnss_bag), str(out_dir))

        table = pq.read_table(str(files["/gnss"]))
        assert table.num_rows == 100
        columns = table.column_names
        assert "timestamp_sec" in columns
        assert "latitude" in columns
        assert "longitude" in columns

    def test_export_multi_topics(self, multi_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(multi_bag), str(out_dir))

        assert len(files) == 3
        assert "/gnss" in files
        assert "/imu" in files
        assert "/lidar" in files

    def test_export_filtered_topics(self, multi_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(multi_bag), str(out_dir), topics=["/gnss"])

        assert len(files) == 1
        assert "/gnss" in files

    def test_export_ai_mode(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(gnss_bag), str(out_dir), ai_mode=True)

        table = pq.read_table(str(files["/gnss"]))
        ts = table.column("timestamp_sec").to_pylist()
        # AI mode: timestamps are relative to bag start, so first should be ~0
        assert ts[0] < 1.0
        # Should be monotonically increasing
        assert all(ts[i] <= ts[i + 1] for i in range(len(ts) - 1))


class TestExportJson:
    def test_export_json(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(gnss_bag), str(out_dir), fmt="json")

        assert len(files) == 1
        path = files["/gnss"]
        assert path.suffix == ".json"

        with open(path) as f:
            data = json.load(f)

        assert isinstance(data, list)
        assert len(data) == 100
        assert "timestamp_sec" in data[0]
        assert "latitude" in data[0]

    def test_json_ai_mode(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(gnss_bag), str(out_dir), fmt="json", ai_mode=True)

        with open(files["/gnss"]) as f:
            data = json.load(f)

        assert data[0]["timestamp_sec"] < 1.0


class TestExportOptions:
    def test_no_flatten(self, imu_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(imu_bag), str(out_dir), fmt="json", flatten=False)

        with open(files["/imu"]) as f:
            data = json.load(f)

        # With no flatten, nested dicts should be preserved as "data" key
        assert "data" in data[0]

    def test_output_dir_created(self, gnss_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "deep" / "nested" / "dir"
        assert not out_dir.exists()

        export_bag(str(gnss_bag), str(out_dir))
        assert out_dir.exists()

    def test_filename_sanitization(self, multi_bag: Path, tmp_path: Path):
        out_dir = tmp_path / "out"
        files = export_bag(str(multi_bag), str(out_dir))

        # Topic names like /gnss should become gnss.parquet
        for path in files.values():
            assert "/" not in path.stem
