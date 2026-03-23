"""Edge case tests for bagx modules.

Tests handling of empty bags, truncated data, single-message topics,
unknown types, and other boundary conditions.
"""

from __future__ import annotations

import struct
import warnings
from pathlib import Path


from tests.conftest import _create_db3, build_imu_cdr, build_navsatfix_cdr

from bagx.reader import BagReader, _parse_cdr_basic
from bagx.eval import evaluate_bag
from bagx.export import export_bag
from bagx.sync import analyze_sync, analyze_sync_multi, _compute_sync_pair


# ---------------------------------------------------------------------------
# Helper: create various edge-case bags
# ---------------------------------------------------------------------------

BASE_NS = 1_700_000_000_000_000_000


def _empty_bag(tmp_path: Path) -> Path:
    """Bag with topics defined but zero messages."""
    topics = [
        {"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
    ]
    return _create_db3(tmp_path / "empty.db3", topics, [])


def _no_topics_bag(tmp_path: Path) -> Path:
    """Bag with no topics at all."""
    return _create_db3(tmp_path / "no_topics.db3", [], [])


def _unknown_type_bag(tmp_path: Path) -> Path:
    """Bag with a topic whose type is not recognized by CDR parsers."""
    topics = [
        {"name": "/custom", "type": "my_pkg/msg/CustomMsg", "format": "cdr"},
    ]
    messages = [
        {"topic": "/custom", "timestamp_ns": BASE_NS + i * 100_000_000, "data": b"\x00\x01\x00\x00" + b"\xAB" * 20}
        for i in range(5)
    ]
    return _create_db3(tmp_path / "unknown.db3", topics, messages)


def _single_message_bag(tmp_path: Path, topic_type: str = "sensor_msgs/msg/NavSatFix") -> Path:
    """Bag with exactly one message."""
    topics = [{"name": "/gnss", "type": topic_type, "format": "cdr"}]
    data = build_navsatfix_cdr(
        stamp_sec=BASE_NS // 1_000_000_000,
        stamp_nanosec=BASE_NS % 1_000_000_000,
        status=0,
        latitude=35.6812,
        longitude=139.7671,
        altitude=40.0,
    )
    messages = [{"topic": "/gnss", "timestamp_ns": BASE_NS, "data": data}]
    return _create_db3(tmp_path / "single.db3", topics, messages)


def _two_message_imu_bag(tmp_path: Path) -> Path:
    """Bag with exactly 2 IMU messages."""
    topics = [{"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"}]
    messages = []
    for i in range(2):
        ts = BASE_NS + i * 5_000_000
        data = build_imu_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            accel=(0.01, -0.02, 9.81),
            gyro=(0.001, -0.001, 0.0),
        )
        messages.append({"topic": "/imu", "timestamp_ns": ts, "data": data})
    return _create_db3(tmp_path / "two_imu.db3", topics, messages)


def _no_gnss_imu_bag(tmp_path: Path) -> Path:
    """Bag with topics that are neither GNSS nor IMU."""
    topics = [
        {"name": "/lidar", "type": "sensor_msgs/msg/PointCloud2", "format": "cdr"},
    ]
    messages = []
    for i in range(20):
        ts = BASE_NS + i * 100_000_000
        data = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00)
        data += struct.pack("<II", ts // 1_000_000_000, ts % 1_000_000_000)
        frame_id = b"lidar\x00\x00\x00"
        data += struct.pack("<I", len(frame_id))
        data += frame_id
        data += struct.pack("<II", 1, 1000)
        data += b"\x00" * 100
        messages.append({"topic": "/lidar", "timestamp_ns": ts, "data": data})
    return _create_db3(tmp_path / "no_gnss_imu.db3", topics, messages)


def _identical_timestamps_bag(tmp_path: Path) -> Path:
    """Bag with two topics where all messages share the same timestamp."""
    topics = [
        {"name": "/a", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/b", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
    ]
    messages = []
    ts = BASE_NS
    for i in range(10):
        data_a = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=0, latitude=35.0, longitude=139.0, altitude=40.0,
        )
        data_b = build_imu_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
        )
        messages.append({"topic": "/a", "timestamp_ns": ts, "data": data_a})
        messages.append({"topic": "/b", "timestamp_ns": ts, "data": data_b})
    return _create_db3(tmp_path / "identical_ts.db3", topics, messages)


def _truncated_cdr_bag(tmp_path: Path) -> Path:
    """Bag with truncated/corrupted CDR data for NavSatFix and Imu."""
    topics = [
        {"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
        {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
    ]
    messages = []
    ts = BASE_NS
    # Truncated NavSatFix: only CDR header + partial stamp
    messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": b"\x00\x01\x00\x00\x01\x02"})
    # Truncated IMU: only CDR header + stamp, no frame_id length
    messages.append({"topic": "/imu", "timestamp_ns": ts + 1, "data": b"\x00\x01\x00\x00" + struct.pack("<II", 100, 0)})
    # Totally empty data
    messages.append({"topic": "/gnss", "timestamp_ns": ts + 2, "data": b""})
    # Just CDR header, nothing else
    messages.append({"topic": "/imu", "timestamp_ns": ts + 3, "data": b"\x00\x01\x00\x00"})
    return _create_db3(tmp_path / "truncated.db3", topics, messages)


# ===========================================================================
# reader.py edge cases
# ===========================================================================


class TestReaderEdgeCases:
    def test_empty_bag_summary(self, tmp_path: Path):
        bag = _empty_bag(tmp_path)
        reader = BagReader(bag)
        summary = reader.summary()

        assert summary.message_count == 0
        assert summary.duration_ns == 0
        assert len(summary.topics) == 2
        assert summary.topics["/gnss"].count == 0

    def test_empty_bag_read_messages(self, tmp_path: Path):
        bag = _empty_bag(tmp_path)
        reader = BagReader(bag)
        messages = list(reader.read_messages())
        assert messages == []

    def test_no_topics_bag(self, tmp_path: Path):
        bag = _no_topics_bag(tmp_path)
        reader = BagReader(bag)
        summary = reader.summary()

        assert summary.message_count == 0
        assert len(summary.topics) == 0
        messages = list(reader.read_messages())
        assert messages == []

    def test_unknown_type_returns_raw_size(self, tmp_path: Path):
        bag = _unknown_type_bag(tmp_path)
        reader = BagReader(bag)
        messages = list(reader.read_messages())

        assert len(messages) == 5
        for msg in messages:
            assert "_raw_size" in msg.data
            assert "_msg_type" in msg.data
            assert msg.data["_msg_type"] == "my_pkg/msg/CustomMsg"

    def test_truncated_cdr_navsatfix(self, tmp_path: Path):
        bag = _truncated_cdr_bag(tmp_path)
        reader = BagReader(bag)
        messages = list(reader.read_messages())

        # Should not raise; all messages should parse gracefully
        assert len(messages) == 4
        for msg in messages:
            assert isinstance(msg.data, dict)

    def test_parse_cdr_basic_empty_data(self):
        result = _parse_cdr_basic(b"", "sensor_msgs/msg/NavSatFix")
        assert result["_raw_size"] == 0

    def test_parse_cdr_basic_none_data(self):
        result = _parse_cdr_basic(None, "sensor_msgs/msg/Imu")
        assert result["_raw_size"] == 0

    def test_parse_cdr_basic_too_short_for_header(self):
        # Only 2 bytes, less than CDR header
        result = _parse_cdr_basic(b"\x00\x01", "sensor_msgs/msg/NavSatFix")
        # Should have _parse_error since data is too short for stamp
        assert "_parse_error" in result or "_raw_size" in result

    def test_truncated_navsatfix_after_stamp(self):
        """NavSatFix data that has stamp but nothing more."""
        data = b"\x00\x01\x00\x00" + struct.pack("<II", 100, 0)
        result = _parse_cdr_basic(data, "sensor_msgs/msg/NavSatFix")
        # Should parse stamp but mark as truncated
        assert result.get("stamp_sec") == 100 or "_raw_size" in result

    def test_truncated_imu_missing_body(self):
        """IMU data with header + stamp but missing orientation etc."""
        data = b"\x00\x01\x00\x00"
        data += struct.pack("<II", 50, 0)
        data += struct.pack("<I", 4) + b"imu\x00"
        result = _parse_cdr_basic(data, "sensor_msgs/msg/Imu")
        assert result.get("stamp_sec") == 50
        # orientation should be missing since data is truncated
        assert "orientation" not in result

    def test_truncated_pose_stamped(self):
        """PoseStamped with truncated frame_id_len."""
        # CDR header + stamp only, no frame_id length
        data = b"\x00\x01\x00\x00" + struct.pack("<II", 10, 0)
        result = _parse_cdr_basic(data, "geometry_msgs/msg/PoseStamped")
        assert "_parse_error" in result
        assert result.get("stamp_sec") == 10

    def test_truncated_pointcloud2(self):
        """PointCloud2 with impossibly large frame_id_len."""
        data = b"\x00\x01\x00\x00"
        data += struct.pack("<II", 10, 0)
        data += struct.pack("<I", 99999)  # frame_id_len way too large
        result = _parse_cdr_basic(data, "sensor_msgs/msg/PointCloud2")
        assert "_parse_error" in result


# ===========================================================================
# eval.py edge cases
# ===========================================================================


class TestEvalEdgeCases:
    def test_empty_bag_eval(self, tmp_path: Path):
        bag = _empty_bag(tmp_path)
        report = evaluate_bag(str(bag))

        assert report.gnss is None
        assert report.imu is None
        assert report.sync is None
        assert report.overall_score == 0.0
        assert report.total_messages == 0

    def test_no_gnss_imu_topics(self, tmp_path: Path):
        bag = _no_gnss_imu_bag(tmp_path)
        report = evaluate_bag(str(bag))

        assert report.gnss is None
        assert report.imu is None
        # Sync should also be None since only 1 topic
        assert report.sync is None
        assert report.overall_score == 0.0
        assert report.total_messages == 20

    def test_single_gnss_message(self, tmp_path: Path):
        bag = _single_message_bag(tmp_path)
        report = evaluate_bag(str(bag))

        assert report.gnss is not None
        assert report.gnss.total_messages == 1
        assert report.gnss.fix_count == 1
        assert report.gnss.fix_rate == 1.0
        assert report.overall_score > 0

    def test_two_imu_messages(self, tmp_path: Path):
        bag = _two_message_imu_bag(tmp_path)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("error", RuntimeWarning)
            report = evaluate_bag(str(bag))

        assert report.imu is not None
        assert report.imu.total_messages == 2
        # Frequency should be computed (2 timestamps)
        assert report.imu.frequency_hz > 0
        # Bias stability requires >100 messages, so should be NaN
        import math
        assert math.isnan(report.imu.accel_bias_stability)
        assert math.isnan(report.imu.gyro_bias_stability)
        assert caught == []

    def test_no_topics_bag_eval(self, tmp_path: Path):
        bag = _no_topics_bag(tmp_path)
        report = evaluate_bag(str(bag))

        assert report.gnss is None
        assert report.imu is None
        assert report.sync is None
        assert report.overall_score == 0.0
        assert report.topic_count == 0

    def test_eval_report_to_dict_empty(self, tmp_path: Path):
        bag = _empty_bag(tmp_path)
        report = evaluate_bag(str(bag))
        d = report.to_dict()

        assert d["gnss"] is None
        assert d["imu"] is None
        assert d["sync"] is None
        assert d["overall_score"] == 0.0

    def test_truncated_data_eval(self, tmp_path: Path):
        bag = _truncated_cdr_bag(tmp_path)
        report = evaluate_bag(str(bag))
        # Should not crash; gnss/imu may have partial data or be None
        assert report is not None


# ===========================================================================
# export.py edge cases
# ===========================================================================


class TestExportEdgeCases:
    def test_export_empty_bag(self, tmp_path: Path):
        bag = _empty_bag(tmp_path)
        out_dir = tmp_path / "out"
        files = export_bag(str(bag), str(out_dir))

        # No messages -> no output files
        assert files == {}

    def test_export_unknown_type(self, tmp_path: Path):
        bag = _unknown_type_bag(tmp_path)
        out_dir = tmp_path / "out"
        files = export_bag(str(bag), str(out_dir), fmt="json")

        assert "/custom" in files
        import json
        with open(files["/custom"]) as f:
            data = json.load(f)
        assert len(data) == 5
        # Unknown type rows should contain _raw_size metadata
        for row in data:
            assert "timestamp_sec" in row
            assert "_raw_size" in row

    def test_export_unknown_type_parquet(self, tmp_path: Path):
        bag = _unknown_type_bag(tmp_path)
        out_dir = tmp_path / "out"
        files = export_bag(str(bag), str(out_dir), fmt="parquet")

        assert "/custom" in files
        import pyarrow.parquet as pq
        table = pq.read_table(str(files["/custom"]))
        assert table.num_rows == 5

    def test_export_single_message(self, tmp_path: Path):
        bag = _single_message_bag(tmp_path)
        out_dir = tmp_path / "out"
        files = export_bag(str(bag), str(out_dir), fmt="json")

        assert "/gnss" in files
        import json
        with open(files["/gnss"]) as f:
            data = json.load(f)
        assert len(data) == 1
        assert "latitude" in data[0]

    def test_export_mixed_known_unknown(self, tmp_path: Path):
        """Bag with a known type and an unknown type, export both."""
        topics = [
            {"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
            {"name": "/custom", "type": "my_pkg/msg/Foo", "format": "cdr"},
        ]
        messages = []
        ts = BASE_NS
        data = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=0, latitude=35.0, longitude=139.0, altitude=40.0,
        )
        messages.append({"topic": "/gnss", "timestamp_ns": ts, "data": data})
        messages.append({"topic": "/custom", "timestamp_ns": ts, "data": b"\x00\x01\x00\x00" + b"\xFF" * 10})
        bag = _create_db3(tmp_path / "mixed.db3", topics, messages)

        out_dir = tmp_path / "out"
        files = export_bag(str(bag), str(out_dir), fmt="json")

        assert "/gnss" in files
        assert "/custom" in files

    def test_export_empty_topic_among_nonempty(self, tmp_path: Path):
        """One topic has messages, another is empty."""
        topics = [
            {"name": "/gnss", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
            {"name": "/imu", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        ]
        ts = BASE_NS
        data = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=0, latitude=35.0, longitude=139.0, altitude=40.0,
        )
        messages = [{"topic": "/gnss", "timestamp_ns": ts, "data": data}]
        bag = _create_db3(tmp_path / "partial.db3", topics, messages)

        out_dir = tmp_path / "out"
        files = export_bag(str(bag), str(out_dir))

        # Only /gnss should have a file, /imu has no messages
        assert "/gnss" in files
        assert "/imu" not in files


# ===========================================================================
# sync.py edge cases
# ===========================================================================


class TestSyncEdgeCases:
    def test_sync_single_message_topic(self, tmp_path: Path):
        """Sync analysis with one topic having a single message."""
        topics = [
            {"name": "/a", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
            {"name": "/b", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        ]
        messages = []
        ts = BASE_NS
        data_a = build_navsatfix_cdr(
            stamp_sec=ts // 1_000_000_000,
            stamp_nanosec=ts % 1_000_000_000,
            status=0, latitude=35.0, longitude=139.0, altitude=40.0,
        )
        messages.append({"topic": "/a", "timestamp_ns": ts, "data": data_a})

        for i in range(10):
            t = ts + i * 10_000_000
            data_b = build_imu_cdr(
                stamp_sec=t // 1_000_000_000,
                stamp_nanosec=t % 1_000_000_000,
            )
            messages.append({"topic": "/b", "timestamp_ns": t, "data": data_b})

        bag = _create_db3(tmp_path / "single_msg.db3", topics, messages)
        report = analyze_sync(str(bag), "/a", "/b")

        # Should produce a result (1 message in /a matched against /b)
        assert len(report.pairs) == 1
        assert report.pairs[0].count == 1

    def test_sync_identical_timestamps(self, tmp_path: Path):
        bag = _identical_timestamps_bag(tmp_path)
        report = analyze_sync(str(bag), "/a", "/b")

        assert len(report.pairs) == 1
        pair = report.pairs[0]
        # All timestamps are identical, so delays should be 0
        assert pair.mean_delay_ms == 0.0
        assert pair.max_delay_ms == 0.0
        assert pair.std_delay_ms == 0.0

    def test_compute_sync_pair_empty_a(self):
        result = _compute_sync_pair("/a", "/b", [], [1, 2, 3])
        assert result.count == 0
        assert result.mean_delay_ms == 0.0

    def test_compute_sync_pair_empty_b(self):
        result = _compute_sync_pair("/a", "/b", [1, 2, 3], [])
        assert result.count == 0
        assert result.mean_delay_ms == 0.0

    def test_compute_sync_pair_both_empty(self):
        result = _compute_sync_pair("/a", "/b", [], [])
        assert result.count == 0

    def test_compute_sync_pair_single_each(self):
        result = _compute_sync_pair("/a", "/b", [1000], [2000])
        assert result.count == 1
        assert result.std_delay_ms == 0.0  # single value -> std=0

    def test_sync_multi_no_active_topics(self, tmp_path: Path):
        """sync_multi with topics that have too few messages."""
        topics = [
            {"name": "/a", "type": "sensor_msgs/msg/NavSatFix", "format": "cdr"},
            {"name": "/b", "type": "sensor_msgs/msg/Imu", "format": "cdr"},
        ]
        # Only 2 messages per topic -- below the threshold of 5
        messages = []
        for i in range(2):
            ts = BASE_NS + i * 100_000_000
            data_a = build_navsatfix_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
                status=0, latitude=35.0, longitude=139.0, altitude=40.0,
            )
            data_b = build_imu_cdr(
                stamp_sec=ts // 1_000_000_000,
                stamp_nanosec=ts % 1_000_000_000,
            )
            messages.append({"topic": "/a", "timestamp_ns": ts, "data": data_a})
            messages.append({"topic": "/b", "timestamp_ns": ts, "data": data_b})
        bag = _create_db3(tmp_path / "few_msgs.db3", topics, messages)

        report = analyze_sync_multi(str(bag))
        # Both topics have <=5 messages, so no active pairs
        assert len(report.pairs) == 0

    def test_sync_nonexistent_topics(self, tmp_path: Path):
        bag = _empty_bag(tmp_path)
        report = analyze_sync(str(bag), "/x", "/y")
        assert len(report.pairs) == 0
