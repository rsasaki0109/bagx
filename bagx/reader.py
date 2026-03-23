"""Rosbag reader abstraction layer.

Wraps rosbag2_py to provide a clean interface for reading bag files.
Supports both .db3 (SQLite) and .mcap formats.
"""

from __future__ import annotations

import hashlib
import logging
import os
import shutil
import sqlite3
import struct
import subprocess
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator

import numpy as np

logger = logging.getLogger(__name__)

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    from mcap.reader import make_reader as mcap_make_reader
    from mcap_ros2.decoder import DecoderFactory as Ros2DecoderFactory

    HAS_MCAP = True
except ImportError:
    HAS_MCAP = False


@contextmanager
def _maybe_suppress_native_stderr() -> Iterator[None]:
    """Suppress native stderr noise when requested by the CLI."""
    if os.environ.get("BAGX_SUPPRESS_NATIVE_STDERR") != "1":
        yield
        return

    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_stderr_fd = os.dup(2)
    try:
        os.dup2(devnull_fd, 2)
        yield
    finally:
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


@dataclass
class TopicInfo:
    name: str
    type: str
    count: int
    serialization_format: str = "cdr"


@dataclass
class Message:
    topic: str
    timestamp_ns: int
    data: dict

    @property
    def timestamp_sec(self) -> float:
        return self.timestamp_ns / 1e9


@dataclass
class BagSummary:
    path: Path
    duration_ns: int
    start_time_ns: int
    end_time_ns: int
    message_count: int
    topics: dict[str, TopicInfo] = field(default_factory=dict)

    @property
    def duration_sec(self) -> float:
        return self.duration_ns / 1e9


def _ros_msg_to_dict(msg) -> dict:
    """Convert a ROS message object to a plain dict recursively."""
    result = {}
    for slot in msg.__slots__:
        attr_name = slot.lstrip("_")
        value = getattr(msg, attr_name)
        if hasattr(value, "__slots__"):
            result[attr_name] = _ros_msg_to_dict(value)
        elif isinstance(value, (list, tuple)):
            converted = []
            for item in value:
                if hasattr(item, "__slots__"):
                    converted.append(_ros_msg_to_dict(item))
                else:
                    converted.append(item)
            result[attr_name] = converted
        elif isinstance(value, np.ndarray):
            result[attr_name] = value.tolist()
        else:
            result[attr_name] = value
    return result


class BagReader:
    """Unified reader for ROS2 bag files."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(f"Bag file not found: {self.path}")

        self._is_mcap = self.path.suffix == ".mcap" or (
            self.path.is_dir() and any(self.path.glob("*.mcap"))
        )
        self._is_db3 = self.path.suffix == ".db3"
        self._summary: BagSummary | None = None
        self._decompressed_db3_paths: list[Path] | None = None

    def summary(self) -> BagSummary:
        if self._summary is not None:
            return self._summary

        if HAS_MCAP and self._is_mcap:
            self._summary = self._summary_mcap()
        elif HAS_ROS:
            try:
                self._summary = self._summary_ros()
            except Exception as exc:
                if self._has_sqlite_storage():
                    logger.info(
                        "ROS backend failed for %s (%s); falling back to SQLite backend",
                        self.path,
                        exc,
                    )
                    self._summary = self._summary_sqlite()
                else:
                    raise
        elif self._has_sqlite_storage():
            self._summary = self._summary_sqlite()
        else:
            raise RuntimeError(
                "rosbag2_py is not available and the bag is not a .db3/.mcap file. "
                "Install ROS2, mcap packages, or provide a .db3 file."
            )
        return self._summary

    def read_messages(
        self, topics: list[str] | None = None
    ) -> Iterator[Message]:
        if HAS_MCAP and self._is_mcap:
            yield from self._read_mcap(topics)
        elif HAS_ROS:
            try:
                yield from self._read_ros(topics)
            except Exception as exc:
                if self._has_sqlite_storage():
                    logger.info(
                        "ROS backend failed for %s (%s); falling back to SQLite backend",
                        self.path,
                        exc,
                    )
                    yield from self._read_sqlite(topics)
                else:
                    raise
        elif self._has_sqlite_storage():
            yield from self._read_sqlite(topics)
        else:
            raise RuntimeError(
                "rosbag2_py is not available and the bag is not a .db3/.mcap file."
            )

    # --- rosbag2_py backend ---

    def _summary_ros(self) -> BagSummary:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.path), storage_id=""
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        with _maybe_suppress_native_stderr():
            reader.open(storage_options, converter_options)

        metadata = reader.get_metadata()
        topics = {}
        for info in metadata.topics_with_message_count:
            ti = info.topic_metadata
            topics[ti.name] = TopicInfo(
                name=ti.name,
                type=ti.type,
                count=info.message_count,
                serialization_format=ti.serialization_format,
            )

        return BagSummary(
            path=self.path,
            duration_ns=metadata.duration.nanoseconds,
            start_time_ns=metadata.starting_time.nanoseconds,
            end_time_ns=metadata.starting_time.nanoseconds
            + metadata.duration.nanoseconds,
            message_count=metadata.message_count,
            topics=topics,
        )

    def _read_ros(self, topics: list[str] | None) -> Iterator[Message]:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.path), storage_id=""
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        with _maybe_suppress_native_stderr():
            reader.open(storage_options, converter_options)

        if topics:
            filter_ = rosbag2_py.StorageFilter(topics=topics)
            reader.set_filter(filter_)

        topic_type_map = {}
        for info in reader.get_all_topics_and_types():
            topic_type_map[info.name] = info.type

        while reader.has_next():
            topic, raw_data, timestamp_ns = reader.read_next()
            msg_type_str = topic_type_map.get(topic, "")
            try:
                msg_type = get_message(msg_type_str)
                msg = deserialize_message(raw_data, msg_type)
                data = _ros_msg_to_dict(msg)
            except Exception:
                # Fall back to basic CDR parser for known types
                data = _parse_cdr_basic(raw_data, msg_type_str)

            yield Message(
                topic=topic, timestamp_ns=timestamp_ns, data=data
            )

    # --- MCAP backend ---

    def _get_mcap_paths(self) -> list[Path]:
        """Return all .mcap files for this bag, sorted by name."""
        if self.path.suffix == ".mcap":
            return [self.path]
        mcap_files = sorted(self.path.glob("*.mcap"))
        if not mcap_files:
            raise FileNotFoundError(
                f"No .mcap file found in {self.path}"
            )
        return mcap_files

    def _summary_mcap(self) -> BagSummary:
        mcap_paths = self._get_mcap_paths()

        all_topics: dict[str, TopicInfo] = {}
        global_start_ns = 0
        global_end_ns = 0
        total_message_count = 0

        for mcap_path in mcap_paths:
            with open(mcap_path, "rb") as f:
                reader = mcap_make_reader(f)
                summary = reader.get_summary()

                if summary is None:
                    raise RuntimeError(
                        f"Could not read summary from {mcap_path}"
                    )

                # Build channel_id -> schema mapping
                schema_map = {
                    sid: s for sid, s in summary.schemas.items()
                }

                # Count messages per channel from statistics
                channel_counts: dict[int, int] = {}
                stats = summary.statistics
                if stats and stats.channel_message_counts:
                    channel_counts = dict(
                        stats.channel_message_counts
                    )

                for channel_id, channel in summary.channels.items():
                    schema = schema_map.get(channel.schema_id)
                    msg_type = schema.name if schema else ""
                    count = channel_counts.get(channel_id, 0)
                    topic_name = channel.topic
                    if topic_name in all_topics:
                        all_topics[topic_name] = TopicInfo(
                            name=topic_name,
                            type=msg_type,
                            count=all_topics[topic_name].count + count,
                            serialization_format=channel.message_encoding
                            or "cdr",
                        )
                    else:
                        all_topics[topic_name] = TopicInfo(
                            name=topic_name,
                            type=msg_type,
                            count=count,
                            serialization_format=channel.message_encoding
                            or "cdr",
                        )

                # Timing from statistics
                if stats:
                    start_ns = stats.message_start_time
                    end_ns = stats.message_end_time
                    msg_count = stats.message_count
                else:
                    start_ns = 0
                    end_ns = 0
                    msg_count = 0

                total_message_count += msg_count
                if global_start_ns == 0 or (
                    start_ns != 0 and start_ns < global_start_ns
                ):
                    global_start_ns = start_ns
                if end_ns > global_end_ns:
                    global_end_ns = end_ns

        return BagSummary(
            path=self.path,
            duration_ns=global_end_ns - global_start_ns,
            start_time_ns=global_start_ns,
            end_time_ns=global_end_ns,
            message_count=total_message_count,
            topics=all_topics,
        )

    def _read_mcap(
        self, topics: list[str] | None
    ) -> Iterator[Message]:
        mcap_paths = self._get_mcap_paths()
        for mcap_path in mcap_paths:
            # Try with decoder first, fall back to raw
            try:
                yield from self._read_mcap_decoded(mcap_path, topics)
            except Exception:
                yield from self._read_mcap_raw(mcap_path, topics)

    def _read_mcap_decoded(
        self, mcap_path: Path, topics: list[str] | None
    ) -> Iterator[Message]:
        with open(mcap_path, "rb") as f:
            reader = mcap_make_reader(
                f, decoder_factories=[Ros2DecoderFactory()]
            )
            kwargs = {}
            if topics:
                kwargs["topics"] = topics
            for (
                schema,
                channel,
                message,
                decoded_msg,
            ) in reader.iter_decoded_messages(**kwargs):
                topic_name = channel.topic
                msg_type = schema.name if schema else ""
                try:
                    if decoded_msg is not None:
                        data = _ros_msg_to_dict(decoded_msg)
                    else:
                        data = _parse_cdr_basic(
                            message.data, msg_type
                        )
                except Exception:
                    data = _parse_cdr_basic(message.data, msg_type)

                yield Message(
                    topic=topic_name,
                    timestamp_ns=message.log_time,
                    data=data,
                )

    def _read_mcap_raw(
        self, mcap_path: Path, topics: list[str] | None
    ) -> Iterator[Message]:
        """Read mcap without decoder, using CDR basic parser as fallback."""
        from mcap.reader import (
            make_reader as mcap_make_reader_raw,
        )

        with open(mcap_path, "rb") as f:
            reader = mcap_make_reader_raw(f)
            kwargs = {}
            if topics:
                kwargs["topics"] = topics
            for schema, channel, message in reader.iter_messages(
                **kwargs
            ):
                topic_name = channel.topic
                msg_type = schema.name if schema else ""
                data = _parse_cdr_basic(message.data, msg_type)
                yield Message(
                    topic=topic_name,
                    timestamp_ns=message.log_time,
                    data=data,
                )

    # --- SQLite fallback backend (no ROS required) ---

    def _has_sqlite_storage(self) -> bool:
        """Return True if the bag is a db3 file or contains db3/db3.zstd files."""
        if self._is_db3:
            return True
        if not self.path.is_dir():
            return False
        return any(self.path.glob("*.db3")) or any(self.path.glob("*.db3.zstd"))

    def _get_db3_paths(self) -> list[Path]:
        """Return all .db3 files for this bag, sorted by name."""
        if self.path.suffix == ".db3":
            return [self.path]
        db3_files = sorted(self.path.glob("*.db3"))
        if db3_files:
            return db3_files

        compressed_db3_files = sorted(self.path.glob("*.db3.zstd"))
        if compressed_db3_files:
            return self._decompress_db3_paths(compressed_db3_files)

        raise FileNotFoundError(
            f"No .db3 file found in {self.path}"
        )

    def _decompress_db3_paths(self, compressed_paths: list[Path]) -> list[Path]:
        """Decompress .db3.zstd files into a reusable temp cache."""
        if self._decompressed_db3_paths is not None:
            return self._decompressed_db3_paths

        zstd_bin = shutil.which("zstd")
        if zstd_bin is None:
            raise RuntimeError(
                "Compressed .db3.zstd bag detected, but 'zstd' command is not available."
            )

        cache_key = hashlib.sha256(str(self.path.resolve()).encode("utf-8")).hexdigest()[:16]
        cache_dir = Path(tempfile.gettempdir()) / "bagx_db3_cache" / cache_key
        cache_dir.mkdir(parents=True, exist_ok=True)

        decompressed_paths: list[Path] = []
        for compressed_path in compressed_paths:
            target_name = compressed_path.name.removesuffix(".zstd")
            output_path = cache_dir / target_name
            stamp_path = cache_dir / f"{target_name}.stamp"
            stat = compressed_path.stat()
            source_sig = f"{compressed_path.resolve()}:{stat.st_size}:{stat.st_mtime_ns}"

            cached_sig = stamp_path.read_text() if stamp_path.exists() else ""
            if not output_path.exists() or cached_sig != source_sig:
                subprocess.run(
                    [zstd_bin, "-d", "-f", str(compressed_path), "-o", str(output_path)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                stamp_path.write_text(source_sig)

            decompressed_paths.append(output_path)

        self._decompressed_db3_paths = decompressed_paths
        return decompressed_paths

    def _summary_sqlite(self) -> BagSummary:
        db_paths = self._get_db3_paths()

        all_topics: dict[str, TopicInfo] = {}
        global_start_ns = 0
        global_end_ns = 0
        total_count = 0

        for db_path in db_paths:
            conn = sqlite3.connect(str(db_path))
            try:
                cursor = conn.cursor()

                cursor.execute(
                    "SELECT id, name, type, serialization_format FROM topics"
                )
                topic_rows = {
                    row[0]: row for row in cursor.fetchall()
                }

                for tid, (_, name, type_, fmt) in topic_rows.items():
                    cursor.execute(
                        "SELECT COUNT(*) FROM messages WHERE topic_id = ?",
                        (tid,),
                    )
                    count = cursor.fetchone()[0]
                    if name in all_topics:
                        all_topics[name] = TopicInfo(
                            name=name,
                            type=type_,
                            count=all_topics[name].count + count,
                            serialization_format=fmt,
                        )
                    else:
                        all_topics[name] = TopicInfo(
                            name=name,
                            type=type_,
                            count=count,
                            serialization_format=fmt,
                        )

                cursor.execute(
                    "SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages"
                )
                row = cursor.fetchone()
                start_ns = row[0] or 0
                end_ns = row[1] or 0
                file_count = row[2] or 0

                total_count += file_count
                if global_start_ns == 0 or (
                    start_ns != 0 and start_ns < global_start_ns
                ):
                    global_start_ns = start_ns
                if end_ns > global_end_ns:
                    global_end_ns = end_ns
            finally:
                conn.close()

        return BagSummary(
            path=self.path,
            duration_ns=global_end_ns - global_start_ns,
            start_time_ns=global_start_ns,
            end_time_ns=global_end_ns,
            message_count=total_count,
            topics=all_topics,
        )

    def _read_sqlite(
        self, topics: list[str] | None
    ) -> Iterator[Message]:
        db_paths = self._get_db3_paths()
        for db_path in db_paths:
            yield from self._read_sqlite_file(db_path, topics)

    def _read_sqlite_file(
        self, db_path: Path, topics: list[str] | None
    ) -> Iterator[Message]:
        """Read messages from a single .db3 file."""
        conn = sqlite3.connect(str(db_path))
        try:
            cursor = conn.cursor()

            cursor.execute(
                "SELECT id, name, type, serialization_format FROM topics"
            )
            topic_map = {}
            for row in cursor.fetchall():
                topic_map[row[0]] = {
                    "name": row[1],
                    "type": row[2],
                    "format": row[3],
                }

            topic_name_to_id = {
                v["name"]: k for k, v in topic_map.items()
            }

            query = "SELECT topic_id, timestamp, data FROM messages"
            params: list = []
            if topics:
                ids = [
                    topic_name_to_id[t]
                    for t in topics
                    if t in topic_name_to_id
                ]
                if ids:
                    placeholders = ",".join("?" * len(ids))
                    query += (
                        f" WHERE topic_id IN ({placeholders})"
                    )
                    params = ids

            query += " ORDER BY timestamp"
            cursor.execute(query, params)

            for topic_id, timestamp_ns, raw_data in cursor:
                info = topic_map.get(topic_id, {})
                topic_name = info.get(
                    "name", f"unknown_{topic_id}"
                )
                msg_type = info.get("type", "")

                data = _parse_cdr_basic(raw_data, msg_type)
                yield Message(
                    topic=topic_name,
                    timestamp_ns=timestamp_ns,
                    data=data,
                )
        finally:
            conn.close()


def _parse_cdr_basic(raw_data: bytes, msg_type: str) -> dict:
    """Best-effort CDR parsing without ROS message definitions.

    For known common types, extract key fields.
    For unknown types, return raw metadata.
    """
    if raw_data is None or len(raw_data) == 0:
        return {"_raw_size": 0, "_msg_type": msg_type}

    parsers = {
        "sensor_msgs/msg/NavSatFix": _parse_navsatfix,
        "sensor_msgs/msg/Imu": _parse_imu,
        "std_msgs/msg/Header": _parse_header,
        "geometry_msgs/msg/PoseStamped": _parse_pose_stamped,
        "geometry_msgs/msg/PoseWithCovarianceStamped": _parse_pose_with_covariance_stamped,
        "geometry_msgs/msg/TwistStamped": _parse_twist_stamped,
        "nav_msgs/msg/Odometry": _parse_odometry,
        "sensor_msgs/msg/PointCloud2": _parse_pointcloud2_meta,
        "tf2_msgs/msg/TFMessage": _parse_tf_message,
    }

    parser = parsers.get(msg_type)
    if parser:
        try:
            return parser(raw_data)
        except (struct.error, ValueError, IndexError):
            return {
                "_raw_size": len(raw_data),
                "_msg_type": msg_type,
                "_parse_error": True,
            }

    return {"_raw_size": len(raw_data), "_msg_type": msg_type}


def _read_cdr_header(data: bytes) -> tuple[int, str]:
    """Read CDR encapsulation header, return (offset, endianness format string)."""
    # CDR encapsulation: 4 bytes header
    if len(data) < 4:
        return 0, "<"
    endian = "<" if data[1] == 0x01 else ">"
    return 4, endian


def _read_cdr_string(buf: bytes, pos: int, endian: str) -> tuple[str, int]:
    """Read a CDR string field and return (decoded_string, next_aligned_pos)."""
    if len(buf) < pos + 4:
        raise ValueError("truncated string length")

    length = struct.unpack_from(f"{endian}I", buf, pos)[0]
    pos += 4
    if length > len(buf) - pos:
        raise ValueError("string length exceeds buffer")

    raw = buf[pos : pos + length]
    value = raw[:-1] if raw.endswith(b"\x00") else raw
    pos += length
    pos = (pos + 3) & ~3
    return value.decode("utf-8", errors="replace"), pos


def _read_std_header_from_buf(buf: bytes, pos: int, endian: str) -> tuple[dict, int]:
    """Read std_msgs/Header payload from a CDR buffer at the given offset."""
    if len(buf) < pos + 8:
        raise ValueError("truncated header stamp")

    sec, nanosec = struct.unpack_from(f"{endian}II", buf, pos)
    pos += 8
    frame_id, pos = _read_cdr_string(buf, pos, endian)
    return {
        "stamp_sec": sec,
        "stamp_nanosec": nanosec,
        "frame_id": frame_id,
    }, pos


def _parse_navsatfix(data: bytes) -> dict:
    """Parse sensor_msgs/msg/NavSatFix from CDR."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]

    # Header: stamp (sec:u32 + nanosec:u32) + frame_id (len:u32 + string)
    if len(buf) < 8:
        return {"_raw_size": len(data), "_parse_error": True}

    sec, nanosec = struct.unpack_from(f"{endian}II", buf, 0)
    if len(buf) < 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    frame_id_len = struct.unpack_from(f"{endian}I", buf, 8)[0]
    if frame_id_len > len(buf) - 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    pos = 12 + frame_id_len
    # Align to 4 bytes
    pos = (pos + 3) & ~3

    # NavSatStatus: status (i8) + service (u16)
    if len(buf) < pos + 4:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
        }

    status = struct.unpack_from(f"{endian}b", buf, pos)[0]
    pos = (pos + 2 + 1) & ~1  # align
    service = struct.unpack_from(f"{endian}H", buf, pos)[0]
    pos += 2
    # Align to 8 bytes for doubles
    pos = (pos + 7) & ~7

    if len(buf) < pos + 24:
        return {
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "status": status,
        }

    latitude, longitude, altitude = struct.unpack_from(
        f"{endian}ddd", buf, pos
    )
    pos += 24

    # position_covariance: 9 doubles
    covariance = []
    if len(buf) >= pos + 72:
        covariance = list(
            struct.unpack_from(f"{endian}9d", buf, pos)
        )
        pos += 72

    covariance_type = 0
    if len(buf) > pos:
        covariance_type = struct.unpack_from(
            f"{endian}B", buf, pos
        )[0]

    return {
        "stamp_sec": sec,
        "stamp_nanosec": nanosec,
        "status": status,
        "service": service,
        "latitude": latitude,
        "longitude": longitude,
        "altitude": altitude,
        "position_covariance": covariance,
        "position_covariance_type": covariance_type,
    }


def _parse_imu(data: bytes) -> dict:
    """Parse sensor_msgs/msg/Imu from CDR."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]

    if len(buf) < 8:
        return {"_raw_size": len(data), "_parse_error": True}

    sec, nanosec = struct.unpack_from(f"{endian}II", buf, 0)
    if len(buf) < 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    frame_id_len = struct.unpack_from(f"{endian}I", buf, 8)[0]
    if frame_id_len > len(buf) - 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    pos = 12 + frame_id_len
    pos = (pos + 7) & ~7  # align to 8 for doubles

    result = {"stamp_sec": sec, "stamp_nanosec": nanosec}

    # orientation: x,y,z,w (4 doubles)
    if len(buf) >= pos + 32:
        ox, oy, oz, ow = struct.unpack_from(
            f"{endian}4d", buf, pos
        )
        result["orientation"] = {
            "x": ox,
            "y": oy,
            "z": oz,
            "w": ow,
        }
        pos += 32

    # orientation_covariance: 9 doubles
    if len(buf) >= pos + 72:
        pos += 72

    # angular_velocity: x,y,z
    if len(buf) >= pos + 24:
        avx, avy, avz = struct.unpack_from(
            f"{endian}3d", buf, pos
        )
        result["angular_velocity"] = {
            "x": avx,
            "y": avy,
            "z": avz,
        }
        pos += 24

    # angular_velocity_covariance: 9 doubles
    if len(buf) >= pos + 72:
        pos += 72

    # linear_acceleration: x,y,z
    if len(buf) >= pos + 24:
        lax, lay, laz = struct.unpack_from(
            f"{endian}3d", buf, pos
        )
        result["linear_acceleration"] = {
            "x": lax,
            "y": lay,
            "z": laz,
        }
        pos += 24

    return result


def _parse_header(data: bytes) -> dict:
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    if len(buf) < 8:
        return {"_raw_size": len(data)}
    sec, nanosec = struct.unpack_from(f"{endian}II", buf, 0)
    return {"stamp_sec": sec, "stamp_nanosec": nanosec}


def _parse_pose_stamped(data: bytes) -> dict:
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    if len(buf) < 8:
        return {"_raw_size": len(data), "_parse_error": True}

    sec, nanosec = struct.unpack_from(f"{endian}II", buf, 0)
    if len(buf) < 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    frame_id_len = struct.unpack_from(f"{endian}I", buf, 8)[0]
    if frame_id_len > len(buf) - 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    pos = 12 + frame_id_len
    pos = (pos + 7) & ~7

    result = {"stamp_sec": sec, "stamp_nanosec": nanosec}

    if len(buf) >= pos + 56:
        px, py, pz = struct.unpack_from(
            f"{endian}3d", buf, pos
        )
        pos += 24
        ox, oy, oz, ow = struct.unpack_from(
            f"{endian}4d", buf, pos
        )
        result["position"] = {"x": px, "y": py, "z": pz}
        result["orientation"] = {
            "x": ox,
            "y": oy,
            "z": oz,
            "w": ow,
        }

    return result


def _parse_pose_with_covariance_stamped(data: bytes) -> dict:
    """Parse geometry_msgs/msg/PoseWithCovarianceStamped from CDR."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    try:
        header, pos = _read_std_header_from_buf(buf, 0, endian)
    except ValueError:
        return {"_raw_size": len(data), "_parse_error": True}

    result = {
        "stamp_sec": header["stamp_sec"],
        "stamp_nanosec": header["stamp_nanosec"],
        "frame_id": header["frame_id"],
    }

    pos = (pos + 7) & ~7
    if len(buf) < pos + 56:
        return result

    px, py, pz, ox, oy, oz, ow = struct.unpack_from(f"{endian}7d", buf, pos)
    pos += 56

    pose = {
        "pose": {
            "position": {"x": px, "y": py, "z": pz},
            "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
        }
    }

    if len(buf) >= pos + 288:
        pose["covariance"] = list(struct.unpack_from(f"{endian}36d", buf, pos))

    result["pose"] = pose
    return result


def _parse_twist_stamped(data: bytes) -> dict:
    """Parse geometry_msgs/msg/TwistStamped from CDR."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    try:
        header, pos = _read_std_header_from_buf(buf, 0, endian)
    except ValueError:
        return {"_raw_size": len(data), "_parse_error": True}

    result = {
        "stamp_sec": header["stamp_sec"],
        "stamp_nanosec": header["stamp_nanosec"],
        "frame_id": header["frame_id"],
    }

    pos = (pos + 7) & ~7
    if len(buf) < pos + 48:
        return result

    lvx, lvy, lvz, avx, avy, avz = struct.unpack_from(f"{endian}6d", buf, pos)
    result["twist"] = {
        "linear": {"x": lvx, "y": lvy, "z": lvz},
        "angular": {"x": avx, "y": avy, "z": avz},
    }
    return result


def _parse_odometry(data: bytes) -> dict:
    """Parse nav_msgs/msg/Odometry from CDR."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    try:
        header, pos = _read_std_header_from_buf(buf, 0, endian)
    except ValueError:
        return {"_raw_size": len(data), "_parse_error": True}

    result = {
        "stamp_sec": header["stamp_sec"],
        "stamp_nanosec": header["stamp_nanosec"],
        "frame_id": header["frame_id"],
    }

    try:
        child_frame_id, pos = _read_cdr_string(buf, pos, endian)
    except ValueError:
        result["_parse_error"] = True
        return result

    result["child_frame_id"] = child_frame_id

    pos = (pos + 7) & ~7
    if len(buf) < pos + 56:
        return result

    px, py, pz, ox, oy, oz, ow = struct.unpack_from(f"{endian}7d", buf, pos)
    pos += 56
    pose = {
        "pose": {
            "position": {"x": px, "y": py, "z": pz},
            "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
        }
    }
    if len(buf) >= pos + 288:
        pose["covariance"] = list(struct.unpack_from(f"{endian}36d", buf, pos))
        pos += 288
    result["pose"] = pose

    if len(buf) < pos + 48:
        return result

    lvx, lvy, lvz, avx, avy, avz = struct.unpack_from(f"{endian}6d", buf, pos)
    pos += 48
    twist = {
        "twist": {
            "linear": {"x": lvx, "y": lvy, "z": lvz},
            "angular": {"x": avx, "y": avy, "z": avz},
        }
    }
    if len(buf) >= pos + 288:
        twist["covariance"] = list(struct.unpack_from(f"{endian}36d", buf, pos))
    result["twist"] = twist

    return result


def _parse_tf_message(data: bytes) -> dict:
    """Parse tf2_msgs/msg/TFMessage from CDR."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    if len(buf) < 4:
        return {"_raw_size": len(data), "_parse_error": True}

    count = struct.unpack_from(f"{endian}I", buf, 0)[0]
    pos = 4
    transforms = []

    for _ in range(count):
        try:
            header, pos = _read_std_header_from_buf(buf, pos, endian)
            child_frame_id, pos = _read_cdr_string(buf, pos, endian)
        except ValueError:
            break

        pos = (pos + 7) & ~7
        if len(buf) < pos + 56:
            break

        tx, ty, tz, qx, qy, qz, qw = struct.unpack_from(f"{endian}7d", buf, pos)
        pos += 56
        transforms.append(
            {
                "stamp_sec": header["stamp_sec"],
                "stamp_nanosec": header["stamp_nanosec"],
                "frame_id": header["frame_id"],
                "child_frame_id": child_frame_id,
                "transform": {
                    "translation": {"x": tx, "y": ty, "z": tz},
                    "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
                },
            }
        )

    result = {"transforms": transforms}
    if count > 0 and not transforms:
        result["_parse_error"] = True
    return result


def _parse_pointcloud2_meta(data: bytes) -> dict:
    """Extract metadata from PointCloud2 (not the full point data)."""
    offset, endian = _read_cdr_header(data)
    buf = data[offset:]
    if len(buf) < 8:
        return {"_raw_size": len(data), "_parse_error": True}

    sec, nanosec = struct.unpack_from(f"{endian}II", buf, 0)
    if len(buf) < 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    frame_id_len = struct.unpack_from(f"{endian}I", buf, 8)[0]
    if frame_id_len > len(buf) - 12:
        return {
            "_raw_size": len(data),
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "_parse_error": True,
        }
    pos = 12 + frame_id_len
    pos = (pos + 3) & ~3

    result = {
        "stamp_sec": sec,
        "stamp_nanosec": nanosec,
        "_raw_size": len(data),
    }

    if len(buf) >= pos + 16:
        height, width = struct.unpack_from(
            f"{endian}II", buf, pos
        )
        result["height"] = height
        result["width"] = width
        result["num_points"] = height * width

    return result
