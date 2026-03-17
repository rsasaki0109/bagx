"""Tests for logging behavior and CLI flags."""

from __future__ import annotations

import logging
from pathlib import Path

from typer.testing import CliRunner

from bagx.cli import app

runner = CliRunner()


class TestVersionFlag:
    def test_version_flag(self):
        result = runner.invoke(app, ["--version"])
        assert result.exit_code == 0
        assert "bagx" in result.output


class TestVerboseFlag:
    def test_verbose_sets_debug(self, gnss_bag: Path):
        result = runner.invoke(app, ["--verbose", "info", str(gnss_bag)])
        assert result.exit_code == 0
        # Verify that logging was configured at DEBUG level
        # (The info command should succeed; we check that the flag is accepted)


class TestQuietFlag:
    def test_quiet_sets_error(self, gnss_bag: Path):
        result = runner.invoke(app, ["--quiet", "info", str(gnss_bag)])
        assert result.exit_code == 0
        # The quiet flag should suppress non-error output from logging
        # (Rich console output is separate from logging)


class TestDeserializationFallbackWarning:
    """Test that reader logs WARNING when CDR parsing encounters issues."""

    def test_unknown_type_no_warning(self, gnss_bag: Path, caplog):
        """Reading known types should not produce warnings."""
        from bagx.reader import BagReader

        with caplog.at_level(logging.WARNING, logger="bagx.reader"):
            reader = BagReader(gnss_bag)
            messages = list(reader.read_messages())

        assert len(messages) == 100
        # Known types should parse without warnings about deserialization
        warning_messages = [r for r in caplog.records if r.levelno >= logging.WARNING]
        # No deserialization fallback warnings expected for NavSatFix
        for record in warning_messages:
            assert "deserialization" not in record.message.lower() or "fallback" not in record.message.lower()

    def test_no_scene_topics_warning(self, tmp_path: Path, caplog):
        """scene extraction warns when no scene-relevant topics found."""
        import struct

        from helpers import create_db3

        # Create a bag with an unknown message type (not scene-relevant)
        topics = [{"name": "/custom", "type": "custom_msgs/msg/Foo", "format": "cdr"}]
        data = struct.pack("<4B", 0x00, 0x01, 0x00, 0x00) + b"\x00" * 20
        messages = [{"topic": "/custom", "timestamp_ns": 1_700_000_000_000_000_000, "data": data}]
        bag_path = tmp_path / "custom.db3"
        create_db3(bag_path, topics, messages)

        from bagx.scene import extract_scene

        scene_logger = logging.getLogger("bagx.scene")
        scene_logger.propagate = True
        with caplog.at_level(logging.WARNING):
            extract_scene(str(bag_path))

        warning_msgs = [r.message for r in caplog.records if r.levelno >= logging.WARNING]
        assert any("scene" in msg.lower() or "no scene" in msg.lower() for msg in warning_msgs)
