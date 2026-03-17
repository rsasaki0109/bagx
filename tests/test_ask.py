"""Tests for bagx.ask module."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from bagx.ask import _build_bag_context, _build_system_prompt, ask_bag


class TestBuildBagContext:
    def test_context_contains_topic_info(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        assert "/gnss" in context
        assert "NavSatFix" in context

    def test_context_contains_duration(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        assert "Duration" in context

    def test_context_contains_message_count(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        assert "100" in context  # 100 GNSS messages

    def test_context_contains_eval_score(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        assert "score" in context.lower()

    def test_context_contains_message_samples(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        assert "Message samples" in context
        assert "latitude" in context

    def test_context_with_multi_bag(self, multi_bag: Path):
        context = _build_bag_context(str(multi_bag))
        assert "/gnss" in context
        assert "/imu" in context
        assert "/lidar" in context

    def test_context_handles_missing_file(self):
        context = _build_bag_context("/nonexistent/path.db3")
        # Should not raise, but include an error note
        assert "unavailable" in context.lower() or "not found" in context.lower()


class TestBuildSystemPrompt:
    def test_system_prompt_contains_context(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        prompt = _build_system_prompt(context)
        assert "BAG CONTEXT" in prompt
        assert "/gnss" in prompt
        assert "rosbag" in prompt.lower()

    def test_system_prompt_contains_instructions(self, gnss_bag: Path):
        context = _build_bag_context(str(gnss_bag))
        prompt = _build_system_prompt(context)
        assert "answer" in prompt.lower()


class TestAskBagAnthropic:
    @patch("bagx.ask._call_anthropic")
    def test_ask_bag_anthropic(self, mock_call, gnss_bag: Path):
        mock_call.return_value = "The bag contains a GNSS sensor."
        answer = ask_bag(str(gnss_bag), "What sensors are in this bag?", provider="anthropic")
        assert answer == "The bag contains a GNSS sensor."
        mock_call.assert_called_once()
        # Verify the system prompt was passed with bag context
        args = mock_call.call_args
        system_prompt = args[0][0]
        assert "/gnss" in system_prompt

    @patch("bagx.ask._call_anthropic")
    def test_ask_bag_passes_question(self, mock_call, gnss_bag: Path):
        mock_call.return_value = "Yes."
        ask_bag(str(gnss_bag), "Is quality good?", provider="anthropic")
        args = mock_call.call_args
        question = args[0][1]
        assert question == "Is quality good?"


class TestAskBagOpenAI:
    @patch("bagx.ask._call_openai")
    def test_ask_bag_openai(self, mock_call, gnss_bag: Path):
        mock_call.return_value = "GNSS sensor detected."
        answer = ask_bag(str(gnss_bag), "What sensors?", provider="openai")
        assert answer == "GNSS sensor detected."
        mock_call.assert_called_once()


class TestAskBagErrors:
    def test_invalid_provider(self, gnss_bag: Path):
        with pytest.raises(ValueError, match="Unsupported provider"):
            ask_bag(str(gnss_bag), "question", provider="invalid")

    @patch.dict("os.environ", {}, clear=True)
    def test_missing_anthropic_key(self, gnss_bag: Path):
        with pytest.raises(EnvironmentError, match="ANTHROPIC_API_KEY"):
            ask_bag(str(gnss_bag), "question", provider="anthropic")

    @patch.dict("os.environ", {}, clear=True)
    def test_missing_openai_key(self, gnss_bag: Path):
        with pytest.raises(EnvironmentError, match="OPENAI_API_KEY"):
            ask_bag(str(gnss_bag), "question", provider="openai")


class TestCallAnthropicMocked:
    @patch.dict("os.environ", {"ANTHROPIC_API_KEY": "test-key"})
    def test_call_anthropic_sdk(self, gnss_bag: Path):
        """Test that _call_anthropic uses the SDK correctly."""
        import sys

        mock_client = MagicMock()
        mock_message = MagicMock()
        mock_message.content = [MagicMock(text="LLM answer")]
        mock_client.messages.create.return_value = mock_message

        mock_anthropic = MagicMock()
        mock_anthropic.Anthropic.return_value = mock_client

        with patch.dict(sys.modules, {"anthropic": mock_anthropic}):
            from bagx.ask import _call_anthropic

            result = _call_anthropic("system prompt", "question")
            assert result == "LLM answer"
            mock_anthropic.Anthropic.assert_called_once_with(api_key="test-key")
            mock_client.messages.create.assert_called_once()
            call_kwargs = mock_client.messages.create.call_args[1]
            assert call_kwargs["model"] == "claude-sonnet-4-20250514"
            assert call_kwargs["system"] == "system prompt"


class TestCallOpenAIMocked:
    @patch.dict("os.environ", {"OPENAI_API_KEY": "test-key"})
    def test_call_openai_sdk(self, gnss_bag: Path):
        """Test that _call_openai uses the SDK correctly."""
        import sys

        mock_client = MagicMock()
        mock_choice = MagicMock()
        mock_choice.message.content = "OpenAI answer"
        mock_response = MagicMock()
        mock_response.choices = [mock_choice]
        mock_client.chat.completions.create.return_value = mock_response

        mock_openai = MagicMock()
        mock_openai.OpenAI.return_value = mock_client

        with patch.dict(sys.modules, {"openai": mock_openai}):
            from bagx.ask import _call_openai

            result = _call_openai("system prompt", "question")
            assert result == "OpenAI answer"
            mock_openai.OpenAI.assert_called_once_with(api_key="test-key")
            mock_client.chat.completions.create.assert_called_once()
            call_kwargs = mock_client.chat.completions.create.call_args[1]
            assert call_kwargs["model"] == "gpt-4o"
