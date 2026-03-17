"""Natural language Q&A about bag files using LLM APIs.

Gathers bag context (summary, eval, message samples) and sends it
along with a user question to an LLM for analysis.
"""

from __future__ import annotations

import os


def _build_bag_context(bag_path: str) -> str:
    """Build a text context string from bag summary, eval, and message samples.

    Robust: catches errors at each stage and includes whatever is available.
    """
    sections: list[str] = []

    # 1. Summary
    try:
        from bagx.reader import BagReader

        reader = BagReader(bag_path)
        summary = reader.summary()
        lines = [
            f"Bag path: {summary.path}",
            f"Duration: {summary.duration_sec:.2f} s",
            f"Total messages: {summary.message_count}",
            f"Number of topics: {len(summary.topics)}",
            "",
            "Topics:",
        ]
        for name, info in sorted(summary.topics.items()):
            lines.append(f"  - {name} (type: {info.type}, count: {info.count})")
        sections.append("\n".join(lines))
    except Exception as e:
        sections.append(f"[Summary unavailable: {e}]")
        summary = None
        reader = None

    # 2. Eval
    try:
        from bagx.eval import evaluate_bag

        report = evaluate_bag(bag_path)
        eval_lines = [
            f"Overall quality score: {report.overall_score:.1f}/100",
        ]
        if report.gnss:
            g = report.gnss
            eval_lines.append(
                f"GNSS: fix_rate={g.fix_rate:.1%}, hdop_mean={g.hdop_mean:.2f}, "
                f"score={g.score:.1f}/100"
            )
        if report.imu:
            m = report.imu
            eval_lines.append(
                f"IMU: frequency={m.frequency_hz:.1f}Hz, "
                f"accel_noise=({m.accel_noise_x:.4f},{m.accel_noise_y:.4f},{m.accel_noise_z:.4f}), "
                f"score={m.score:.1f}/100"
            )
        if report.sync:
            s = report.sync
            eval_lines.append(f"Sync score: {s.score:.1f}/100")
            for i, (t1, t2) in enumerate(s.topic_pairs):
                eval_lines.append(
                    f"  {t1} <-> {t2}: mean={s.mean_delay_ms[i]:.1f}ms, "
                    f"max={s.max_delay_ms[i]:.1f}ms"
                )
        sections.append("\n".join(eval_lines))
    except Exception as e:
        sections.append(f"[Eval unavailable: {e}]")

    # 3. Message samples (first 3 from each topic)
    try:
        if reader is not None and summary is not None:
            sample_lines = ["Message samples (first 3 per topic):"]
            topic_counts: dict[str, int] = {}
            for msg in reader.read_messages():
                topic = msg.topic
                topic_counts.setdefault(topic, 0)
                if topic_counts[topic] < 3:
                    sample_lines.append(
                        f"  [{topic}] t={msg.timestamp_sec:.3f}s data={msg.data}"
                    )
                    topic_counts[topic] += 1
                # Stop early if we have enough from all topics
                if all(c >= 3 for c in topic_counts.values()) and len(topic_counts) == len(
                    summary.topics
                ):
                    break
            sections.append("\n".join(sample_lines))
    except Exception as e:
        sections.append(f"[Message samples unavailable: {e}]")

    return "\n\n".join(sections)


def _build_system_prompt(context: str) -> str:
    """Build the system prompt that provides bag context to the LLM."""
    return (
        "You are a ROS2 rosbag analysis assistant. You have been given detailed "
        "information about a rosbag file including its topics, message counts, "
        "duration, quality evaluation scores, and sample messages.\n\n"
        "Use this context to answer the user's question accurately and concisely.\n\n"
        "=== BAG CONTEXT ===\n"
        f"{context}\n"
        "=== END CONTEXT ==="
    )


def _call_anthropic(system_prompt: str, question: str) -> str:
    """Call the Anthropic API using the anthropic SDK."""
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        raise EnvironmentError(
            "ANTHROPIC_API_KEY environment variable is not set. "
            "Set it to use the Anthropic provider."
        )

    import anthropic

    client = anthropic.Anthropic(api_key=api_key)
    message = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=1024,
        system=system_prompt,
        messages=[{"role": "user", "content": question}],
    )
    return message.content[0].text


def _call_openai(system_prompt: str, question: str) -> str:
    """Call the OpenAI API using the openai SDK."""
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise EnvironmentError(
            "OPENAI_API_KEY environment variable is not set. "
            "Set it to use the OpenAI provider."
        )

    import openai

    client = openai.OpenAI(api_key=api_key)
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": question},
        ],
    )
    return response.choices[0].message.content


def ask_bag(bag_path: str, question: str, provider: str = "anthropic") -> str:
    """Ask a natural language question about a bag file, answered by an LLM.

    Args:
        bag_path: Path to the bag file (.db3 or directory).
        question: The natural language question to ask.
        provider: LLM provider - "anthropic" or "openai".

    Returns:
        The LLM's answer as a string.

    Raises:
        ValueError: If the provider is not supported.
        EnvironmentError: If the required API key is not set.
    """
    if provider not in ("anthropic", "openai"):
        raise ValueError(f"Unsupported provider: {provider}. Use 'anthropic' or 'openai'.")

    context = _build_bag_context(bag_path)
    system_prompt = _build_system_prompt(context)

    if provider == "anthropic":
        return _call_anthropic(system_prompt, question)
    else:
        return _call_openai(system_prompt, question)
