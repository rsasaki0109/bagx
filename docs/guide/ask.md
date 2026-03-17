# ask — LLM Queries

Ask natural language questions about a bag file, answered by an LLM.

## Usage

```bash
bagx ask recording.db3 "What sensors are in this bag?"
bagx ask recording.db3 "Is this data suitable for SLAM?" --provider openai
```

## Requirements

```bash
pip install bagx[llm]
```

Set one of:

- `ANTHROPIC_API_KEY` for Claude (default)
- `OPENAI_API_KEY` for GPT-4o

## How it works

1. Gathers bag context automatically:
    - Summary (topics, message counts, duration)
    - Quality evaluation (GNSS/IMU/sync scores)
    - First 3 message samples from each topic
2. Sends the context + your question to the LLM
3. Returns the LLM's analysis

## Providers

| Provider | Model | Flag |
|----------|-------|------|
| Anthropic | claude-sonnet-4-20250514 | `--provider anthropic` (default) |
| OpenAI | gpt-4o | `--provider openai` |
