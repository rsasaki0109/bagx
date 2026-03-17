#!/usr/bin/env python3
"""Generate CLI reference section for README.md from bagx --help output.

Usage:
    python scripts/generate_docs.py          # Print to stdout
    python scripts/generate_docs.py --update  # Update README.md in-place
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

MARKER_START = "<!-- CLI_REFERENCE_START -->"
MARKER_END = "<!-- CLI_REFERENCE_END -->"


def get_help(command: list[str]) -> str:
    """Run a command and capture its help output."""
    result = subprocess.run(
        command + ["--help"],
        capture_output=True,
        text=True,
        env={**__import__("os").environ, "COLUMNS": "80"},
    )
    return result.stdout.strip()


def generate_cli_reference() -> str:
    """Generate CLI reference markdown from help output."""
    lines = ["## CLI Reference", ""]

    # Main help
    main_help = get_help(["bagx"])
    lines.append("```")
    lines.append(main_help)
    lines.append("```")
    lines.append("")

    # Get subcommands from main help
    subcommands = []
    in_commands = False
    for line in main_help.splitlines():
        if "Commands" in line:
            in_commands = True
            continue
        if in_commands:
            stripped = line.strip()
            if stripped.startswith("│"):
                parts = stripped.strip("│ ").split()
                if parts:
                    subcommands.append(parts[0])
            elif stripped.startswith("╰") or stripped.startswith("└"):
                break

    for cmd in subcommands:
        cmd_help = get_help(["bagx", cmd])
        lines.append(f"### `bagx {cmd}`")
        lines.append("")
        lines.append("```")
        lines.append(cmd_help)
        lines.append("```")
        lines.append("")

    return "\n".join(lines)


def update_readme(readme_path: Path, reference: str) -> None:
    """Update README.md with CLI reference between markers."""
    content = readme_path.read_text()

    if MARKER_START in content and MARKER_END in content:
        before = content[: content.index(MARKER_START)]
        after = content[content.index(MARKER_END) + len(MARKER_END) :]
        new_content = f"{before}{MARKER_START}\n\n{reference}\n\n{MARKER_END}{after}"
    else:
        new_content = content.rstrip() + f"\n\n{MARKER_START}\n\n{reference}\n\n{MARKER_END}\n"

    readme_path.write_text(new_content)
    print(f"Updated {readme_path}")


def main():
    reference = generate_cli_reference()

    if "--update" in sys.argv:
        readme_path = Path(__file__).parent.parent / "README.md"
        update_readme(readme_path, reference)
    else:
        print(reference)


if __name__ == "__main__":
    main()
