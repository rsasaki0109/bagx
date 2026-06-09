"""Generate tiny benchmark bags and run the benchmark CLI as a smoke test."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

def main() -> None:
    from bagx.contracts import REPORT_SCHEMA_VERSION
    from scripts.prepare_ci_benchmark import prepare_ci_benchmark

    with tempfile.TemporaryDirectory(prefix="bagx-benchmark-smoke-") as tmp_dir:
        tmp_path = Path(tmp_dir)
        manifest_path = prepare_ci_benchmark(tmp_path)
        report_path = tmp_path / "benchmark-report.json"

        subprocess.run(
            [
                sys.executable,
                "-m",
                "bagx.cli",
                "benchmark",
                str(manifest_path),
                "--json",
                str(report_path),
            ],
            check=True,
        )

        with open(report_path) as f:
            data = json.load(f)

        if data["schema_version"] != REPORT_SCHEMA_VERSION:
            raise SystemExit(f"unexpected schema_version: {data['schema_version']}")
        if data["report_type"] != "benchmark_suite":
            raise SystemExit(f"unexpected report_type: {data['report_type']}")
        if data["failed_cases"] != 0 or data["passed_cases"] != 4:
            raise SystemExit(f"unexpected benchmark result: {data}")


if __name__ == "__main__":
    main()
