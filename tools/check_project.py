"""Run reproducible, device-free checks from an installed development checkout."""

from __future__ import annotations

import shutil
import subprocess
import sys
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    """Stop at the first failure; never launch a deployment or physical workflow."""
    npm = shutil.which("npm.cmd") or shutil.which("npm")
    if npm is None:
        print("Node.js 20+ and npm are required for the full development check", file=sys.stderr)
        return 2
    python = sys.executable
    # A fresh leaf avoids deleting another run's files and Windows handles.
    temporary_root = ROOT / "artifacts" / "pytest-runs" / uuid.uuid4().hex
    temporary_root.parent.mkdir(parents=True, exist_ok=True)
    commands = [
        [python, "-m", "ruff", "check", "src", "tests", "tools"],
        [python, "-m", "ruff", "format", "--check", "src", "tests", "tools"],
        [python, "tools/check_publication.py"],
        [python, "-m", "robot_infra", "validate", "configs/fake-control.json", "--dry-run"],
        [
            python,
            "-m",
            "robot_infra",
            "collect-validate",
            "configs/fake-a4s-collection.json",
            "--dry-run",
        ],
        *[
            [python, "-m", "robot_infra", "experiment-plan", str(p.relative_to(ROOT))]
            for p in sorted((ROOT / "configs/graphs").glob("*.json"))
        ],
        [python, "-m", "pytest", "-q", "--basetemp", str(temporary_root)],
        [npm, "run", "format:check"],
        [npm, "test"],
    ]
    for command in commands:
        print("+ " + " ".join(command), flush=True)
        result = subprocess.run(command, cwd=ROOT, check=False)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
