"""Bootstrap passive discovery from a source checkout without installing SDKs."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from robot_infra.discovery import main

if __name__ == "__main__":
    raise SystemExit(main())
