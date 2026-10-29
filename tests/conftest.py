import json
from pathlib import Path

import pytest

from robot_infra.runtime import Session

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    return json.loads((ROOT / "configs/fake-control.json").read_text())


@pytest.fixture
def session(config):
    s = Session(config, session_id="test-session")
    s.start()
    return s
