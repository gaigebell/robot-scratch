"""Exercise the installed Bash entry with a Python stub, never ROS or devices."""

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    sys.platform != "linux" or not shutil.which("bash"), reason="device Bash entry requires Linux"
)


@pytest.fixture
def launcher(tmp_path):
    root = tmp_path / "device root"
    (root / "bin").mkdir(parents=True)
    script = root / "bin/robot-infra"
    script.write_bytes((Path(__file__).parents[1] / "tools/robot-infra-device.sh").read_bytes())
    fake_bin = tmp_path / "stub-bin"
    fake_bin.mkdir()
    stub = fake_bin / "python3"
    stub.write_text(
        f"#!{sys.executable}\n"
        + """
import json,os,sys
if sys.argv[1:3]==['-m','robot_infra']:
    print(json.dumps({'args':sys.argv[3:],'setup':os.environ.get('LAUNCHER_SETUP_MARKER'),'pythonpath':os.environ.get('PYTHONPATH')}))
else:
    os.execv(sys.executable,[sys.executable,*sys.argv[1:]])
"""
    )
    stub.chmod(0o755)

    def config(name, marker):
        p = root / name
        p.parent.mkdir(parents=True, exist_ok=True)
        setup = p.parent / (marker + ".bash")
        setup.write_text("export LAUNCHER_SETUP_MARKER=" + marker + "\n")
        p.write_text(json.dumps({"setup_scripts": [setup.name], "python_paths": ["src-" + marker]}))
        return p

    default = config("deployment.json", "default")
    alternate = config("profiles/alternate deployment.json", "alternate")
    env = os.environ.copy()
    env["PATH"] = str(fake_bin) + os.pathsep + env["PATH"]
    env.pop("ROBOT_INFRA_DEPLOYMENT", None)

    def invoke(arguments, override=None):
        selected = env.copy()
        if override is not None:
            selected["ROBOT_INFRA_DEPLOYMENT"] = str(override)
        return subprocess.run(
            [shutil.which("bash"), str(script), *arguments],
            cwd=root,
            env=selected,
            capture_output=True,
            text=True,
            timeout=10,
        )

    return invoke, default, alternate


@pytest.mark.parametrize("selection", ["default", "environment", "explicit", "equals"])
def test_device_entry_uses_one_manifest_for_cli_and_environment(launcher, selection):
    invoke, default, alternate = launcher
    arguments = ["collect", "--task", "demo"]
    override = None
    expected = default
    if selection == "environment":
        override = alternate
        expected = alternate
    elif selection == "explicit":
        arguments += ["--deployment", str(alternate)]
        override = default
        expected = alternate
    elif selection == "equals":
        arguments += ["--deployment=" + str(alternate)]
        override = default
        expected = alternate
    r = invoke(arguments, override)
    assert r.returncode == 0, r.stderr
    value = json.loads(r.stdout)
    assert value["setup"] == ("default" if expected == default else "alternate")
    assert value["pythonpath"].split(os.pathsep)[0] == str(
        expected.parent / ("src-" + value["setup"])
    )
    args = value["args"]
    occurrences = [
        x.split("=", 1)[1] if x.startswith("--deployment=") else args[i + 1]
        for i, x in enumerate(args)
        if x == "--deployment" or x.startswith("--deployment=")
    ]
    assert occurrences == [str(expected)]


def test_device_entry_offline_command_does_not_inject_deployment_argument(launcher):
    invoke, _, _ = launcher
    r = invoke(["episode-qc", "example", "--deep"])
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout)["args"] == ["episode-qc", "example", "--deep"]


@pytest.mark.parametrize(
    "arguments",
    [
        ["collect", "--deployment"],
        ["collect", "--deployment="],
        ["collect", "--deployment", "--task", "demo"],
    ],
)
def test_device_entry_rejects_missing_deployment_value(launcher, arguments):
    invoke, _, _ = launcher
    r = invoke(arguments)
    assert r.returncode == 2 and "--deployment requires a file path" in r.stderr
