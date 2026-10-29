import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "host_inventory", Path(__file__).resolve().parents[1] / "tools/collect_host_inventory.py"
)
inventory = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inventory)


def test_windows_inventory_does_not_run_linux_or_sdk_commands():
    def forbidden(argv):
        pytest.fail("unexpected command: " + str(argv))

    data = inventory.collect("A4-S", system="Windows", run=forbidden)
    assert data["status"] == "inventory_only_not_validation"
    assert data["robot_initialization"] == data["camera_streaming"] == "not_run"
    assert all(p["status"] == "not_applicable" for p in data["probes"].values())


def test_linux_inventory_only_uses_fixed_read_queries(tmp_path):
    calls = []

    def run(argv):
        calls.append(argv)
        return {"status": "observed", "argv": argv, "stdout": "test-only"}

    data = inventory.collect("A4-S", system="Linux", usb_root=tmp_path, run=run)
    assert calls == list(inventory.LINUX_PROBES.values())
    assert calls[0] == ["ip", "-details", "-statistics", "link", "show", "type", "can"]
    assert data["usb_devices"]["devices"] == []


def test_sysfs_usb_serials_and_missing_fields(tmp_path):
    device = tmp_path / "1-2"
    device.mkdir()
    (device / "idVendor").write_text("8086\n")
    (device / "serial").write_text("TEST_SERIAL\n")
    data = inventory.usb_inventory(tmp_path)
    assert data["devices"][0]["serial"] == "TEST_SERIAL"
    assert data["devices"][0]["speed"] is None
    assert inventory.usb_inventory(tmp_path / "missing")["status"] == "not_available"


def test_missing_tools_and_timeouts(monkeypatch):
    monkeypatch.setattr(inventory.shutil, "which", lambda name: None)
    assert inventory.run_probe(["lsusb", "-t"])["status"] == "not_available"
    monkeypatch.setattr(inventory.shutil, "which", lambda name: name)

    def timeout(*args, **kwargs):
        assert kwargs["timeout"] == 5 and not kwargs.get("shell", False)
        raise subprocess.TimeoutExpired(args[0], 5)

    monkeypatch.setattr(inventory.subprocess, "run", timeout)
    assert inventory.run_probe(["lsusb", "-t"])["status"] == "failed"


def test_git_paths_are_arguments_not_shell_code():
    calls = []
    inventory.source_inventory(Path("a path with spaces"), lambda argv: calls.append(argv))
    assert all(argv[:3] == ["git", "-C", "a path with spaces"] for argv in calls)
    assert calls[0][3:] == ["rev-parse", "HEAD"]


def test_output_is_exclusive_and_reference_argument_validation(tmp_path, monkeypatch):
    monkeypatch.setattr(
        inventory, "collect", lambda *a, **kw: {"status": "inventory_only_not_validation"}
    )
    output = tmp_path / "host.json"
    assert inventory.main(["--bench", "custom-lab-bench", "--output", str(output)]) == 0
    assert json.loads(output.read_text())["status"] == "inventory_only_not_validation"
    with pytest.raises(FileExistsError):
        inventory.main(["--bench", "A1", "--output", str(output)])
    with pytest.raises(SystemExit):
        inventory.main(["--bench", "A1", "--reference", "missing-equals"])
