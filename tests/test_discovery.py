import json
import subprocess
import sys
from pathlib import Path

import pytest

from robot_infra.discovery import (
    LinuxSysfsCollector,
    candidate,
    discover,
    load_json,
    main,
    resolve_bindings,
    validate_snapshot,
)


class FixtureCollector:
    name = "fixture"

    def __init__(self, devices, diagnostics=()):
        self.devices, self.diagnostics = devices, list(diagnostics)

    def collect(self):
        return self.devices, self.diagnostics


def camera(serial="CAM-A", path="1-2", resource=None):
    return candidate(
        "usb_device",
        "/sys/bus/usb/devices/" + path,
        resource or "usb:" + path,
        {"serial": serial},
        {},
        [],
    )


def profile(*roles):
    return {"schema_version": "device-bindings/1", "roles": list(roles)}


def role(name="wrist", serial="CAM-A", **kwargs):
    return {
        "role": name,
        "kind": "usb_device",
        "match": {"serial": serial},
        "confirmation": "confirmed",
        **kwargs,
    }


def snapshot(*devices, diagnostics=()):
    return discover([FixtureCollector(list(devices), diagnostics)])


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def test_serial_binding_survives_usb_port_change_without_changing_role():
    before = resolve_bindings(snapshot(camera()), profile(role()))
    after = resolve_bindings(snapshot(camera(path="2-4")), profile(role()))
    assert before["ready"] and after["ready"]
    assert before["roles"][0]["candidates"] != after["roles"][0]["candidates"]


@pytest.mark.parametrize(
    "devices,confirmation,status",
    [
        ([], "confirmed", "missing"),
        ([camera()], "unconfirmed", "matched_unconfirmed"),
        ([camera(), camera(path="2-4")], "confirmed", "ambiguous"),
    ],
)
def test_missing_unknown_role_and_duplicate_serial_do_not_auto_select(
    devices, confirmation, status
):
    r = resolve_bindings(snapshot(*devices), profile(role(confirmation=confirmation)))
    assert not r["ready"] and r["roles"][0]["status"] == status


def test_same_camera_multiple_video_nodes_are_one_exclusive_resource():
    a = candidate("video_interface", "/sys/video0", "usb:1-2", {"port": "/dev/video0"}, {}, [])
    b = candidate("video_interface", "/sys/video1", "usb:1-2", {"port": "/dev/video1"}, {}, [])
    roles = [
        role("a", kind="video_interface", match={"port": "/dev/video0"}),
        role("b", kind="video_interface", match={"port": "/dev/video1"}),
    ]
    r = resolve_bindings(snapshot(a, b), profile(*roles))
    assert not r["ready"] and all(x["status"] == "resource_conflict" for x in r["roles"])
    for x in roles:
        x["exclusive"] = False
    assert resolve_bindings(snapshot(a, b), profile(*roles))["ready"]


def test_partial_discovery_is_not_complete_even_if_a_role_matched():
    r = resolve_bindings(
        snapshot(camera(), diagnostics=[{"status": "enumeration_truncated"}]), profile(role())
    )
    assert not r["ready"] and not r["discovery_complete"]


@pytest.mark.parametrize(
    "change",
    [
        {"match": {}},
        {"match": {"serial": None}},
        {"match": {"serial": ""}},
        {"confirmation": "auto"},
        {"exclusive": "true"},
        {"unknown": 1},
    ],
)
def test_invalid_binding_configuration_is_rejected(change):
    with pytest.raises(ValueError):
        resolve_bindings(snapshot(camera()), profile(role(**change)))


def test_duplicate_role_and_snapshot_identity_rejected():
    with pytest.raises(ValueError, match="unique"):
        resolve_bindings(snapshot(camera()), profile(role(), role()))
    with pytest.raises(ValueError, match="duplicate candidate"):
        snapshot(camera(), camera())


def test_sysfs_enumeration_groups_usb_can_serial_video_without_device_open(tmp_path, monkeypatch):
    sysroot, devroot = tmp_path / "sys", tmp_path / "dev"
    usb = sysroot / "bus/usb/devices/1-2"
    for field, value in {
        "idVendor": "1234",
        "idProduct": "abcd",
        "serial": "ADAPTER",
        "product": "Fixture USB",
        "speed": "5000",
    }.items():
        put(usb / field, value)
    # A symlinked interface is the normal sysfs layout.
    interface = usb / "interface0"
    interface.mkdir()
    original_resolve = Path.resolve
    simulated_links = {}

    def link(path, target, directory=False):
        try:
            path.symlink_to(target, target_is_directory=directory)
        except OSError as exc:
            if getattr(exc, "winerror", None) != 1314:
                raise
            # Windows may lack symlink privileges. Linux exercises real links;
            # this fixture emulates resolution while keeping metadata files real.
            if directory:
                path.mkdir()
            else:
                path.touch()
            simulated_links[path] = target

    monkeypatch.setattr(
        Path,
        "resolve",
        lambda path, *a, **kw: original_resolve(simulated_links.get(path, path), *a, **kw),
    )
    can = sysroot / "class/net/can9"
    put(can / "type", "280")
    put(can / "operstate", "down")
    put(can / "statistics/tx_packets", "0")
    link(can / "device", interface, True)
    for folder, name in [("tty", "ttyUSB9"), ("video4linux", "video9")]:
        entry = sysroot / "class" / folder / name
        entry.mkdir(parents=True)
        link(entry / "device", interface, True)
    put(devroot / "ttyUSB9", "")
    alias = devroot / "serial/by-id/fixture"
    alias.parent.mkdir(parents=True)
    link(alias, devroot / "ttyUSB9")
    original = Path.open

    def protected_open(path, *args, **kwargs):
        assert not path.is_relative_to(devroot), "collector must not open device nodes"
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", protected_open)
    report = discover([LinuxSysfsCollector(sysroot, devroot)])
    assert not report["diagnostics"]
    kinds = {c["kind"]: c for c in report["candidates"]}
    assert set(kinds) == {"usb_device", "can_interface", "serial_port", "video_interface"}
    assert kinds["can_interface"]["identity"]["serial"] == "ADAPTER"
    assert kinds["can_interface"]["state"]["tx_packets"] == "0"
    assert kinds["serial_port"]["identity"]["by_id"] == str(alias)
    assert kinds["video_interface"]["resource_id"] == kinds["usb_device"]["resource_id"]


def test_permission_failure_disconnect_and_limits_are_reported(tmp_path, monkeypatch):
    root = tmp_path / "sys"
    root.mkdir()
    for n in range(3):
        put(root / "bus/usb/devices" / str(n) / "idVendor", "1234")
    collector = LinuxSysfsCollector(root, max_entries=1)
    report = discover([collector])
    assert len(report["candidates"]) == 1
    assert any(d["status"] == "enumeration_truncated" for d in report["diagnostics"])
    original = Path.open

    def deny(path, *args, **kwargs):
        if path.name == "idVendor":
            raise PermissionError("fixture")
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", deny)
    assert any(d["status"] == "PermissionError" for d in discover([collector])["diagnostics"])
    assert discover([LinuxSysfsCollector(tmp_path / "missing")])["diagnostics"]


def test_snapshot_envelope_roundtrip_no_probe_and_exclusive_output(tmp_path, monkeypatch, capsys):
    source, bindings, output = (
        tmp_path / "in.json",
        tmp_path / "bindings.json",
        tmp_path / "out.json",
    )
    source.write_text(json.dumps(snapshot(camera())), encoding="utf-8")
    bindings.write_text(json.dumps(profile(role())), encoding="utf-8")
    monkeypatch.setattr(
        LinuxSysfsCollector, "collect", lambda *a: pytest.fail("snapshot mode must not probe")
    )
    args = ["--snapshot", str(source), "--bindings", str(bindings), "--output", str(output)]
    assert main(args) == 0
    assert main(["--snapshot", str(output), "--bindings", str(bindings)]) == 0
    assert main(args) == 2
    bindings.write_text(json.dumps(profile(role(serial="ABSENT"))), encoding="utf-8")
    assert main(["--snapshot", str(source), "--bindings", str(bindings)]) == 3


def test_cli_and_standalone_bootstrap_share_discovery_contract(tmp_path):
    source = tmp_path / "in.json"
    source.write_text(json.dumps(snapshot(camera())), encoding="utf-8")
    commands = [
        [sys.executable, "-m", "robot_infra", "discover"],
        [sys.executable, "tools/discover_devices.py"],
    ]
    for command in commands:
        p = subprocess.run(command + ["--snapshot", str(source)], capture_output=True, text=True)
        assert p.returncode == 0, p.stderr
        validate_snapshot(json.loads(p.stdout)["discovery"])


def test_json_duplicate_and_nonfinite_values_rejected(tmp_path):
    p = tmp_path / "bad.json"
    for text in ['{"roles":[],"roles":[]}', '{"x":NaN}']:
        p.write_text(text)
        with pytest.raises(ValueError):
            load_json(p)


def test_no_collector_and_template_produce_explicit_unresolved_results():
    report = discover([])
    r = resolve_bindings(report, load_json("configs/device-bindings.example.json"))
    assert not r["ready"] and all(x["status"] == "missing" for x in r["roles"])
