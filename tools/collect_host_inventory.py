"""Read-only host inventory. No SDK imports, ROS nodes, camera streaming or CAN writes.

Intended to run with Python 3.8+ on existing Ubuntu installations; only the
actually tested environments are evidence. Collection success is not bench admission.
"""

import argparse
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path

LINUX_PROBES = {
    "can_links": ["ip", "-details", "-statistics", "link", "show", "type", "can"],
    "usb_topology": ["lsusb", "-t"],
    "pci_devices": ["lspci", "-nn"],
}
PACKAGES = ("robot-infra", "mcap", "jsonschema", "pyAgxArm", "piper_sdk", "pyrealsense2")


def run_probe(argv):
    if shutil.which(argv[0]) is None:
        return {"status": "not_available", "argv": argv}
    try:
        proc = subprocess.run(
            argv, capture_output=True, text=True, errors="replace", timeout=5, check=False
        )
        return {
            "status": "observed" if proc.returncode == 0 else "failed",
            "argv": argv,
            "exit_code": proc.returncode,
            "stdout": proc.stdout[:16000],
            "stderr": proc.stderr[:4000],
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"status": "failed", "argv": argv, "error": type(exc).__name__ + ": " + str(exc)}


def usb_inventory(root):
    """Read sysfs attributes only, without opening or configuring USB devices."""
    devices = []
    if not root.is_dir():
        return {"status": "not_available", "devices": devices}
    try:
        for path in sorted(root.iterdir()):
            if not (path / "idVendor").exists():
                continue
            device = {"sysfs_id": path.name}
            for field in (
                "idVendor",
                "idProduct",
                "product",
                "serial",
                "busnum",
                "devpath",
                "speed",
            ):
                try:
                    device[field] = (path / field).read_text().strip()
                except FileNotFoundError:
                    device[field] = None
                except OSError as exc:
                    device[field] = {"status": "unreadable", "error": type(exc).__name__}
            devices.append(device)
        return {"status": "observed", "devices": devices}
    except OSError as exc:
        return {"status": "failed", "error": type(exc).__name__, "devices": devices}


def source_inventory(path, run=run_probe):
    return {
        "path": str(path),
        "head": run(["git", "-C", str(path), "rev-parse", "HEAD"]),
        "tracked_changes": run(["git", "-C", str(path), "diff", "--name-only"]),
        "staged_changes": run(["git", "-C", str(path), "diff", "--cached", "--name-only"]),
        "untracked_files": "not_checked; provide patch/hash or a commit before cross-host validation",
    }


def collect(
    bench,
    *,
    repo_root=None,
    references=(),
    system=None,
    usb_root=Path("/sys/bus/usb/devices"),
    run=run_probe,
):
    system = system or platform.system()
    packages = {}
    for package in PACKAGES:
        try:
            packages[package] = metadata.version(
                package
            )  # Metadata only; do not import vendor modules.
        except metadata.PackageNotFoundError:
            packages[package] = None
    result = {
        "schema_version": "host-inventory/1",
        "bench": bench,
        "collected_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "inventory_only_not_validation",
        "system": system,
        "kernel": platform.release(),
        "machine": platform.machine(),
        "python": {"version": platform.python_version(), "executable": sys.executable},
        "package_metadata": packages,
        "ros2_executable": shutil.which("ros2"),
        "robot_initialization": "not_run",
        "camera_streaming": "not_run",
    }
    if repo_root is not None:
        result["project_source"] = source_inventory(Path(repo_root), run)
    result["reference_sources"] = {name: source_inventory(path, run) for name, path in references}
    if system == "Linux":
        try:
            result["os_release"] = Path("/etc/os-release").read_text()
        except OSError as exc:
            result["os_release"] = {"status": "unreadable", "error": type(exc).__name__}
        result["probes"] = {key: run(list(argv)) for key, argv in LINUX_PROBES.items()}
        result["usb_devices"] = usb_inventory(usb_root)
    else:
        result["probes"] = {key: {"status": "not_applicable"} for key in LINUX_PROBES}
        result["usb_devices"] = {"status": "not_applicable", "devices": []}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bench", required=True, help="user-defined bench/scenario label")
    parser.add_argument(
        "--repo-root", type=Path, help="project checkout; optional read-only Git queries"
    )
    parser.add_argument(
        "--reference",
        action="append",
        default=[],
        metavar="NAME=PATH",
        help="repeat for the existing, working SDK/ROS/VR checkouts",
    )
    parser.add_argument("--output", type=Path, help="new JSON file; otherwise print to stdout")
    args = parser.parse_args(argv)
    if not args.bench.strip():
        parser.error("--bench must be nonempty")
    references = []
    for item in args.reference:
        name, sep, value = item.partition("=")
        if not sep or not name or not value or any(name == n for n, _ in references):
            parser.error("--reference requires unique NAME=PATH entries")
        references.append((name, Path(value)))
    data = collect(args.bench, repo_root=args.repo_root, references=references)
    text = json.dumps(data, ensure_ascii=True, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
