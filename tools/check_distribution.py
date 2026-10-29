"""Validate built archives against the public source list and runtime resources."""

from __future__ import annotations

import argparse
import tarfile
import zipfile
from pathlib import Path

from check_publication import ROOT, public_paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist-dir", type=Path, default=ROOT / "dist")
    args = parser.parse_args()
    approved = set(public_paths())
    archives = sorted(args.dist_dir.glob("robot_scratch-*"))
    if not archives:
        raise ValueError("Build robot-scratch before checking distributions")
    for archive in archives:
        if archive.suffix == ".whl":
            with zipfile.ZipFile(archive) as wheel:
                names = wheel.namelist()
                unexpected = [
                    name
                    for name in names
                    if not name.endswith("/")
                    and not name.startswith("robot_scratch-")
                    and "src/" + name not in approved
                ]
                resources = {
                    "robot_infra/web/xr-session.html",
                    "robot_infra/web/xr-transport.js",
                    "robot_infra/web/vendor/THREE-LICENSE.txt",
                    "robot_infra/schemas/graph.json",
                }
                if not resources.issubset(names):
                    raise ValueError("Missing runtime resources: " + archive.name)
        elif archive.name.endswith(".tar.gz"):
            with tarfile.open(archive) as source:
                names = [
                    member.name.split("/", 1)[1]
                    for member in source.getmembers()
                    if member.isfile() and "/" in member.name
                ]
                unexpected = [
                    name
                    for name in names
                    if name not in approved
                    and name not in ("PKG-INFO", "setup.cfg")
                    and not name.startswith("src/robot_scratch.egg-info/")
                ]
        else:
            continue
        if unexpected:
            raise ValueError(f"{archive.name}: unapproved paths: {unexpected}")
        print(f"{archive.name}: {len(names)} public/package files; passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
