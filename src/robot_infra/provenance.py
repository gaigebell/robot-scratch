"""Compact source/environment identity captured once at session launch."""

import hashlib
import importlib.metadata
import os
import platform
from pathlib import Path


def runtime_identity():
    root = Path(__file__).resolve().parent
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.suffix in (".py", ".js", ".html", ".css", ".json", ".txt"):
            digest.update(
                path.relative_to(root).as_posix().encode()
                + b"\0"
                + hashlib.sha256(path.read_bytes()).digest()
            )
    versions = {}
    for name in ("mcap", "jsonschema", "Pillow", "pin", "scipy", "websockets", "av", "aiortc"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "metadata_not_available"
    return {
        "framework_tree_sha256": digest.hexdigest(),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "dependencies": versions,
    }
