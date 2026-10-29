"""Check the public file set without reading or printing private file contents.

The same allowlist drives staging reviews and source-distribution packaging.
This is a conservative guard, not a substitute for reviewing the Git history.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import re
import subprocess
import sys
import tomllib
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def public_paths(root: Path = ROOT) -> list[str]:
    """Expand only approved globs; never enumerate ignored datasets or environments."""
    rules = tomllib.loads((root / "publication.toml").read_text(encoding="utf-8"))["publication"]
    return sorted(
        {
            p.relative_to(root).as_posix()
            for pattern in rules["include"]
            for p in root.glob(pattern)
            if p.is_file()
        }
    )


def permitted(name: str, patterns: list[str]) -> bool:
    """Match path components, so a shallow glob cannot include hidden subtrees."""
    parts = name.split("/")
    return any(
        len(parts) == len(pattern.split("/"))
        and all(fnmatch.fnmatchcase(value, rule) for value, rule in zip(parts, pattern.split("/")))
        for pattern in patterns
    )


def read_index() -> dict[str, bytes]:
    """Inspect staged bytes, including edits which differ from the working tree."""
    raw = subprocess.check_output(["git", "ls-files", "--stage", "-z"], cwd=ROOT)
    result = {}
    for entry in raw.split(b"\0"):
        if not entry:
            continue
        meta, name = entry.split(b"\t", 1)
        mode, object_id, stage = meta.split()
        if stage != b"0" or mode not in (b"100644", b"100755"):
            raise ValueError("Unmerged, symlink or submodule entry in public index")
        result[name.decode("utf-8")] = subprocess.check_output(
            ["git", "cat-file", "blob", object_id.decode("ascii")], cwd=ROOT
        )
    return result


def check_files(files: dict[str, bytes], rules: dict) -> list[str]:
    """Return path/rule diagnostics; never echo a possible credential value."""
    errors = []
    sensitive = {
        "private key": re.compile(r"-----BEGIN (?:[A-Z0-9]+ )?PRIVATE KEY-----"),
        "access token": re.compile(
            r"\b(?:gh[pousr]_[A-Za-z0-9]{25,}|github_pat_[A-Za-z0-9_]{30,}|hf_[A-Za-z0-9]{25,})\b"
        ),
        "private LAN address": re.compile(
            r"\b(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})\b"
        ),
        "personal home path": re.compile(
            r"(?:[A-Za-z]:[\\/]Users[\\/][^\s/\\]+|(?<![A-Za-z0-9_])/"
            r"home/(?!user(?:/|\b))[^\s/]+)"
        ),
    }
    for name, raw in files.items():
        if not permitted(name, rules["include"]):
            errors.append(f"{name}: outside public allowlist")
            continue
        if len(raw) > rules["max_file_bytes"]:
            errors.append(f"{name}: exceeds public file size limit")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            errors.append(f"{name}: public source must be UTF-8 text")
            continue
        if text.startswith("\ufeff"):
            errors.append(f"{name}: unexpected UTF-8 BOM")
        if "/vendor/" not in name and b"\r\n" in raw:
            errors.append(f"{name}: first-party text must use LF")
        for reason, pattern in sensitive.items():
            if pattern.search(text):
                errors.append(f"{name}: possible {reason}; review locally")
        if name.endswith(".json"):
            try:
                json.loads(text)
            except ValueError:
                errors.append(f"{name}: invalid JSON")
        if name.endswith(".md"):
            # Plain local links must also exist in the public set, not merely on
            # the maintainer's machine. Code paths in backticks are not links.
            for target in re.findall(r"\]\(([^\s)]+)(?:\s+[^)]*)?\)", text):
                url = urlsplit(target.strip("<>"))
                if url.scheme or url.netloc or not url.path:
                    continue
                path = (ROOT / name).parent.joinpath(unquote(url.path)).resolve()
                if not path.is_relative_to(ROOT) or path.relative_to(ROOT).as_posix() not in files:
                    errors.append(f"{name}: link target is not a public file: {url.path}")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--staged",
        action="store_true",
        help="check the exact Git index rather than approved working files",
    )
    args = parser.parse_args(argv)
    rules = tomllib.loads((ROOT / "publication.toml").read_text(encoding="utf-8"))["publication"]
    files = (
        read_index()
        if args.staged
        else {name: (ROOT / name).read_bytes() for name in public_paths()}
    )
    if not files:
        print("No public files selected", file=sys.stderr)
        return 1
    errors = check_files(files, rules)
    for error in errors:
        print(error, file=sys.stderr)
    print(f"Public files: {len(files)}; issues: {len(errors)}")
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
