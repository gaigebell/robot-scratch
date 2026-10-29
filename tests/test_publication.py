"""Public releases must not depend on a maintainer's private working files."""

import importlib.util
import subprocess
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "publication", Path(__file__).parents[1] / "tools/check_publication.py"
)
publication = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(publication)


def test_shallow_allowlist_does_not_admit_nested_private_files():
    assert publication.permitted("src/module.py", ["src/*.py"])
    assert not publication.permitted("src/private/secret.py", ["src/*.py"])


def test_links_must_exist_in_public_set_even_if_private_target_exists(tmp_path, monkeypatch):
    monkeypatch.setattr(publication, "ROOT", tmp_path)
    (tmp_path / "internal.md").write_text("private", encoding="utf-8")
    files = {"README.md": b"[local analysis](internal.md)"}
    errors = publication.check_files(files, {"include": ["README.md"], "max_file_bytes": 1000})
    assert any("not a public file" in error for error in errors)


@pytest.mark.parametrize(
    "value,reason",
    [
        ("192" + ".168.10.2", "private LAN address"),
        ("/" + "home/" + "operator/private", "personal home path"),
        ("-----BEGIN " + "PRIVATE KEY-----", "private key"),
        ("ghp_" + "x" * 30, "access token"),
    ],
)
def test_sensitive_values_are_reported_without_echoing_them(value, reason):
    errors = publication.check_files(
        {"sample.txt": value.encode()}, {"include": ["sample.txt"], "max_file_bytes": 1000}
    )
    assert any(reason in error for error in errors)
    assert not any(value in error for error in errors)


def test_staged_bytes_are_checked_instead_of_later_worktree_edits(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "--quiet", str(tmp_path)], check=True)
    path = tmp_path / "README.md"
    staged = b"staged content\n"
    path.write_bytes(staged)
    subprocess.run(["git", "add", "README.md"], cwd=tmp_path, check=True)
    path.write_text("different unstaged content\n", encoding="utf-8")
    monkeypatch.setattr(publication, "ROOT", tmp_path)
    assert publication.read_index() == {"README.md": staged}
