"""Task-scoped episode numbers and recoverable directory commits; no devices."""

import hashlib
import json
import os
import re
import socket
import uuid
from pathlib import Path


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def task_id(task):
    if not isinstance(task, str) or not task.strip() or len(task) > 1024:
        raise ValueError("nonempty task text required (max 1024 characters)")
    return (
        task
        if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", task)
        else "task-" + hashlib.sha256(task.encode()).hexdigest()[:16]
    )


class EpisodeStore:
    def __init__(self, root, task, start_episode, existing, *, recover_lock=False):
        if type(start_episode) is not int or not 0 <= start_episode <= 999999999:
            raise ValueError("invalid start episode")
        if existing not in ("error", "skip", "overwrite"):
            raise ValueError("existing must be error/skip/overwrite")
        self.task, self.index, self.existing = task, start_episode, existing
        self.root = Path(root).expanduser().resolve() / task_id(task)
        self.root.mkdir(parents=True, exist_ok=True)
        for name in ("episodes", ".incomplete", "discarded", "backups"):
            (self.root / name).mkdir(exist_ok=True)
        self.lock = self.root / ".collector.lock"
        self.token = uuid.uuid4().hex
        if recover_lock and self.lock.exists():
            previous = json.loads(self.lock.read_text(encoding="utf-8"))
            if previous["host"] != socket.gethostname():
                raise ValueError("cannot recover a lock owned by another host")
            try:
                os.kill(previous["pid"], 0)
            except ProcessLookupError:
                self.lock.unlink()
            else:
                raise ValueError("collector lock owner is still alive")
        with self.lock.open("x", encoding="utf-8") as stream:
            json.dump(
                {"pid": os.getpid(), "host": socket.gethostname(), "token": self.token}, stream
            )
        try:
            meta = self.root / "task.json"
            if meta.exists() and json.loads(meta.read_text(encoding="utf-8"))["task"] != task:
                raise ValueError("task directory identity mismatch")
            write_json(
                meta, {"schema_version": "dataset-task/1", "task": task, "task_id": self.root.name}
            )
            self._select()
        except BaseException:
            self.close()
            raise

    def name(self):
        return f"episode_{self.index:06d}"

    def _occupied(self):
        return (self.root / "episodes" / self.name()).exists() or any(
            (self.root / ".incomplete").glob(self.name() + "_*")
        )

    def _select(self):
        if self.existing == "skip":
            while self._occupied():
                self.index += 1
        elif self.existing == "error" and self._occupied():
            raise FileExistsError("episode already exists or is incomplete: " + self.name())

    def begin(self):
        self._select()
        path = self.root / ".incomplete" / (self.name() + "_" + uuid.uuid4().hex)
        path.mkdir(exist_ok=False)
        return path

    def commit(self, path, save):
        path = Path(path).resolve()
        if path.parent != (self.root / ".incomplete").resolve() or not path.name.startswith(
            self.name() + "_"
        ):
            raise ValueError("episode transaction path mismatch")
        target = (
            self.root / ("episodes" if save else "discarded") / (self.name() if save else path.name)
        )
        backup = None
        if save and target.exists():
            if self.existing != "overwrite":
                raise FileExistsError("episode appeared before commit")
            backup = self.root / "backups" / (self.name() + "_" + uuid.uuid4().hex)
        transaction = {
            "schema_version": "episode-commit/1",
            "source": str(path),
            "target": str(target),
            "backup": str(backup) if backup else None,
        }
        write_json(path / "commit.json", transaction)
        if backup:
            os.replace(target, backup)
        try:
            os.replace(path, target)
        except BaseException:
            if backup and not target.exists():
                os.replace(backup, target)
            raise
        if save:
            self.index += 1
        # Selection happens at next begin. Never turn a successful commit into
        # an error just because the next number is already occupied.
        return target

    def close(self):
        if (
            self.lock.exists()
            and json.loads(self.lock.read_text(encoding="utf-8"))["token"] == self.token
        ):
            self.lock.unlink()
