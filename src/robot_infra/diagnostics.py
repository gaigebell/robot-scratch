"""Append-only segmented diagnostics; never discards old segments."""

import hashlib
import json
import queue
import shutil
import threading
import time
from collections import Counter
from pathlib import Path


def diagnostic_config(value=None):
    value = {} if value is None else value
    defaults = {
        "segment_bytes": 128 * 1024 * 1024,
        "max_bytes": None,
        "min_free_bytes": 1024 * 1024 * 1024,
    }
    if not isinstance(value, dict) or set(value) - set(defaults):
        raise ValueError("invalid diagnostics config")
    c = {**defaults, **value}
    for k in ("segment_bytes", "min_free_bytes"):
        if type(c[k]) is not int or c[k] < 1:
            raise ValueError("positive diagnostic " + k + " required")
    if c["max_bytes"] is not None and (type(c["max_bytes"]) is not int or c["max_bytes"] < 1):
        raise ValueError("diagnostic max_bytes must be null or positive")
    return c


def journal_segments(root):
    root = Path(root)
    return [root / "events.jsonl", *sorted(root.glob("events.[0-9]*.jsonl"))]


class EventJournal:
    def __init__(
        self,
        root,
        max_bytes=None,
        segment_bytes=128 * 1024 * 1024,
        min_free_bytes=1024 * 1024 * 1024,
    ):
        settings = diagnostic_config(
            {
                "max_bytes": max_bytes,
                "segment_bytes": segment_bytes,
                "min_free_bytes": min_free_bytes,
            }
        )
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.queue = queue.Queue(8192)
        self.error = None
        self.counts = Counter()
        self.bytes = 0
        self.max_bytes = settings["max_bytes"]
        self.segment_limit = settings["segment_bytes"]
        self.min_free_bytes = settings["min_free_bytes"]
        self.segments = []
        self.segment_bytes = 0
        self.digest = hashlib.sha256()
        self.file = (self.root / "events.jsonl").open("x", encoding="utf-8", newline="\n")
        self.thread = threading.Thread(target=self._write, name="diagnostic-writer", daemon=True)
        self.thread.start()

    def emit(self, kind, payload):
        if self.error:
            raise RuntimeError("journal failed: " + self.error)
        event = {
            "schema_version": "live-teleop-event/1",
            "host_monotonic_ns": time.monotonic_ns(),
            "host_unix_ns": time.time_ns(),
            "kind": kind,
            "payload": payload,
        }
        try:
            self.queue.put_nowait(event)
        except queue.Full:
            self.error = "queue_full"
            raise RuntimeError("journal queue full")
        return event

    def _seal(self):
        import os

        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        self.segments.append(
            {
                "path": Path(self.file.name).name,
                "bytes": self.segment_bytes,
                "sha256": self.digest.hexdigest(),
            }
        )

    def _write(self):
        checked = 0.0
        try:
            while True:
                event = self.queue.get()
                if event is None:
                    break
                line = json.dumps(event, allow_nan=False, ensure_ascii=True) + "\n"
                body = line.encode()
                if self.max_bytes is not None and self.bytes + len(body) > self.max_bytes:
                    raise RuntimeError("journal capacity reached")
                if time.monotonic() - checked > 1:
                    checked = time.monotonic()
                    if shutil.disk_usage(self.root).free < self.min_free_bytes:
                        raise RuntimeError("diagnostic disk reserve reached")
                if self.segment_bytes and self.segment_bytes + len(body) > self.segment_limit:
                    self._seal()
                    self.segment_bytes = 0
                    self.digest = hashlib.sha256()
                    self.file = (self.root / f"events.{len(self.segments):06d}.jsonl").open(
                        "x", encoding="utf-8", newline="\n"
                    )
                self.file.write(line)
                self.digest.update(body)
                self.segment_bytes += len(body)
                self.bytes += len(body)
                self.counts[event["kind"]] += 1
                if self.queue.empty():
                    self.file.flush()
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        if self.thread.is_alive():
            self.queue.put(None, timeout=2)
            self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise TimeoutError("diagnostic writer still draining")
        if not self.file.closed:
            self._seal()
        (self.root / "journal-summary.json").write_text(
            json.dumps(
                {
                    "counts": self.counts,
                    "bytes": self.bytes,
                    "error": self.error,
                    "writer_stopped": True,
                    "segments": self.segments,
                    "retention": "all segments preserved; no silent deletion",
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
