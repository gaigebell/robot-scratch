"""Demand-driven latest-frame preview; recording has an independent queue."""

import hashlib
import math
import threading
import time
from collections import Counter


def preview_config(value=None):
    defaults = {"enabled": True, "on_demand": True, "lease_s": 2.0, "archive_jpeg": False}
    if value is None:
        return defaults
    if not isinstance(value, dict) or set(value) - set(defaults):
        raise ValueError("invalid preview_runtime config")
    c = {**defaults, **value}
    for key in ("enabled", "on_demand", "archive_jpeg"):
        if type(c[key]) is not bool:
            raise ValueError("preview_runtime." + key + " must be boolean")
    if (
        type(c["lease_s"]) not in (int, float)
        or not math.isfinite(c["lease_s"])
        or not 0.1 <= c["lease_s"] <= 60
    ):
        raise ValueError("invalid preview lease_s")
    return c


class PreviewWorker:
    def __init__(
        self, roles, encode, publish, metrics, *, config=None, archive_root=None, emit=None
    ):
        self.config = preview_config(config)
        self.roles = set(roles)
        if any(not r or "/" in r or "\\" in r or r in (".", "..") for r in self.roles):
            raise ValueError("invalid preview role")
        self.encode, self.publish, self.metrics = encode, publish, metrics
        self.archive_root, self.emit = archive_root, emit
        if self.config["archive_jpeg"] and (archive_root is None or emit is None):
            raise ValueError("preview archive requires root and event sink")
        self.condition = threading.Condition()
        self.pending = {}
        self.demand = {}
        self.counts = Counter()
        self.sequence = Counter()
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, name="preview-encoder", daemon=True)
        self.thread.start()

    def request(self, role):
        if role not in self.roles:
            return False
        with self.condition:
            self.demand[role] = time.monotonic() + self.config["lease_s"]
        return self.config["enabled"]

    def _wanted(self, role):
        return self.config["enabled"] and (
            not self.config["on_demand"] or self.demand.get(role, 0) > time.monotonic()
        )

    def wanted(self, role):
        with self.condition:
            return not self.closed and self.error is None and self._wanted(role)

    def offer(self, role, meta, raw, settings):
        with self.condition:
            if self.closed or self.error is not None or not self._wanted(role):
                return False
            if role not in self.roles:
                raise ValueError("unknown preview role")
            if role in self.pending:
                self.counts["replaced"] += 1
            self.pending[role] = (meta, raw, settings, time.monotonic_ns())
            self.condition.notify()
            return True

    def snapshot(self):
        with self.condition:
            return {
                "config": dict(self.config),
                "counts": dict(self.counts),
                "pending_roles": sorted(self.pending),
                "error": self.error,
                "worker_alive": self.thread.is_alive(),
            }

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.closed or self.pending)
                    if self.closed:
                        return
                    role = next(iter(self.pending))
                    meta, raw, settings, queued = self.pending.pop(role)
                    if not self._wanted(role):
                        continue
                self.metrics.elapsed("preview.queue." + role, queued)
                began = time.monotonic_ns()
                jpeg = self.encode(meta, raw, settings)
                self.metrics.elapsed("preview.encode." + role, began)
                self.sequence[role] += 1
                seq = self.sequence[role]
                if self.config["archive_jpeg"]:
                    began = time.monotonic_ns()
                    target = self.archive_root / "preview" / role
                    target.mkdir(parents=True, exist_ok=True)
                    path = target / (str(seq) + ".jpg")
                    path.write_bytes(jpeg)
                    self.emit(
                        "camera_preview",
                        {
                            "role": role,
                            "seq": seq,
                            "path": str(path.relative_to(self.archive_root)),
                            "device_stamp_ns": meta["stamp_ns"],
                            "received_s": meta["host_monotonic_ns"] / 1e9,
                            "frame_id": meta["frame_id"],
                            "encoding": meta["encoding"],
                            "profile": settings,
                            "sha256": hashlib.sha256(jpeg).hexdigest(),
                            "lossy": True,
                        },
                    )
                    self.metrics.elapsed("preview.archive." + role, began)
                self.publish(role, meta, jpeg, seq)
                self.metrics.elapsed(
                    "preview.receive_to_publish." + role, meta["host_monotonic_ns"]
                )
                with self.condition:
                    self.counts["encoded"] += 1
        except Exception as exc:
            with self.condition:
                self.error = str(exc)

    def close(self):
        with self.condition:
            self.closed = True
            self.pending.clear()
            self.condition.notify_all()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise TimeoutError("preview worker did not stop")
