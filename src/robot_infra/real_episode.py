"""Nonblocking capture -> bounded byte queue -> independent MCAP/disk worker."""

import hashlib
import json
import math
import os
import shutil
import threading
import time
from collections import Counter, deque
from copy import deepcopy
from pathlib import Path

from .episode_store import EpisodeStore, write_json
from .image_codec import resize_rgb
from .image_storage import check_image_storage, create_image_storage, storage_config


def validate_recording(c):
    if c.get("schema_version") != "real-episode-config/1":
        raise ValueError("unsupported recording config")
    for key in ("queue_bytes", "queue_items"):
        if type(c.get(key)) is not int or c[key] < 1:
            raise ValueError("positive recording queue bound required")
    for key in ("min_free_bytes", "flush_timeout_s"):
        if type(c.get(key)) not in (float, int) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("invalid recording bound: " + key)
    for key in ("min_fps_ratio", "max_unaligned_ratio"):
        if type(c["qc"].get(key)) not in (int, float) or not 0 <= c["qc"][key] <= 1:
            raise ValueError("invalid QC ratio")
    for key in ("max_frame_gap_s", "min_duration_s"):
        if (
            type(c["qc"].get(key)) not in (int, float)
            or not math.isfinite(c["qc"][key])
            or c["qc"][key] <= 0
        ):
            raise ValueError("invalid QC timing")
    if not isinstance(c["alignment"].get("reference_stream"), str) or not c["alignment"][
        "reference_stream"
    ].startswith("camera/"):
        raise ValueError("explicit alignment reference required")
    delta = c["alignment"].get("max_delta_s")
    if type(delta) not in (int, float) or not math.isfinite(delta) or delta <= 0:
        raise ValueError("invalid alignment tolerance")
    if not isinstance(c.get("required_events"), list) or any(
        not isinstance(k, str) or not k for k in c["required_events"]
    ):
        raise ValueError("explicit required event kinds required")
    storage_config(c.get("storage"))
    return c


class RateGate:
    def __init__(self, fps):
        self.period = 1e9 / fps
        self.next = None

    def accept(self, stamp):
        if self.next is not None and stamp + self.period * 0.1 < self.next:
            return False
        self.next = max(
            stamp, self.next + self.period if self.next is not None else stamp + self.period
        )
        return True


class HashStream:
    def __init__(self, path):
        self.file = Path(path).open("xb")
        self.digest = hashlib.sha256()

    def write(self, data):
        self.digest.update(data)
        return self.file.write(data)

    def tell(self):
        return self.file.tell()

    def flush(self):
        self.file.flush()

    def close(self):
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()


class EpisodeWriter:
    def __init__(self, path, manifest):
        from mcap.writer import CompressionType, Writer

        self.path, self.manifest = Path(path), manifest
        self.storage = storage_config(manifest["recording"].get("storage"))
        expected_version = (
            "real-episode/2" if self.storage["image_backend"] == "mp4" else "real-episode/1"
        )
        if manifest.get("schema_version") != expected_version:
            raise ValueError("episode version does not match image storage backend")
        self.images = create_image_storage(self.path, self.storage)
        self.stream = HashStream(self.path / "streams.mcap")
        self.writer = Writer(
            self.stream,
            compression=CompressionType.NONE,
            chunk_size=4 * 1024 * 1024,
            enable_data_crcs=True,
        )
        self.writer.start(
            profile="robot-infra-" + manifest.get("schema_version", "real-episode/1"),
            library="robot-infra",
        )
        self.schemas = {
            "json": self.writer.register_schema(
                "robot_infra.RealEvent/1", "jsonschema", b'{"type":"object"}'
            ),
            "image": self.writer.register_schema(
                "robot_infra.RawImage/1",
                "robot-infra-envelope/1",
                b"uint32_le json_length; UTF8 JSON metadata; image bytes including row padding",
            ),
        }
        self.schemas["video"] = self.writer.register_schema(
            "robot_infra.VideoFrame/1", "jsonschema", b'{"type":"object"}'
        )
        self.channels = {}
        self.count = 0
        self.counts = Counter()
        self.gates = {}
        self.frame_capture_stats = {}
        self.index = (self.path / "index.jsonl").open("x", encoding="utf-8")
        self.write_event("episode_manifest", manifest, time.monotonic_ns(), time.time_ns())

    def _write(self, name, body, typ, mono, wall, source=None, extra=None):
        if name not in self.channels:
            self.channels[name] = self.writer.register_channel(
                "/" + name,
                "robot-infra-raw-image/1" if typ == "image" else "json",
                self.schemas[typ],
            )
        sequence = self.count
        self.writer.add_message(
            self.channels[name],
            wall,
            body,
            publish_time=source if source is not None and source >= 0 else wall,
            sequence=sequence,
        )
        self.count += 1
        self.counts[name] += 1
        if name.startswith(("camera/", "feedback/", "pose_feedback/")):
            self.index.write(
                json.dumps(
                    {
                        "stream": name,
                        "sequence": sequence,
                        "received_ns": mono,
                        "source_stamp_ns": source,
                        **(extra or {}),
                    }
                )
                + "\n"
            )

    def write_event(self, kind, payload, mono, wall):
        name = "feedback/" + payload["side"] if kind == "feedback" else "event/" + kind
        source = payload.get("device_stamp_ns") if kind == "feedback" else None
        extra = None
        if kind == "pose_feedback":
            name = "pose_feedback/" + payload["source_id"]
            source = payload["source_stamp_ns"]
            extra = {k: payload[k] for k in ("quaternion_norm", "quaternion_unit") if k in payload}
        body = json.dumps(
            {
                "schema_version": "real-episode-event/1",
                "host_monotonic_ns": mono,
                "host_unix_ns": wall,
                "kind": kind,
                "payload": payload,
            },
            allow_nan=False,
            separators=(",", ":"),
        ).encode()
        self._write(name, body, "json", mono, wall, source, extra)

    def frame(self, role, stream, meta, raw):
        settings = self.manifest["camera_pipeline"][role]["record"].get(stream)
        if not settings:
            return
        name = "camera/" + role + "/" + stream
        capture = self.manifest["camera_pipeline"][role]["capture"][stream]
        stats = self.frame_capture_stats.setdefault(
            name, {"received_by_writer": 0, "rate_filtered": 0, "written": 0}
        )
        stats["received_by_writer"] += 1
        # Native-rate recording preserves every received frame, including
        # delivery bursts. A receive-time limiter would discard real frames.
        if settings["fps"] < capture["fps"] and not self.gates.setdefault(
            name, RateGate(settings["fps"])
        ).accept(meta["host_monotonic_ns"]):
            stats["rate_filtered"] += 1
            return
        match = all(meta[k] == capture[k] for k in ("width", "height"))
        if stream == "rgb":
            meta, raw = resize_rgb(meta, raw, settings["width"], settings["height"])
        frame = self.images.frame(name, meta, raw, settings)
        self._write(
            name,
            frame.body,
            frame.kind,
            meta["host_monotonic_ns"],
            meta["host_unix_ns"],
            meta["stamp_ns"],
            {"profile_match": match, **frame.index},
        )
        stats["written"] += 1

    def finish(self, updates):
        self.manifest.update(updates)
        self.manifest["frame_capture_stats"] = self.frame_capture_stats
        assets = self.images.finish()
        if self.storage["image_backend"] == "mp4":
            self.manifest["media"] = assets
        self.write_event("episode_finalize", updates, time.monotonic_ns(), time.time_ns())
        self.writer.finish()
        self.stream.close()
        self.index.flush()
        os.fsync(self.index.fileno())
        self.index.close()
        self.manifest.update(
            mcap_sha256=self.stream.digest.hexdigest(),
            mcap_messages=self.count,
            counts=dict(self.counts),
            status="review",
        )
        self.manifest["index_sha256"] = hashlib.sha256(
            (self.path / "index.jsonl").read_bytes()
        ).hexdigest()
        write_json(self.path / "manifest.json", self.manifest)
        from .episode_qc import quality_report

        qc = quality_report(self.path, self.manifest)
        self.manifest["qc_passed"] = qc["passed"]
        write_json(self.path / "manifest.json", self.manifest)
        return qc

    def abort(self):
        try:
            self.images.abort()
            self.writer.flush()
        finally:
            try:
                self.stream.close()
            finally:
                self.index.close()


# Compatibility for callers/tests using the previous raw-only writer name.
RawEpisodeWriter = EpisodeWriter


class EpisodeRecorder:
    """Queue capture/lifecycle requests for a single episode disk worker.

    Construction reserves the task store and checks the encoder. After that,
    capture callbacks enqueue bounded immutable snapshots; the worker owns
    episode files, encoding, queue drain, QC and the final save/discard commit.
    """

    def __init__(
        self,
        config,
        root,
        task,
        start_episode,
        existing,
        *,
        recover_lock=False,
        writer_factory=EpisodeWriter,
        metrics=None,
    ):
        self.metrics = metrics
        self.config = validate_recording(deepcopy(config))
        selected = check_image_storage(self.config.get("storage"))
        if selected["image_backend"] == "mp4":
            self.config["storage"] = selected
        self.store = EpisodeStore(root, task, start_episode, existing, recover_lock=recover_lock)
        self.condition = threading.Condition(threading.RLock())
        self.queue = deque()
        self.queued_bytes = 0
        self.ops = deque()
        self.state = "IDLE"
        self.error = None
        self.writer = None
        self.path = None
        self.qc = None
        self.saved = 0
        self.discarded = 0
        self.dropped = 0
        self.capture_sequence = 0
        self.closing = False
        self.stop_reason = None
        self.factory = writer_factory
        self.last_commit = None
        self.capture_end_ns = None
        self.thread = threading.Thread(target=self._run, name="episode-disk-writer", daemon=True)
        self.thread.start()

    def start(self, context):
        with self.condition:
            if self.state != "IDLE":
                raise RuntimeError("recorder is not idle")
            self.state = "STARTING"
            self.error = None
            self.qc = None
            self.dropped = 0
            self.stop_reason = None
            self.capture_end_ns = None
            self.ops.append(("start", deepcopy(context)))
            self.condition.notify()

    def _enqueue(self, value, size):
        with self.condition:
            if self.state != "RECORDING":
                return False
            if (
                size + self.queued_bytes > self.config["queue_bytes"]
                or len(self.queue) >= self.config["queue_items"]
            ):
                self.dropped += 1
                self.error = "recording_queue_overflow"
                self.state = "FINALIZING"
                self.stop_reason = self.error
                self.capture_end_ns = time.monotonic_ns()
                self.condition.notify()
                return False
            self.queue.append((value, size, time.monotonic_ns()))
            self.queued_bytes += size
            self.capture_sequence += 1
            self.condition.notify()
            return True

    def event(self, kind, payload, mono=None, wall=None):
        if self.state != "RECORDING":
            return False
        value = (
            "event",
            kind,
            deepcopy(payload),
            mono or time.monotonic_ns(),
            wall or time.time_ns(),
        )
        return self._enqueue(value, len(json.dumps(value, allow_nan=False)) + 128)

    def frame(self, role, stream, meta, raw):
        if self.state != "RECORDING":
            return False
        return self._enqueue(("frame", role, stream, dict(meta), raw), len(raw) + 1024)

    def finish(self, interrupted=None):
        """Close the capture window; let the worker drain before entering REVIEW."""
        with self.condition:
            if self.state == "RECORDING":
                self.state = "FINALIZING"
                self.stop_reason = interrupted
                self.capture_end_ns = time.monotonic_ns()
                self.condition.notify()
            elif self.state == "STARTING":
                self.stop_reason = interrupted or "cancelled_before_recording"

    def decide(self, save):
        with self.condition:
            if self.state != "REVIEW":
                raise RuntimeError("episode is not ready for review")
            self.state = "SAVING" if save else "DISCARDING"
            self.ops.append(("commit", bool(save)))
            self.condition.notify()

    def snapshot(self):
        with self.condition:
            return {
                "state": self.state,
                "error": self.error,
                "task": self.store.task,
                "task_path": str(self.store.root),
                "episode": self.store.index,
                "image_storage": storage_config(self.config.get("storage"))["image_backend"],
                "saved_episodes": self.saved,
                "discarded_episodes": self.discarded,
                "queue_bytes": self.queued_bytes,
                "queue_items": len(self.queue),
                "dropped": self.dropped,
                "qc": deepcopy(self.qc),
                "last_commit": self.last_commit,
                "pending_path": str(self.path) if self.path else None,
            }

    def _begin(self, context):
        if shutil.disk_usage(self.store.root).free < self.config["min_free_bytes"]:
            raise OSError("insufficient disk space")
        storage = check_image_storage(self.config.get("storage"), context["camera_pipeline"])
        path = self.store.begin()
        self.path = path
        pipeline = context["camera_pipeline"]
        expected = {}
        for role, p in pipeline.items():
            for stream, settings in p["record"].items():
                if settings:
                    expected["camera/" + role + "/" + stream] = settings
        for side in context["arms"]:
            expected["feedback/" + side] = {}
        from .pose_feedback import validate_pose_sources

        for name in validate_pose_sources(context.get("pose_feedback")):
            expected["pose_feedback/" + name] = {}
        if self.config["alignment"]["reference_stream"] not in expected:
            raise ValueError("alignment reference is not a recorded stream")
        manifest = {
            "schema_version": "real-episode/2"
            if storage["image_backend"] == "mp4"
            else "real-episode/1",
            "task": self.store.task,
            "episode": self.store.index,
            "status": "recording",
            "start_monotonic_ns": time.monotonic_ns(),
            "start_unix_ns": time.time_ns(),
            "recording": self.config,
            "camera_pipeline": pipeline,
            "expected_streams": expected,
            "context": context,
        }
        write_json(path / "manifest.json", manifest)
        self.writer = self.factory(path, manifest)
        manifest["start_monotonic_ns"] = time.monotonic_ns()
        manifest["start_unix_ns"] = time.time_ns()
        self.writer.write_event(
            "recording_started",
            {"start_monotonic_ns": manifest["start_monotonic_ns"]},
            manifest["start_monotonic_ns"],
            manifest["start_unix_ns"],
        )
        with self.condition:
            self.state = "FINALIZING" if self.stop_reason else "RECORDING"

    def _run(self):
        last_disk_check = 0.0
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.ops
                        or self.queue
                        or self.state == "FINALIZING"
                        or self.closing,
                        timeout=0.2,
                    )
                    if self.ops:
                        op, value = self.ops.popleft()
                        item = None
                    elif self.queue:
                        item, size, queued_ns = self.queue.popleft()
                        self.queued_bytes -= size
                        op = None
                    elif self.state == "FINALIZING":
                        op, value = "finish", None
                        item = None
                    elif self.closing:
                        break
                    else:
                        continue
                try:
                    if op == "start":
                        self._begin(value)
                    elif op == "finish":
                        end = max(
                            self.writer.manifest["start_monotonic_ns"],
                            self.capture_end_ns or time.monotonic_ns(),
                        )
                        self.qc = self.writer.finish(
                            {
                                "end_monotonic_ns": end,
                                "interrupted": self.stop_reason,
                                "dropped": self.dropped,
                                "writer_error": self.error,
                            }
                        )
                        self.writer = None
                        with self.condition:
                            self.state = "REVIEW"
                    elif op == "commit":
                        manifest = json.loads(
                            (self.path / "manifest.json").read_text(encoding="utf-8")
                        )
                        manifest.update(status="saved" if value else "discarded")
                        write_json(self.path / "manifest.json", manifest)
                        self.last_commit = str(self.store.commit(self.path, value))
                        self.saved += value
                        self.discarded += not value
                        self.path = None
                        with self.condition:
                            self.state = "IDLE"
                    elif item:
                        category = "event" if item[0] == "event" else item[1] + "." + item[2]
                        if self.metrics:
                            self.metrics.elapsed("record.queue." + category, queued_ns)
                        began = time.monotonic_ns()
                        if item[0] == "event":
                            self.writer.write_event(*item[1:])
                        else:
                            self.writer.frame(*item[1:])
                        if self.metrics:
                            self.metrics.elapsed("record.write." + category, began)
                        if time.monotonic() - last_disk_check > 1:
                            last_disk_check = time.monotonic()
                            if (
                                shutil.disk_usage(self.store.root).free
                                < self.config["min_free_bytes"]
                            ):
                                raise OSError("disk reserve reached")
                except Exception as exc:
                    with self.condition:
                        self.error = str(exc)
                        self.state = "ERROR"
                        self.queue.clear()
                        self.queued_bytes = 0
                        self.ops.clear()
                    if self.writer:
                        try:
                            self.writer.abort()
                        except Exception:
                            pass
                        self.writer = None
                    if self.path:
                        try:
                            write_json(
                                self.path / "failure.json",
                                {"error": self.error, "incomplete": True},
                            )
                        except Exception:
                            pass
        finally:
            self.store.close()

    def close(self):
        with self.condition:
            if self.state in ("STARTING", "RECORDING"):
                self.finish("process_exit")
            self.closing = True
            self.condition.notify()
        self.thread.join(timeout=self.config["flush_timeout_s"])
        if self.thread.is_alive():
            raise TimeoutError(
                "episode writer still draining; incomplete data retained and task lock stays held"
            )
