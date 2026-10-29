"""Streaming MCAP for collection-event/1; validation never constructs devices."""

import hashlib
from collections import Counter
from pathlib import Path

from jsonschema import Draft202012Validator
from mcap.reader import NonSeekingReader
from mcap.writer import CompressionType, Writer

from .collection import EVENT_VERSION, validate_collection
from .contracts import canonical
from .graph import strict_json

PROFILE = "robot-infra-collection-fake/1"
SCHEMA_NAME = "robot_infra.CollectionEvent/1"
KINDS = [
    "manifest",
    "lifecycle",
    "ownership",
    "raw_source",
    "intent",
    "motion_result",
    "guard",
    "command",
    "receipt",
    "feedback",
    "state",
    "camera",
    "snapshot",
    "finalize",
]
SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": [
        "schema_version",
        "session_id",
        "event_id",
        "kind",
        "time_ns",
        "clock_domain",
        "epoch",
        "payload",
    ],
    "properties": {
        "schema_version": {"const": EVENT_VERSION},
        "session_id": {"type": "string", "minLength": 1},
        "event_id": {"type": "integer", "minimum": 0},
        "kind": {"enum": KINDS},
        "time_ns": {"type": "integer", "minimum": 0},
        "clock_domain": {"const": "fake_monotonic"},
        "epoch": {"type": "integer", "minimum": 0},
        "payload": {"type": "object"},
    },
}
VALIDATOR = Draft202012Validator(SCHEMA)


class CollectionRecorder:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("xb")
        self.writer = Writer(
            self.stream,
            compression=CompressionType.NONE,
            chunk_size=1024 * 1024,
            enable_data_crcs=True,
        )
        self.writer.start(profile=PROFILE, library="robot-infra")
        self.schema = self.writer.register_schema(SCHEMA_NAME, "jsonschema", canonical(SCHEMA))
        self.channels, self.counts, self.hasher = {}, Counter(), hashlib.sha256()
        self.count, self.last, self.finished = 0, None, False

    def write(self, event):
        if self.finished:
            raise ValueError("recording finalized")
        VALIDATOR.validate(event)
        if event["event_id"] != self.count:
            raise ValueError("event sequence mismatch")
        if self.last and (
            event["session_id"] != self.last["session_id"]
            or event["time_ns"] < self.last["time_ns"]
            or event["epoch"] < self.last["epoch"]
        ):
            raise ValueError("mixed session/non-monotonic event")
        kind = event["kind"]
        if kind not in self.channels:
            self.channels[kind] = self.writer.register_channel(
                "/collection/" + kind, "json", self.schema
            )
        body = canonical(event)
        self.writer.add_message(
            self.channels[kind],
            event["time_ns"],
            body,
            publish_time=event["time_ns"],
            sequence=event["event_id"],
        )
        if kind != "finalize":
            self.hasher.update(body + b"\n")
        self.count += 1
        self.counts[kind] += 1
        self.last = event
        self.finished = kind == "finalize"

    def finish(self, result):
        if not self.last or self.finished or result["events_before_finalize"] != self.count:
            raise ValueError("finalize count/lifecycle mismatch")
        self.write(
            {
                **self.last,
                "event_id": self.count,
                "kind": "finalize",
                "payload": {**result, "events_hash": self.hasher.hexdigest()},
            }
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            if exc_type is None and self.finished:
                self.writer.finish()
            else:
                self.writer.flush()  # Preserve evidence, no normal footer/complete claim.
        finally:
            self.stream.close()


def read_collection(path):
    """Validate CRC/schema/hash/coverage and batch associations with bounded state."""
    count, counts, hasher = 0, Counter(), hashlib.sha256()
    manifest = last = final = guard = pending = awaiting_feedback = None
    states, cameras, backend_calls = set(), set(), Counter()
    had_failure = False
    try:
        with Path(path).open("rb") as stream:
            if stream.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("not MCAP")
            stream.seek(-8, 2)
            if stream.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("incomplete MCAP footer")
            stream.seek(0)
            reader = NonSeekingReader(stream, validate_crcs=True, record_size_limit=4 * 1024 * 1024)
            if reader.get_header().profile != PROFILE:
                raise ValueError("unsupported collection profile")
            stream.seek(0)
            reader = NonSeekingReader(stream, validate_crcs=True, record_size_limit=4 * 1024 * 1024)
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                if (
                    schema is None
                    or schema.name != SCHEMA_NAME
                    or schema.encoding != "jsonschema"
                    or strict_json(schema.data.decode()) != SCHEMA
                    or channel.message_encoding != "json"
                ):
                    raise ValueError("event schema/encoding mismatch")
                event = strict_json(message.data.decode())
                VALIDATOR.validate(event)
                if (
                    final
                    or event["event_id"] != count
                    or message.sequence != count
                    or message.log_time != event["time_ns"]
                    or message.publish_time != event["time_ns"]
                    or channel.topic != "/collection/" + event["kind"]
                ):
                    raise ValueError("event order/topic/time/finalize mismatch")
                if last and (
                    event["session_id"] != last["session_id"]
                    or event["time_ns"] < last["time_ns"]
                    or event["epoch"] < last["epoch"]
                ):
                    raise ValueError("mixed session/non-monotonic event")
                kind, p = event["kind"], event["payload"]
                if kind == "lifecycle" and p.get("state") == "faulted":
                    had_failure = True
                if count == 0:
                    if kind != "manifest":
                        raise ValueError("manifest must be first")
                    manifest = validate_collection(p["config"])
                    if p["config_hash"] != manifest["config_hash"]:
                        raise ValueError("manifest hash mismatch")
                elif kind == "manifest":
                    raise ValueError("duplicate manifest")
                if kind == "state":
                    states.add(p["backend"])
                if kind == "camera":
                    cameras.add(p["role"])
                if kind == "guard":
                    guard = event if p["decision"] == "accepted" else None
                if kind == "command":
                    if (
                        not guard
                        or p["guard_event_id"] != guard["event_id"]
                        or event["epoch"] != guard["epoch"]
                        or pending
                        or awaiting_feedback
                    ):
                        raise ValueError("command lacks guard or prior receipt/feedback")
                    expected = {
                        t["endpoint_id"]: t["positions"]
                        for t in guard["payload"]["intent"]["targets"]
                    }
                    members = manifest["batches"][p["backend"]]
                    if p["targets"] != {e: expected[e] for e in members}:
                        raise ValueError("backend batch differs from approved command set")
                    pending = event
                if kind == "receipt":
                    if (
                        pending is None
                        or p["command_event_id"] != pending["event_id"]
                        or p["backend"] != pending["payload"]["backend"]
                    ):
                        raise ValueError("receipt/command association mismatch")
                    if p["status"] == "executed_fake":
                        backend_calls[p["backend"]] += 1
                        awaiting_feedback = (
                            event["event_id"],
                            p["backend"],
                            pending["payload"]["targets"],
                        )
                    elif p["status"] == "failed_execution_unknown":
                        had_failure = True
                    else:
                        raise ValueError("unsupported fake receipt")
                    pending = None
                if kind == "feedback":
                    if (
                        awaiting_feedback is None
                        or (p["receipt_event_id"], p["backend"], p["positions"])
                        != awaiting_feedback
                    ):
                        raise ValueError("feedback/receipt association mismatch")
                    awaiting_feedback = None
                if kind == "finalize":
                    if (
                        p["events_before_finalize"] != count
                        or p["events_hash"] != hasher.hexdigest()
                        or p["outcome"] not in ("complete", "aborted")
                    ):
                        raise ValueError("final hash/count/outcome mismatch")
                    if (
                        p["counts"] != dict(counts)
                        or p["backend_calls"] != {b: backend_calls[b] for b in manifest["batches"]}
                        or pending
                        or awaiting_feedback
                    ):
                        raise ValueError("incomplete batch evidence/counts")
                    if had_failure and p["outcome"] == "complete":
                        raise ValueError("failed episode mislabeled complete")
                    final = p
                else:
                    hasher.update(canonical(event) + b"\n")
                counts[kind] += 1
                count += 1
                last = event
        if final is None:
            raise ValueError("missing finalize")
        missing = {
            "backends": sorted(set(manifest["batches"]) - states),
            "cameras": sorted(set(manifest["config"]["cameras"]) - cameras),
        }
        if final["outcome"] == "complete" and any(missing.values()):
            raise ValueError("complete episode lacks required observations")
        return {
            "schema_version": EVENT_VERSION,
            "mode": "fake",
            "session_id": last["session_id"],
            "events": count,
            "counts": dict(counts),
            "outcome": final["outcome"],
            "missing_streams": missing,
            "backend_calls": final["backend_calls"],
            "events_hash": final["events_hash"],
        }
    except Exception as exc:
        raise ValueError(f"invalid collection recording: {exc}") from exc
