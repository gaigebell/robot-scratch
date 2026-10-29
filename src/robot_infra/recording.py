"""MCAP event evidence and read-only replay. This module imports no devices."""

import json
from collections import Counter
from importlib.resources import files
from pathlib import Path

from jsonschema import Draft202012Validator
from mcap.reader import NonSeekingReader
from mcap.writer import CompressionType, Writer

from . import CONTRACT_VERSION
from .contracts import canonical, digest
from .graph import strict_json, validate_graph

SCHEMA_NAME = f"robot_infra.Event/{CONTRACT_VERSION}"
PROFILE = "robot-infra-m1"
MAGIC = b"\x89MCAP0\r\n"


class RecordingError(ValueError):
    pass


class Recorder:
    """Create exclusively; leave an explicitly incomplete file on failures."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("xb")
        self._writer = Writer(self._stream, compression=CompressionType.NONE, enable_data_crcs=True)
        self._writer.start(profile=PROFILE, library="robot-infra/0.1.0")
        self._schema_data = files("robot_infra").joinpath("schemas/event.json").read_bytes()
        self._schema = self._writer.register_schema(SCHEMA_NAME, "jsonschema", self._schema_data)
        self._channels = {}
        self._finalized = False

    def write(self, event):
        if self._finalized:
            raise RecordingError("cannot write after finalize")
        kind = event["kind"]
        if kind not in self._channels:
            self._channels[kind] = self._writer.register_channel(
                f"/events/{kind}", "json", self._schema
            )
        self._writer.add_message(
            self._channels[kind],
            event["time_ns"],
            canonical(event),
            publish_time=event["time_ns"],
            sequence=event["event_id"],
        )
        self._finalized = kind == "finalize"

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            if exc_type is None and self._finalized:
                self._writer.finish()
            # Flush buffered evidence even on an interrupted run, without claiming finalization.
            elif not self._stream.closed:
                self._writer.flush()
        finally:
            self._stream.close()


def replay(path):
    """Validate the complete file before returning any events to consumers."""
    expected_schema = json.loads(files("robot_infra").joinpath("schemas/event.json").read_text())
    validator = Draft202012Validator(expected_schema)
    events = []
    try:
        with Path(path).open("rb") as stream:
            if stream.read(8) != MAGIC:
                raise RecordingError("not an MCAP file")
            stream.seek(-8, 2)
            if stream.read(8) != MAGIC:
                raise RecordingError("incomplete MCAP footer")
            stream.seek(0)
            reader = NonSeekingReader(
                stream, validate_crcs=True, record_size_limit=16 * 1024 * 1024
            )
            if reader.get_header().profile != PROFILE:
                raise RecordingError("unsupported MCAP profile")
            stream.seek(0)
            reader = NonSeekingReader(
                stream, validate_crcs=True, record_size_limit=16 * 1024 * 1024
            )
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                if (
                    schema is None
                    or schema.name != SCHEMA_NAME
                    or schema.encoding != "jsonschema"
                    or strict_json(schema.data.decode()) != expected_schema
                    or channel.message_encoding != "json"
                ):
                    raise RecordingError("unsupported event schema/encoding")
                event = strict_json(message.data.decode())
                error = next(validator.iter_errors(event), None)
                if error:
                    raise RecordingError(f"event schema: {error.message}")
                if (
                    event["event_id"] != len(events)
                    or message.sequence != event["event_id"]
                    or channel.topic != f"/events/{event['kind']}"
                    or message.log_time != event["time_ns"]
                    or message.publish_time != event["time_ns"]
                ):
                    raise RecordingError("event order/topic/timestamp mismatch")
                if events and (
                    event["session_id"] != events[0]["session_id"]
                    or event["time_ns"] < events[-1]["time_ns"]
                    or event["epoch"] < events[-1]["epoch"]
                ):
                    raise RecordingError("mixed session, non-monotonic time or epoch")
                events.append(event)
        _validate_timeline(events)
        return events
    except RecordingError:
        raise
    except Exception as exc:
        raise RecordingError(f"invalid/incomplete recording: {type(exc).__name__}: {exc}") from exc


def _validate_timeline(events):
    if not events or events[0]["kind"] != "manifest" or events[-1]["kind"] != "finalize":
        raise RecordingError("missing manifest/finalize")
    if any(e["kind"] in ("manifest", "finalize") for e in events[1:-1]):
        raise RecordingError("duplicate manifest/finalize")
    manifest, final = events[0]["payload"], events[-1]["payload"]
    compiled = validate_graph(manifest["config"])
    if manifest["config_hash"] != compiled["config_hash"] or manifest["asset_hash"] != digest(
        manifest["config"]["asset"]
    ):
        raise RecordingError("manifest hash mismatch")
    if final["event_count"] != len(events) - 1 or final["events_hash"] != digest(events[:-1]):
        raise RecordingError("event count/hash mismatch")
    if final["outcome"] not in ("complete", "aborted"):
        raise RecordingError("unknown outcome")
    commands, receipts, feedback = {}, {}, set()
    for event in events:
        p = event["payload"]
        if event["kind"] == "command":
            guard_id = p["guard_event_id"]
            if (
                type(guard_id) is not int
                or not 0 <= guard_id < event["event_id"]
                or events[guard_id]["kind"] != "guard"
                or events[guard_id]["payload"].get("decision") != "accepted"
                or events[guard_id]["payload"].get("intent") != p["intent"]
            ):
                raise RecordingError("command lacks matching final guard approval")
            commands[event["event_id"]] = p["intent"]
        elif event["kind"] == "receipt":
            command_id = p["command_event_id"]
            if (
                command_id not in commands
                or p["intent"] != commands[command_id]
                or command_id in receipts.values()
            ):
                raise RecordingError("receipt/command association mismatch")
            if p["status"] != "executed" or p["device"] != "fake_robot":
                raise RecordingError("M1 requires explicit synchronous fake execution receipt")
            receipts[event["event_id"]] = command_id
        elif event["kind"] == "feedback":
            receipt_id = p["receipt_event_id"]
            if receipt_id not in receipts or receipt_id in feedback:
                raise RecordingError("feedback/receipt association mismatch")
            if p["positions"] != commands[receipts[receipt_id]]["positions"]:
                raise RecordingError("fake feedback/command positions mismatch")
            feedback.add(receipt_id)
        elif event["kind"] == "snapshot":
            for key, kind in (("state_event_id", "state"), ("camera_event_id", "camera")):
                index = p[key]
                if (
                    type(index) is not int
                    or not 0 <= index < event["event_id"]
                    or events[index]["kind"] != kind
                ):
                    raise RecordingError("snapshot association mismatch")
    if (
        len(commands) != len(receipts)
        or len(receipts) != len(feedback)
        or final["driver_calls"] != len(commands)
    ):
        raise RecordingError("incomplete execution evidence")


def summary(events):
    return {
        "mode": "fake",
        "session_id": events[0]["session_id"],
        "events": len(events),
        "counts": dict(Counter(e["kind"] for e in events)),
        "outcome": events[-1]["payload"]["outcome"],
        "driver_calls": events[-1]["payload"]["driver_calls"],
        "events_hash": digest(events),
        "rejections": dict(
            Counter(
                e["payload"]["reason"]
                for e in events
                if e["kind"] == "guard" and e["payload"]["decision"] == "rejected"
            )
        ),
    }
