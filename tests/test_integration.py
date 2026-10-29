import json
import subprocess
import sys
from copy import deepcopy
from pathlib import Path

import pytest
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

from robot_infra.app import run_demo
from robot_infra.contracts import canonical, digest
from robot_infra.recording import PROFILE, SCHEMA_NAME, Recorder, RecordingError, replay
from robot_infra.runtime import Session

ROOT = Path(__file__).resolve().parents[1]


def cli(*args):
    return subprocess.run(
        [sys.executable, "-m", "robot_infra", *map(str, args)],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )


def test_end_to_end_cli_and_independent_mcap_reader(tmp_path):
    output = tmp_path / "demo.mcap"
    valid = cli("validate", "configs/fake-control.json", "--dry-run")
    assert (
        valid.returncode == 0
        and json.loads(valid.stdout)["plan"]["nodes_by_kind"]["guard"] == "guard"
    )
    run = cli("run", "configs/fake-control.json", "--output", output)
    assert run.returncode == 0, run.stderr
    report = json.loads(run.stdout)
    assert report["driver_calls"] == 14
    assert report["rejections"] == {
        "old_epoch": 1,
        "joint_limit": 1,
        "not_due_or_expired": 1,
        "no_valid_lease": 1,
    }
    # Use the public MCAP reader directly, independently of our replay code.
    with output.open("rb") as stream:
        reader = make_reader(stream, validate_crcs=True)
        assert reader.get_header().profile == PROFILE
        messages = list(reader.iter_messages(log_time_order=False))
    assert len(messages) == report["events"]
    decoded = [json.loads(message.data) for _, _, message in messages]
    camera = next(e["payload"] for e in decoded if e["kind"] == "camera")
    assert (
        camera["role"] == "front"
        and camera["width"] == 2
        and camera["clock_domain"] == "fake_monotonic"
    )
    assert all(
        schema.name == SCHEMA_NAME and channel.message_encoding == "json"
        for schema, channel, _ in messages
    )
    read = cli("replay", output)
    assert read.returncode == 0 and json.loads(read.stdout) == report
    jsonl = cli("replay", output, "--jsonl")
    assert jsonl.returncode == 0
    assert [json.loads(line) for line in jsonl.stdout.splitlines()] == decoded
    before = output.read_bytes()
    again = cli("run", "configs/fake-control.json", "--output", output)
    assert again.returncode == 2 and output.read_bytes() == before


def test_live_equals_replay_and_deterministic_events(config, tmp_path):
    timelines = []
    for index in range(2):
        output = tmp_path / f"run-{index}.mcap"
        with Recorder(output) as rec:
            s = Session(config, session_id="deterministic", sink=rec.write)
            s.start()
            lease = s.acquire("teleop")
            s.submit(s.intent("teleop", (0.1, 0.1), lease))
            s.submit(s.intent("teleop", (float("nan"), 0.0), lease))
            s.finalize()
        assert replay(output) == s.events
        timelines.append(replay(output))
    assert canonical(timelines[0]) == canonical(timelines[1])


def test_replay_does_not_construct_or_write_devices(config, tmp_path, monkeypatch):
    output = tmp_path / "run.mcap"
    run_demo(config, output, steps=2)

    def forbidden(*args, **kwargs):
        pytest.fail("replay touched the runtime/device")

    monkeypatch.setattr("robot_infra.runtime.Session.__init__", forbidden)
    monkeypatch.setattr("robot_infra.fake.FakeRobot.__init__", forbidden)
    monkeypatch.setattr("robot_infra.fake.FakeRobot._execute", forbidden)
    assert replay(output)[-1]["kind"] == "finalize"


def test_faulted_episode_has_explicit_aborted_outcome(config, tmp_path):
    output = tmp_path / "aborted.mcap"
    with Recorder(output) as rec:
        s = Session(config, sink=rec.write)
        s.start()
        s.fault("injected_fault")
        s.finalize()
    assert replay(output)[-1]["payload"]["outcome"] == "aborted"


def rewrite(path, events, schema_data, *, schema_name=SCHEMA_NAME, repair_hash=False):
    """External writer makes structurally valid MCAP with invalid application evidence."""
    events = deepcopy(events)
    if repair_hash:
        events[-1]["payload"]["events_hash"] = digest(events[:-1])
        events[-1]["payload"]["event_count"] = len(events) - 1
    with path.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.NONE, enable_data_crcs=True)
        writer.start(profile=PROFILE)
        sid = writer.register_schema(schema_name, "jsonschema", schema_data)
        for e in events:
            channel = writer.register_channel(f"/events/{e['kind']}", "json", sid)
            writer.add_message(
                channel, e["time_ns"], canonical(e), e["time_ns"], sequence=e["event_id"]
            )
        writer.finish()


@pytest.mark.parametrize(
    "damage",
    [
        "truncated",
        "crc",
        "schema",
        "version",
        "hash",
        "association",
        "guard",
        "camera_snapshot",
        "missing_finalize",
    ],
)
def test_corruption_and_unsupported_evidence_rejected(config, tmp_path, damage):
    good = tmp_path / "good.mcap"
    run_demo(config, good, steps=1)
    raw = good.read_bytes()
    events = replay(good)
    schema = (ROOT / "src/robot_infra/schemas/event.json").read_bytes()
    bad = tmp_path / "bad.mcap"
    if damage == "truncated":
        bad.write_bytes(raw[:-12])
    elif damage == "crc":
        # Corrupt actual JSON in a chunk, keep envelope/footer otherwise intact.
        data = bytearray(raw)
        pos = data.find(b"joint_a")
        assert pos > 0
        data[pos] ^= 1
        bad.write_bytes(data)
    else:
        name = SCHEMA_NAME
        repair = False
        if damage == "schema":
            name = "unknown.Event/9"
        elif damage == "version":
            events[0]["schema_version"] = "9"
        elif damage == "hash":
            events[0]["payload"]["policy_artifact"]["id"] = "tampered"
        elif damage == "association":
            next(e for e in events if e["kind"] == "receipt")["payload"]["command_event_id"] = 0
            repair = True
        elif damage == "guard":
            next(e for e in events if e["kind"] == "command")["payload"]["guard_event_id"] = 0
            repair = True
        elif damage == "camera_snapshot":
            next(e for e in events if e["kind"] == "snapshot")["payload"]["camera_event_id"] = 0
            repair = True
        elif damage == "missing_finalize":
            events.pop()
        rewrite(bad, events, schema, schema_name=name, repair_hash=repair)
    with pytest.raises(RecordingError):
        replay(bad)
    result = cli("replay", bad, "--jsonl")
    assert result.returncode == 2 and not result.stdout  # No partial success stream.


def test_interrupted_file_not_reported_as_complete(config, tmp_path):
    output = tmp_path / "interrupted.mcap"
    with pytest.raises(RuntimeError):
        with Recorder(output) as rec:
            s = Session(config, sink=rec.write)
            s.start()
            raise RuntimeError("interrupted")
    assert output.exists()
    with pytest.raises(RecordingError):
        replay(output)


def test_cli_invalid_config_creates_no_recording(config, tmp_path):
    config["mode"] = "real"
    bad = tmp_path / "real.json"
    bad.write_text(json.dumps(config))
    output = tmp_path / "forbidden.mcap"
    result = cli("run", bad, "--output", output)
    assert result.returncode == 2 and not output.exists()
    assert cli("validate", bad).returncode == 2


def test_invalid_step_count_rejected_before_recording(config, tmp_path):
    output = tmp_path / "invalid.mcap"
    with pytest.raises(ValueError):
        run_demo(config, output, steps=0)
    assert not output.exists()
