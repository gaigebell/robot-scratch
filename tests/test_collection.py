import json
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import pytest
from mcap.reader import make_reader

from robot_infra.collection_demo import run_collection_demo
from robot_infra.collection_episode import CollectionRecorder, read_collection
from robot_infra.collection_runtime import CollectionSession, FakeBatchBackend

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def collection_config():
    return json.loads((ROOT / "configs/fake-a4s-collection.json").read_text())


def started(config, **kwargs):
    events = []
    s = CollectionSession(config, sink=events.append, **kwargs)
    s.start()
    return s, events


def target(s, source="teleop"):
    lease = s.acquire(source)
    snapshot = s.observe()
    values = {e["id"]: e["initial"] for e in s.config["endpoints"]}
    return s.intent(source, values, lease, snapshot)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda c: c.update(mode="real"),
        lambda c: c.update(schema_version="future"),
        lambda c: c.update(extra="unknown"),
        lambda c: c["endpoints"][0].update(unit="deg"),
        lambda c: c["endpoints"][0].update(backend="missing"),
        lambda c: c["endpoints"][1].update(id="left.arm"),
        lambda c: c["endpoints"][1].update(names=["joint1"]),
        lambda c: c["endpoints"][0].update(initial=[0]),
        lambda c: c["endpoints"][0].update(max_step=float("nan")),
        lambda c: c["endpoints"][0].update(limits=[[1, -1]] * 6),
        lambda c: c["group"].update(members=["left.arm"]),
        lambda c: c["timing"].update(period_ns=True),
        lambda c: c["cameras"].append(c["cameras"][0]),
        lambda c: c["sources"][1].update(id="teleop"),
    ],
)
def test_plan_errors_precede_backend_construction(collection_config, mutation):
    mutation(collection_config)

    def forbidden(*args):
        pytest.fail("invalid plan must not construct any backend")

    with pytest.raises(ValueError):
        CollectionSession(collection_config, backend_factory=forbidden)


def test_named_units_and_complete_backend_batch(collection_config):
    s, events = started(collection_config)
    i = target(s)
    values = tuple(
        replace(t, positions=(0.043,)) if t.endpoint_id == "left.gripper" else t for t in i.targets
    )
    result = s.submit(replace(i, targets=values))
    assert result["decision"] == "executed_fake"
    commands = [e["payload"] for e in events if e["kind"] == "command"]
    assert len(commands) == 2
    assert commands[0]["targets"] == {"left.arm": [0] * 6, "left.gripper": [0.043]}
    assert s.backends["right.piper"].values["right.gripper"] == (0.04,)
    with pytest.raises(PermissionError):
        s.backends["left.piper"].send({"left.arm": [0] * 6, "left.gripper": [0]}, object())


@pytest.mark.parametrize(
    "change,reason",
    [
        (lambda i: replace(i, epoch=True), "old_epoch"),
        (lambda i: replace(i, source="unknown"), "no_valid_lease"),
        (lambda i: replace(i, session_id="other"), "schema_or_session_mismatch"),
        (lambda i: replace(i, targets=i.targets[:-1]), "incomplete_or_duplicate_group"),
        (
            lambda i: replace(i, targets=i.targets + (i.targets[0],)),
            "incomplete_or_duplicate_group",
        ),
        (
            lambda i: replace(i, targets=(replace(i.targets[0], unit="m"),) + i.targets[1:]),
            "endpoint_semantics",
        ),
        (
            lambda i: replace(
                i,
                targets=(replace(i.targets[0], names=tuple(reversed(i.targets[0].names))),)
                + i.targets[1:],
            ),
            "endpoint_semantics",
        ),
        (
            lambda i: replace(i, targets=(replace(i.targets[0], positions=(0,)),) + i.targets[1:]),
            "dimension",
        ),
        (
            lambda i: replace(
                i,
                targets=i.targets[:2]
                + (replace(i.targets[2], positions=(2,) * 6),)
                + i.targets[3:],
            ),
            "limit",
        ),
        (
            lambda i: replace(
                i,
                targets=i.targets[:2]
                + (replace(i.targets[2], positions=(0.2,) * 6),)
                + i.targets[3:],
            ),
            "max_step",
        ),
        (
            lambda i: replace(
                i, targets=(replace(i.targets[0], positions=(float("nan"),) * 6),) + i.targets[1:]
            ),
            "non_finite_position",
        ),
        (lambda i: replace(i, target_ns=True), "invalid_time"),
    ],
)
def test_group_preflight_rejects_all_writes(collection_config, change, reason):
    s, _ = started(collection_config)
    result = s.submit(change(target(s)))
    assert result == {"decision": "rejected", "reason": reason}
    assert all(b.calls == 0 for b in s.backends.values())


def test_source_takeover_rejects_late_work_and_policy_cannot_preempt(collection_config):
    s, _ = started(collection_config)
    delayed = target(s, "stub_policy")
    human = s.acquire("teleop", takeover=True)
    assert s.submit(delayed)["reason"] == "old_epoch"
    with pytest.raises(PermissionError):
        s.acquire("stub_policy", takeover=True)
    snapshot = s.observe()
    values = {e["id"]: e["initial"] for e in s.config["endpoints"]}
    assert s.submit(s.intent("teleop", values, human, snapshot))["decision"] == "executed_fake"
    s.release(human)
    assert not s._snapshots and not s._accepted


def test_stale_feedback_and_explicit_fake_watchdog(collection_config):
    s, _ = started(collection_config)
    i = target(s)
    s.clock.advance(collection_config["timing"]["max_state_age_ns"] + 1)
    assert s.submit(replace(i, target_ns=s.clock.now_ns))["reason"] == "stale_feedback"
    s.clock.advance(s.lease.expires_ns - s.clock.now_ns)
    s.watchdog_tick()
    assert s.state == "faulted" and s.lease is None
    assert all(b.calls == 0 for b in s.backends.values())
    with pytest.raises(ValueError):
        s.start()
    s.reset()
    s.start()
    assert s.lease is None


def test_duplicate_and_expired_intents(collection_config):
    s, _ = started(collection_config)
    i = target(s)
    assert s.submit(i)["decision"] == "executed_fake"
    assert s.submit(i)["reason"] == "sequence"
    s.clock.advance(collection_config["timing"]["max_action_age_ns"])
    assert s.submit(replace(i, seq=2))["reason"] == "not_due_or_expired"
    assert all(b.calls == 1 for b in s.backends.values())


def test_partial_batch_failure_is_not_reported_as_atomic_rollback(collection_config, tmp_path):
    class Failing(FakeBatchBackend):
        def send(self, targets, capability):
            if "right.arm" in targets:
                raise OSError("injected second backend failure")
            return super().send(targets, capability)

    path = tmp_path / "partial.mcap"
    with CollectionRecorder(path) as recorder:
        s = CollectionSession(collection_config, sink=recorder.write, backend_factory=Failing)
        s.start()
        result = s.submit(target(s))
        assert [r["status"] for r in result["receipts"]] == [
            "executed_fake",
            "failed_execution_unknown",
        ]
        assert s.state == "faulted" and s.lease is None
        assert s.backends["left.piper"].calls == 1 and s.backends["right.piper"].calls == 0
        assert all(b.stop_calls for b in s.backends.values())
        recorder.finish(s.finish())
    result = read_collection(path)
    assert result["outcome"] == "aborted"
    assert result["backend_calls"] == {"left.piper": 1, "right.piper": 0}


def test_expiry_between_batches_prevents_second_send(collection_config):
    s, events = started(collection_config)
    i = target(s)
    original = s.backends["left.piper"].send

    def slow_send(values, capability):
        status = original(values, capability)
        s.clock.advance(collection_config["timing"]["lease_ttl_ns"])
        return status

    s.backends["left.piper"].send = slow_send
    assert s.submit(i)["reason"] == "lease_expired"
    assert s.backends["left.piper"].calls == 1 and s.backends["right.piper"].calls == 0
    assert s.state == "faulted"


def test_recording_failure_before_command_prevents_write(collection_config):
    def sink(event):
        if event["kind"] == "command":
            raise OSError("disk full")

    s = CollectionSession(collection_config, sink=sink)
    s.start()
    with pytest.raises(OSError, match="disk full"):
        s.submit(target(s))
    assert (
        s.state == "faulted" and s.lease is None and all(b.calls == 0 for b in s.backends.values())
    )


def test_streamed_episode_switches_sources_and_readback_creates_no_devices(
    collection_config, tmp_path, monkeypatch
):
    path = tmp_path / "collection.mcap"
    result = run_collection_demo(collection_config, path, steps=12)
    assert result["outcome"] == "complete" and result["backend_calls"] == {
        "left.piper": 12,
        "right.piper": 12,
    }
    with path.open("rb") as f:
        events = [
            json.loads(message.data)
            for _, _, message in make_reader(f).iter_messages(log_time_order=False)
        ]
    assert {e["payload"]["source"] for e in events if e["kind"] == "raw_source"} == {
        "teleop",
        "stub_policy",
    }
    assert {e["payload"]["role"] for e in events if e["kind"] == "camera"} == set(
        collection_config["cameras"]
    )
    assert all(
        e["payload"]["profile"] == "synthetic-native-joint-identity"
        for e in events
        if e["kind"] == "motion_result"
    )

    def forbidden(*args, **kwargs):
        pytest.fail("reader must not construct backends")

    monkeypatch.setattr(FakeBatchBackend, "__init__", forbidden)
    assert read_collection(path) == result
    with pytest.raises(FileExistsError):
        CollectionRecorder(path)
    broken = tmp_path / "broken.mcap"
    broken.write_bytes(path.read_bytes()[:-8])
    with pytest.raises(ValueError, match="footer"):
        read_collection(broken)
    with pytest.raises(ValueError):
        run_collection_demo(collection_config, tmp_path / "bad.mcap", steps=True)


def test_unfinalized_and_tampered_files_are_rejected(collection_config, tmp_path):
    path = tmp_path / "incomplete.mcap"
    with CollectionRecorder(path) as rec:
        s = CollectionSession(collection_config, sink=rec.write)
        s.start()
    with pytest.raises(ValueError):
        read_collection(path)
    full = tmp_path / "full.mcap"
    run_collection_demo(collection_config, full, steps=2)
    content = bytearray(full.read_bytes())
    content[len(content) // 2] ^= 1
    corrupt = tmp_path / "corrupt.mcap"
    corrupt.write_bytes(content)
    with pytest.raises(ValueError):
        read_collection(corrupt)


def test_required_streams_are_checked_beyond_file_finalization(collection_config, tmp_path):
    path = tmp_path / "missing-camera.mcap"
    with CollectionRecorder(path) as rec:

        def replace_role(event):
            if event["kind"] == "camera":
                event = deepcopy(event)
                event["payload"]["role"] = "unbound-camera"
            rec.write(event)

        s = CollectionSession(collection_config, sink=replace_role)
        s.start()
        rec.finish(s.finish())
    with pytest.raises(ValueError, match="required observations"):
        read_collection(path)


def test_snapshot_memory_is_bounded_and_one_arm_is_configurable(collection_config):
    c = deepcopy(collection_config)
    c["endpoints"] = c["endpoints"][:2]
    c["backends"] = c["backends"][:1]
    c["group"]["members"] = [e["id"] for e in c["endpoints"]]
    c["cameras"] = ["custom_rgb"]
    s, _ = started(c)
    for _ in range(80):
        s.observe()
    assert len(s._snapshots) == 32 and not hasattr(s, "events")
    assert s.submit(target(s))["decision"] == "executed_fake"


def test_fault_reset_does_not_erase_episode_failure(collection_config, tmp_path):
    path = tmp_path / "recovered.mcap"
    with CollectionRecorder(path) as rec:
        s = CollectionSession(collection_config, sink=rec.write)
        with pytest.raises(ValueError, match="unstarted"):
            s.finish()
        s.start()
        s.fault("test_fault")
        s.reset()
        s.start()
        rec.finish(s.finish())
    assert read_collection(path)["outcome"] == "aborted"


def test_reader_rejects_failure_mislabeled_complete(collection_config, tmp_path):
    path = tmp_path / "mislabeled.mcap"
    with CollectionRecorder(path) as rec:
        s = CollectionSession(collection_config, sink=rec.write)
        s.start()
        s.fault("test_fault")
        result = s.finish()
        result["outcome"] = "complete"
        rec.finish(result)
    with pytest.raises(ValueError, match="mislabeled complete"):
        read_collection(path)


def test_cli_validation_and_m1_boundary(collection_config, tmp_path, capsys):
    from robot_infra.cli import main
    from robot_infra.graph import ConfigError, validate_graph

    with pytest.raises(ConfigError):
        validate_graph(collection_config)
    assert (
        main(["collect-validate", str(ROOT / "configs/fake-a4s-collection.json"), "--dry-run"]) == 0
    )
    assert json.loads(capsys.readouterr().out)["physical_commands"] is False
    path = tmp_path / "cli.mcap"
    assert (
        main(
            [
                "collect-demo",
                str(ROOT / "configs/fake-a4s-collection.json"),
                "--output",
                str(path),
                "--steps",
                "2",
            ]
        )
        == 0
    )
    capsys.readouterr()
    assert main(["collect-read", str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["outcome"] == "complete"
