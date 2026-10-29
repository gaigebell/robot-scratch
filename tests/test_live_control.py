import math
from dataclasses import replace

import pytest

from robot_infra.kinematics import SerialModel, origin
from robot_infra.live_control import LiveGateway, RelativePositionMapper
from robot_infra.live_teleop import validate_profile


@pytest.fixture
def model(tmp_path):
    text = '<robot name="test"><link name="base"/>'
    for i, axis in enumerate(("0 0 1", "0 1 0", "0 1 0"), 1):
        text += (
            f'<link name="l{i}"/><joint name="j{i}" type="revolute"><parent link="'
            + ("base" if i == 1 else f"l{i - 1}")
            + f'"/><child link="l{i}"/><origin xyz="{0 if i == 1 else 1} 0 0"/><axis xyz="{axis}"/><limit lower="-2" upper="2"/></joint>'
        )
    text += '<link name="tip"/><joint name="tool" type="fixed"><parent link="l3"/><child link="tip"/><origin xyz="1 0 0"/></joint></robot>'
    path = tmp_path / "model.urdf"
    path.write_text(text)
    return SerialModel(path, "base", "tip", ["j1", "j2", "j3"])


def test_fk_ik_local_and_unreachable(model):
    assert [r[3] for r in model.fk([0, 0, 0])][:3] == [3, 0, 0]
    q = [0.3, -0.4, 0.2]
    target = model.fk([0.31, -0.39, 0.19])
    result = model.ik(target, q, max_ms=500)
    assert result.accepted and result.position_error_m < 0.0008
    assert not model.ik(origin([10, 0, 0], [0, 0, 0]), q, max_ms=500).accepted
    with pytest.raises(ValueError, match="seed"):
        model.ik(target, [3, 0, 0])


class Backend:
    def __init__(self):
        self.sent, self.enabled = [], []

    def bind(self, capability):
        self.capability = capability

    def enable(self, side, capability):
        assert capability is self.capability
        self.enabled.append(side)

    def send(self, side, q, gripper, capability):
        assert capability is self.capability
        self.sent.append((side, tuple(q)))


@pytest.fixture
def setup(model):
    clock = [10.0]
    config = {
        "schema_version": "live-teleop/1",
        "gripper_control": False,
        "arms": {
            s: {"command_topic": "/" + s + "/cmd", "enable_topic": "/" + s + "/enable"}
            for s in ("left", "right")
        },
        "initial_limit_tolerance_rad": 0.06,
        "feedback_timeout_s": 0.2,
        "input_timeout_s": 0.3,
        "max_joint_excursion_rad": 0.12,
        "max_tracking_error_rad": 0.03,
        "max_joint_speed_rad_s": 0.1,
        "max_cartesian_delta_m": 0.04,
        "scale": 1.0,
        "clutch_button": 1,
        "driver_speed_percent": 5,
        "alignment": [[0, 0, -1], [-1, 0, 0], [0, 1, 0]],
        "ik_deadline_ms": 35,
    }
    backend, events = Backend(), []
    g = LiveGateway(
        validate_profile(config),
        model,
        backend,
        lambda k, p: events.append((k, p)),
        clock=lambda: clock[0],
    )
    for s in config["arms"]:
        g.feedback(s, [0.3, -0.4, 0.2], 0.0, 123)
    return g, backend, clock, events


def test_ownership_and_guard_before_any_write(setup):
    g, b, t, e = setup
    g.arm("vr")
    initial = len(b.sent)
    t[0] += 0.05
    values = {s: [0.301, -0.4, 0.2] for s in g.config["arms"]}
    command = g.command(values, 1)
    assert not g.submit(replace(command, source="other"))["accepted"]
    assert len(b.sent) == initial
    assert g.submit(command)["accepted"]
    assert not g.submit(command)["accepted"]
    assert len(b.sent) == initial + 2
    with pytest.raises(ValueError):
        g.arm("other")


def test_watchdog_revoke_hold_and_old_epoch(setup):
    g, b, t, e = setup
    g.arm("vr")
    command = g.command({s: g.states[s]["q"] for s in g.states}, 1)
    old_epoch = g.epoch
    t[0] += 0.31
    for s in g.states.copy():
        g.feedback(s, [0.3, -0.4, 0.2], 0, 124)
    g.watchdog()
    assert g.owner is None and not g.enabled and g.epoch > old_epoch
    assert e[-1][1]["result"] == "measured_hold_published_not_acknowledged"
    assert not g.submit(command)["accepted"]


def test_frozen_device_stamp_is_not_fresh(setup):
    g, b, t, e = setup
    g.arm("vr")
    count = len(b.sent)
    t[0] += 0.21
    for s in g.states.copy():
        g.feedback(s, [0.3, -0.4, 0.2], 0, 123)
    g.watchdog()
    assert not g.enabled and g.last_reason == "stale_feedback"
    assert len(b.sent) == count
    assert e[-1][1]["result"] == "no_fresh_feedback_hold_unavailable"


@pytest.mark.parametrize(
    "change,reason",
    [
        ("nan", "joint_semantics"),
        ("speed", "joint_speed"),
        ("tracking", "tracking_error"),
        ("missing", "endpoint_set"),
        ("expired", "command_expired"),
    ],
)
def test_guard_rejects_bad_commands(setup, change, reason):
    g, b, t, e = setup
    g.arm("vr")
    t[0] += 0.05
    command = g.command({s: g.states[s]["q"] for s in g.states}, 1)
    if change in ("nan", "speed", "tracking"):
        v = {"nan": math.nan, "speed": 0.31, "tracking": 0.34}[change]
        first = replace(command.targets[0], positions=(v, -0.4, 0.2))
        command = replace(command, targets=(first, command.targets[1]))
    if change == "missing":
        command = replace(command, targets=command.targets[:1])
    if change == "expired":
        command = replace(command, expires_ns=0)
    count = len(b.sent)
    assert g.submit(command)["reason"] == reason
    assert len(b.sent) == count


def test_partial_send_fault_revoke(setup):
    g, b, t, e = setup
    g.arm("vr")
    t[0] += 0.05
    original = b.send

    def fail(side, *args):
        if side == "right":
            raise OSError("test failure")
        original(side, *args)

    b.send = fail
    with pytest.raises(OSError):
        g.submit(g.command({s: g.states[s]["q"] for s in g.states}, 1))
    assert g.faulted and g.owner is None


def test_mapper_reference_reset_revokes(setup):
    g, b, t, e = setup
    g.arm("vr")
    mapper = RelativePositionMapper(g)
    p = {
        "schema_version": "webxr-input/1",
        "session_id": "vr",
        "seq": 0,
        "event": "frame",
        "reference_space": "local-floor",
        "epoch": 0,
        "client_time_ms": 1.0,
        "client_unix_ms": 1.0,
        "visibility": "visible",
        "head": {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]},
        "inputs": [],
    }
    mapper.process(p)
    p.update(seq=1, epoch=1)
    mapper.process(p)
    assert not g.enabled and g.last_reason == "reference_reset"


def test_profile_reflection_rejected(setup):
    g, *_ = setup
    c = {**g.config, "alignment": [[1, 0, 0], [0, 1, 0], [0, 0, -1]]}
    with pytest.raises(ValueError, match="rotation"):
        validate_profile(c)


def test_initial_outside_model_rejected_before_enable(setup):
    g, b, t, e = setup
    g.feedback("left", [-2.01, -0.4, 0.2], 0, 125)
    with pytest.raises(ValueError, match="initial state outside"):
        g.arm("vr")
    assert not b.enabled and not b.sent


def test_journal_failure_still_revokes_and_attempts_hold(setup):
    g, b, t, e = setup
    g.arm("vr")
    count = len(b.sent)

    def broken(*args):
        raise OSError("disk failure")

    g.emit = broken
    g.stop("journal_failure", fault=True)
    assert g.faulted and not g.enabled and g.owner is None
    assert len(b.sent) == count + 2


def test_small_measurement_error_projects_seed_but_not_command_limits(setup):
    g, b, t, e = setup
    g.config["measurement_tolerance_rad"] = 0.005
    g.feedback("right", [0.3, -0.4, 2.001], 0, 125)
    g.arm("vr")
    assert b.sent[0][1][-1] <= 2 and b.sent[1][1][-1] <= 2
    target = g.model.fk([0.3, -0.4, 1.999])
    r = g.model.ik(target, [0.3, -0.4, 2.001], seed_tolerance_rad=0.005, max_ms=500)
    assert r.accepted and r.seed_projection_rad == pytest.approx(0.001)
    assert all(lo <= v <= hi for v, (lo, hi) in zip(r.positions, g.model.limits))


def test_explicit_unrestricted_test_envelope_profile(setup):
    g, *_ = setup
    c = {
        **g.config,
        "max_cartesian_delta_m": None,
        "max_joint_excursion_rad": None,
        "driver_speed_percent": 100,
    }
    assert validate_profile(c)["max_cartesian_delta_m"] is None


def test_http_diagnostics_do_not_arm_robot(setup, tmp_path):
    import json
    import threading
    from urllib.request import Request, urlopen

    from robot_infra.live_teleop import EventJournal, LiveApp, make_server

    g, b, t, e = setup
    journal = EventJournal(tmp_path / "journal")
    app = LiveApp(g.config, g.model, Backend(), journal)
    server = make_server(("127.0.0.1", 0), app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = "http://127.0.0.1:" + str(server.server_port)
    try:
        with urlopen(base + "/api/status") as response:
            assert not json.load(response)["enabled"]
        with urlopen(
            Request(
                base + "/api/client-event",
                data=b'{"stage":"test"}',
                headers={"Content-Type": "application/json"},
            )
        ) as response:
            assert json.load(response)["recorded"]
        assert not app.gateway.backend.enabled and not app.gateway.backend.sent
    finally:
        app.keep_running = False
        app.watchdog_thread.join(1)
        server.shutdown()
        server.server_close()
        thread.join(1)
        journal.emit("finalize", {"outcome": "unit_test"})
        journal.close()
    events = [json.loads(line) for line in (journal.root / "events.jsonl").read_text().splitlines()]
    assert [x["kind"] for x in events] == ["client_event", "finalize"]
    assert json.loads((journal.root / "journal-summary.json").read_text())["error"] is None
