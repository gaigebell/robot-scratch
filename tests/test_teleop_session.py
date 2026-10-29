import json
from copy import deepcopy
from pathlib import Path

import pytest

from robot_infra.live_control import LiveGateway
from robot_infra.teleop_session import TeleopSession, validate_session_profile


class Model:
    names = ["j1", "j2"]
    base = "base"
    tip = "tool"
    limits = [(-2.0, 2.0), (-2.0, 2.0)]

    def fk(self, q):
        return [[1, 0, 0, q[0]], [0, 1, 0, q[1]], [0, 0, 1, 0], [0, 0, 0, 1]]

    def relative_target(self, pose0, hand0, hand, alignment, scale, rotation):
        target = deepcopy(pose0)
        target[0][3] += hand["position_m"][0] - hand0["position_m"][0]
        return target


class Backend:
    def __init__(self):
        self.calls = []
        self.q = {s: (0.3, -0.3) for s in ("left", "right")}
        self.jaws = {s: 0.03 for s in self.q}

    def bind(self, cap):
        self.cap = cap

    def enable(self, side, cap):
        assert cap is self.cap
        self.calls.append(("enable", side))

    def send(self, side, q, jaw, cap):
        assert cap is self.cap
        self.q[side] = tuple(q)
        if jaw is not None:
            self.jaws[side] = jaw
        self.calls.append(("send", side, tuple(q), jaw))

    def send_gripper(self, side, width, cap):
        assert cap is self.cap
        self.jaws[side] = width
        self.calls.append(("jaw", side, width))


class Worker:
    ready = True
    error = None

    def __init__(self):
        self.latest = self.result = None
        self.closed = False
        self.resolve = True

    def request(self, goal):
        self.latest = goal

    def poll(self):
        if self.latest and self.resolve:
            g = self.latest
            self.latest = None
            self.result = {
                "token": g["token"],
                "goal_seq": g["seq"],
                "created_s": g["created_s"],
                "result": {"positions": [g["target"][0][3], g["target"][1][3]], "accepted": True},
            }

    def take_result(self):
        r, self.result = self.result, None
        return r

    def clear(self):
        self.latest = self.result = None

    def close(self):
        self.closed = True


@pytest.fixture
def rig():
    config = json.loads(
        (Path(__file__).parents[1] / "configs/teleop-session.example.json").read_text()
    )
    config["gripper_control_mode"] = (
        "clutch"  # compatibility fixture; independent mode tested explicitly below
    )
    config["model"]["joint_names"] = Model.names
    for arm in config["arms"].values():
        arm["home_rad"] = [0.0, 0.0]
    config["session"].update(home_duration_s=0.3, home_settle_s=0.1, home_timeout_s=2.0)
    validate_session_profile(config)
    now = [10.0]
    events = []
    backend = Backend()
    model = Model()
    g = LiveGateway(
        config, model, backend, lambda k, p: events.append((k, p)), clock=lambda: now[0]
    )
    workers = {s: Worker() for s in config["arms"]}
    runtime = TeleopSession(g, workers, clock=lambda: now[0])
    seq = [0]
    values = {"A": 0.0, "left": 0.0, "right": 0.0, "trigger": 0.0}
    stamp = [1]

    def step(dt=0.02, **change):
        now[0] += dt
        values.update(change)
        stamp[0] += 1
        for s in config["arms"]:
            g.feedback(s, backend.q[s], backend.jaws[s], stamp[0])
        hands = []
        for side in config["arms"]:
            buttons = [{"value": 0.0, "pressed": False, "touched": False} for _ in range(6)]
            for index, value in (
                (0, values["trigger"]),
                (1, values[side]),
                (4, values["A"] if side == "right" else 0.0),
            ):
                buttons[index] = {"value": value, "pressed": bool(value), "touched": bool(value)}
            hands.append(
                {
                    "handedness": side,
                    "source_id": side,
                    "grip": {
                        "position_m": [values.get("x", 0.0), 0.0, 0.0],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                    "gamepad": {"buttons": buttons, "axes": []},
                }
            )
        packet = {
            "schema_version": "webxr-input/1",
            "event": "frame",
            "session_id": "vr",
            "seq": seq[0],
            "epoch": 0,
            "reference_space": "local-floor",
            "client_time_ms": 1.0,
            "client_unix_ms": 1.0,
            "visibility": "visible",
            "head": {"position_m": [0.0, 1.0, 0.0], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
            "inputs": hands,
        }
        seq[0] += 1
        runtime.ingest(packet)
        runtime.tick()
        return packet

    step()  # self check -> standby, neutral observed
    return runtime, g, backend, workers, step, now, events


def to_ready(rig):
    runtime, g, b, w, step, t, e = rig
    step(A=1.0)
    assert runtime.state == "HOMING"
    for _ in range(8):
        step(0.1)  # held A must not also start teleop
    assert runtime.state == "READY"
    step(A=0.0)


def test_home_status_reports_each_arm_without_changing_settle_threshold(rig):
    r, g, b, w, step, t, e = rig
    assert r.snapshot()["home"] is None
    step(A=1.0)
    with g.lock:
        g.states["right"]["q"] = (0.12, -0.04)
    progress = r.snapshot()["home"]
    assert progress["error_rad"]["right"] == pytest.approx(0.12)
    assert progress["tolerance_rad"] == r.config["session"]["home_tolerance_rad"]
    assert progress["settled_s"] == 0.0
    for _ in range(8):
        step(0.1)
    assert r.state == "READY" and r.snapshot()["home"] is None


def test_A_home_ready_start_end_and_gripper_omission(rig):
    r, g, b, w, step, t, e = rig
    assert r.state == "STANDBY" and not b.calls
    to_ready(rig)
    assert all(call[3] is None for call in b.calls if call[0] == "send")
    assert b.jaws == {"left": 0.03, "right": 0.03}
    count = len(b.calls)
    step(A=1.0)
    assert r.state == "TELEOP" and len(b.calls) == count  # start does not move/enable
    step(A=0.0)
    step(left=1.0, trigger=0.5)
    step(x=0.08)
    step(x=0.08)
    assert b.q["left"][0] > 0.05 and b.jaws["left"] == pytest.approx(0.035)
    assert b.q["right"] == pytest.approx([0.0, 0.0]) and b.jaws["right"] == 0.03
    step(left=0.0)
    assert not g.active_sides
    step(A=1.0)
    assert r.state == "STANDBY" and r.trials == 1 and g.owner is None


def test_pause_invalidates_worker_and_resume_requires_released_buttons(rig):
    r, g, b, w, step, t, e = rig
    to_ready(rig)
    step(A=1.0)
    step(A=0.0, left=1.0)
    token = r.tokens["left"]
    epoch = g.epoch
    r.request_pause("vr_exit")
    r.tick()
    assert r.state == "PAUSED" and g.epoch > epoch and not g.enabled
    step(A=1.0, left=1.0)
    assert r.state == "PAUSED"
    step(A=0.0, left=0.0)
    step(A=1.0)
    assert r.state == "TELEOP" and not r.anchors
    step(A=0.0, left=1.0)
    assert r.tokens["left"] != token
    w["left"].result = {
        "token": token,
        "goal_seq": 0,
        "created_s": t[0],
        "result": {"accepted": True, "positions": [1.5, 1.5]},
    }
    w["left"].resolve = False
    step()
    assert b.q["left"][0] != 1.5


def test_no_result_cannot_block_A_or_pause(rig):
    r, g, b, w, step, t, e = rig
    to_ready(rig)
    step(A=1.0)
    step(A=0.0)
    for worker in w.values():
        worker.resolve = False
    step(left=1.0)
    step(A=1.0)
    assert r.state == "STANDBY"


def test_home_cancelled_by_VR_pause(rig):
    r, g, b, w, step, t, e = rig
    step(A=1.0)
    r.request_pause("VR_hidden")
    r.tick()
    count = len(b.calls)
    assert r.state == "PAUSED" and r.resume_state == "STANDBY"
    t[0] += 1
    r.tick()
    assert len(b.calls) == count


@pytest.mark.parametrize("phase", ["STANDBY", "READY"])
def test_A_after_idle_pause_performs_displayed_phase_action(rig, phase):
    r, g, b, w, step, t, e = rig
    if phase == "READY":
        to_ready(rig)
    r.request_pause("page_navigation")
    r.tick()
    assert r.state == "PAUSED" and r.resume_state == phase
    step(A=0.0)
    step(A=1.0)
    assert r.state == ("HOMING" if phase == "STANDBY" else "TELEOP")


def test_input_timeout_releases_without_new_frame(rig):
    r, g, b, w, step, t, e = rig
    to_ready(rig)
    step(A=1.0)
    step(A=0.0, left=1.0)
    t[0] += 0.7
    r.tick()
    assert r.state == "PAUSED" and not g.enabled


def test_free_target_has_no_commissioning_cartesian_envelope(rig):
    r, g, b, w, step, t, e = rig
    to_ready(rig)
    step(A=1.0)
    step(A=0.0, left=1.0)
    step(x=0.7)
    step(x=0.7)
    assert b.q["left"][0] > 0.6
    assert not [p for k, p in e if k == "guard" and p.get("reason") == "test_envelope"]


def test_solver_failure_faults_and_close_cleans_workers(rig):
    r, g, b, w, step, t, e = rig
    w["right"].error = "hung"
    r.tick()
    assert r.state == "FAULT"
    r.close()
    assert all(x.closed for x in w.values()) and r.state == "CLOSED"


def test_invalid_profile_rejected_before_resources(rig):
    r, *_ = rig
    c = deepcopy(r.config)
    c["arms"]["left"].pop("home_rad")
    with pytest.raises(ValueError, match="home"):
        validate_session_profile(c)
    c = deepcopy(r.config)
    c["recording_enabled"] = True
    c.pop("recording")
    with pytest.raises(ValueError, match="recording"):
        validate_session_profile(c)


def test_jaw_limit_rejected_by_shared_gateway(rig):
    r, g, b, w, step, t, e = rig
    to_ready(rig)
    step(A=1.0)
    step(A=0.0, left=1.0)
    q = {s: b.q[s] for s in b.q}
    command = g.command(q, 999, {"left": 0.2})
    count = len(b.calls)
    assert g.submit(command, active_sides={"left"}, jaw_sides={"left"})["reason"] == "gripper_limit"
    assert len(b.calls) == count


@pytest.mark.parametrize("action,pressed,released", [("open", 0.07, 0.0), ("close", 0.0, 0.07)])
def test_configured_trigger_polarity_only_applies_under_clutch(rig, action, pressed, released):
    r, g, b, w, step, t, e = rig
    r.config["arms"]["left"]["gripper"]["trigger_pressed"] = action
    to_ready(rig)
    step(A=1.0)
    step(A=0.0, trigger=1.0)
    assert b.jaws["left"] == 0.03
    step(left=1.0)
    assert b.jaws["left"] == pytest.approx(pressed)
    step(trigger=0.0)
    assert b.jaws["left"] == pytest.approx(released)
    step(left=0.0, trigger=1.0)
    assert b.jaws["left"] == pytest.approx(released)


def test_vendor_enable_does_not_close_gripper():
    from robot_infra.adapters.piper_driver import driver_class

    class SDK:
        def __init__(self):
            self.enabled = []

        def EnableArm(self, n):
            self.enabled.append(n)

        def GripperCtrl(self, *args):
            raise AssertionError("unexpected gripper side effect")

    class Vendor:
        class PiperRosNode:
            pass

    cls = driver_class(Vendor, None)
    node = cls.__new__(cls)
    node.piper = SDK()

    class Message:
        data = True

    node.enable_callback(Message())
    assert node.piper.enabled == [7] and node._PiperRosNode__enable_flag


class Recorder:
    def __init__(self):
        self.state = "IDLE"
        self.saved = 0
        self.decisions = []

    def snapshot(self):
        return {"state": self.state, "error": None, "saved_episodes": self.saved}

    def start(self, context):
        self.state = "STARTING"

    def finish(self, reason=None):
        self.state = "FINALIZING"
        self.reason = reason

    def decide(self, save):
        self.decisions.append(save)
        self.state = "SAVING"


def test_recording_lifecycle_is_asynchronous_and_A_X_are_review_only(rig):
    r, g, b, w, step, t, e = rig
    rec = Recorder()
    r.recorder = rec
    to_ready(rig)
    count = len(b.calls)
    step(A=1.0)
    assert r.state == "PREPARING" and rec.state == "STARTING" and len(b.calls) == count
    step(A=0.0)
    rec.state = "RECORDING"
    step()
    assert r.state == "TELEOP"
    step(A=1.0)
    assert r.state == "FINALIZING" and not g.enabled
    step(A=0.0)
    rec.state = "REVIEW"
    step()
    assert r.state == "REVIEW"
    step(A=1.0)
    assert r.state == "COMMITTING" and rec.decisions == [True]
    rec.state = "IDLE"
    rec.saved = 1
    step(A=0.0)
    assert r.state == "STANDBY"


def test_recording_VR_loss_interrupts_and_requires_review_before_new_motion(rig):
    r, g, b, w, step, t, e = rig
    rec = Recorder()
    r.recorder = rec
    to_ready(rig)
    step(A=1.0)
    step(A=0.0)
    rec.state = "RECORDING"
    step()
    r.request_pause("VR_exit")
    r.tick()
    assert rec.reason == "VR_exit" and not g.enabled
    step(A=0.0)
    step(A=1.0)
    assert r.state == "FINALIZING"  # A cannot re-home while draining
    rec.state = "REVIEW"
    step(A=0.0)
    assert r.state == "REVIEW"
    # Replay a left button4 rising edge without enabling a hardware path.
    p = step()
    p["seq"] += 100
    p["inputs"][0]["gamepad"]["buttons"][4].update(value=1.0, pressed=True)
    r.ingest(p)
    r.tick()
    assert rec.decisions == [False] and r.state == "COMMITTING"


def test_fast_overflow_finalize_cannot_leave_teleop_moving_without_recording(rig):
    r, g, b, w, step, t, e = rig
    rec = Recorder()
    r.recorder = rec
    to_ready(rig)
    step(A=1.0)
    step(A=0.0)
    rec.state = "RECORDING"
    step(left=1.0)
    rec.state = "REVIEW"
    step()
    assert r.state == "REVIEW" and not g.enabled and not r.anchors


def test_independent_trigger_sends_no_arm_enable_or_arm_target(rig):
    r, g, b, w, step, t, e = rig
    r.config["gripper_control_mode"] = "independent_trigger"
    to_ready(rig)
    step(A=1.0)
    step(A=0.0)
    b.calls.clear()
    q = deepcopy(b.q)
    j = deepcopy(b.jaws)
    step()
    assert (
        not b.calls and b.jaws == j
    )  # entering teleop preserves jaw until a deliberate trigger change
    step(trigger=1.0)
    assert b.calls and all(c[0] == "jaw" for c in b.calls)
    assert b.q == q and b.jaws == {"left": 0.07, "right": 0.07}
    step(trigger=0.0)
    assert b.jaws == {"left": 0.0, "right": 0.0} and not g.active_sides
    r.request_pause("VR_exit")
    r.tick()
    count = len(b.calls)
    step(trigger=1.0)
    assert len(b.calls) == count


def test_follow_trigger_opens_by_default_and_closes_without_clutch(rig):
    r, g, b, w, step, t, e = rig
    r.config["gripper_control_mode"] = "independent_trigger"
    for arm in r.config["arms"].values():
        arm["gripper"].update(trigger_pressed="close", teleop_start="follow_trigger")
    to_ready(rig)
    assert b.jaws == {"left": 0.03, "right": 0.03}
    b.calls.clear()
    q = deepcopy(b.q)
    step(A=1.0)  # First TELEOP packet already follows the released trigger.
    assert b.jaws == {"left": 0.07, "right": 0.07} and all(c[0] == "jaw" for c in b.calls)
    step(A=0.0, trigger=0.5)
    assert all(v == pytest.approx(0.035) for v in b.jaws.values())
    step(trigger=1.0)
    assert b.jaws == {"left": 0.0, "right": 0.0} and b.q == q and not g.active_sides
    step(trigger=0.0)
    assert b.jaws == {"left": 0.07, "right": 0.07}
    r.request_pause("VR_exit")
    r.tick()
    count = len(b.calls)
    step(trigger=1.0)
    assert len(b.calls) == count


def test_follow_trigger_needs_controller_and_profile_value_is_checked(rig):
    r, g, b, w, step, t, e = rig
    c = deepcopy(r.config)
    c["arms"]["left"]["gripper"]["teleop_start"] = "open_on_driver_start"
    with pytest.raises(ValueError, match="start behavior"):
        validate_session_profile(c)
    r.config["gripper_control_mode"] = "independent_trigger"
    for arm in r.config["arms"].values():
        arm["gripper"].update(trigger_pressed="close", teleop_start="follow_trigger")
    to_ready(rig)
    step(A=1.0)
    step(A=0.0)
    b.calls.clear()
    r._teleop_tick({"inputs": [], "epoch": 0, "seq": 999}, False)
    assert not b.calls
    r._teleop_tick(
        {
            "inputs": [
                {
                    "handedness": "left",
                    "source_id": "missing-trigger",
                    "gamepad": {"buttons": [], "axes": []},
                }
            ],
            "epoch": 0,
            "seq": 1000,
        },
        False,
    )
    assert not b.calls  # Missing button data must not become an implicit released trigger.


def test_jaw_only_driver_never_commands_arm():
    from robot_infra.adapters.piper_driver import driver_class

    class SDK:
        def __init__(self):
            self.calls = []

        def GetSDKGripperRangeParam(self):
            return 0.0, 0.07

        def GripperCtrl(self, *args):
            self.calls.append(args)

        def EnableArm(self, *args):
            raise AssertionError("unexpected arm enable")

        def JointCtrl(self, *args):
            raise AssertionError("unexpected arm target")

    class Vendor:
        class PiperRosNode:
            pass

    cls = driver_class(Vendor, None)
    node = cls.__new__(cls)
    node.piper = SDK()

    class Message:
        name = ["gripper"]
        position = [0.07]
        effort = [1.0]

    node.gripper_callback(Message())
    assert node.piper.calls == [(70000, 1000, 1, 0)]
    Message.position = [0.09]
    with pytest.raises(ValueError):
        node.gripper_callback(Message())
