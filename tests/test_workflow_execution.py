import json
import threading
import time
from pathlib import Path

import pytest
from test_teleop_session import Backend, Model

from robot_infra.live_control import LiveGateway
from robot_infra.workflow import AsyncPolicySource, ExecutionSession, MockPolicy, ReplaySource
from robot_infra.workflow_runner import load_workflow, run_workflow

SPEC = {
    "joint_names": ["j1"],
    "base": "base",
    "tip": "tip",
    "joint_unit": "rad",
    "gripper_unit": "m",
}


def rig():
    c = json.loads((Path(__file__).parents[1] / "configs/teleop-session.example.json").read_text())
    c["model"]["joint_names"] = Model.names
    for arm in c["arms"].values():
        arm["home_rad"] = [0.0, 0.0]
    now = [10.0]
    backend = Backend()
    events = []
    gateway = LiveGateway(
        c, Model(), backend, lambda k, p: events.append((k, p)), clock=lambda: now[0]
    )

    def feedback(dt=0.02):
        now[0] += dt
        for s in c["arms"]:
            gateway.feedback(s, backend.q[s], backend.jaws[s], round(now[0] * 1e9))

    feedback()
    settings = {
        "alignment_duration_s": 0.1,
        "alignment_timeout_s": 1.0,
        "alignment_settle_s": 0.04,
        "alignment_tolerance_rad": 0.02,
        "max_tick_gap_s": 0.2,
    }
    initial = {s: {"q": [0.2, -0.2], "gripper_m": 0.0} for s in c["arms"]}
    source = ReplaySource(
        initial,
        [
            {
                "time_s": 0.1,
                "side": "left",
                "field": "q",
                "value": [0.4, -0.2],
                "evidence": {"test": True},
            }
        ],
        0.4,
    )
    session = ExecutionSession(gateway, source, settings, clock=lambda: now[0])
    return session, gateway, source, backend, now, events, feedback


def test_shared_replay_alignment_gripper_and_completion():
    s, g, source, b, now, events, step = rig()
    s.start()
    while s.state == "ALIGNING":
        step()
        s.tick()
    assert not any(c[0] == "jaw" for c in b.calls)  # Alignment preserves actual opening.
    for _ in range(25):
        step()
        s.tick()
    assert s.state == "COMPLETED" and g.owner is None
    assert b.q["left"] == (0.4, -0.2) and b.jaws == {"left": 0.0, "right": 0.0}
    old = len(b.calls)
    s.close()
    s.tick()
    assert len(b.calls) == old and source.closed
    assert any(k == "source_progress" for k, p in events)


@pytest.mark.parametrize("failure", ["stalled", "feedback", "epoch"])
def test_shared_stop_rejects_stall_stale_feedback_and_revocation(failure):
    s, g, source, b, now, events, step = rig()
    s.start()
    if failure == "stalled":
        step(0.3)
    elif failure == "feedback":
        now[0] += 0.15
        g.config["feedback_timeout_s"] = 0.05
    else:
        g.stop("takeover")
    s.tick()
    assert s.state == "FAULT" and not g.enabled
    old = len(b.calls)
    step()
    s.tick()
    assert len(b.calls) == old
    s.close()


def test_start_validation_precedes_enable_and_replay_refuses_catchup():
    s, g, source, b, now, events, step = rig()
    source.initial["left"]["q"] = [99.0, 0.0]
    with pytest.raises(ValueError, match="joint limits"):
        s.start()
    assert not b.calls
    source = ReplaySource({"custom": {}}, [], 1.0, max_lag_s=0.1)
    source.begin(1)
    source.sample(0.0, None)
    with pytest.raises(TimeoutError, match="stalled"):
        source.sample(0.2, None)


def snapshot(now):
    return {
        "time_ns": round(now * 1e9),
        "states": {"custom": {"q": [0.0]}},
        "images": {
            r: {"encoding": "rgb8", "shape": [1, 1, 3], "data": bytes([1, 2, 3])}
            for r in ("a", "b", "c")
        },
    }


def ready_result(source):
    deadline = time.monotonic() + 1.0
    while source.results.empty() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert not source.results.empty()


@pytest.mark.parametrize(
    "mode", ["normal", "late", "wrong_epoch", "exception", "nonfinite", "empty", "units"]
)
def test_async_mock_protocol_and_failure_admission(mode):
    now = [10.0]
    events = []
    initial = {"custom": {"q": [0.0], "gripper_m": 0.0}}

    class Policy(MockPolicy):
        def infer(self, request, cancel):
            result = super().infer(request, cancel)
            if mode == "wrong_epoch":
                result["epoch"] += 1
            if mode == "empty":
                result["actions"] = []
            if mode == "nonfinite":
                result["actions"][0]["time_s"] = float("nan")
            if mode == "units":
                result["action_spec"]["joint_unit"] = "degree"
            return result

    p = Policy(
        initial, action_spec=SPEC, delay_s=0.0, fail_request=1 if mode == "exception" else None
    )
    source = AsyncPolicySource(
        p, initial, 1.0, lambda k, p: events.append((k, p)), action_spec=SPEC, clock=lambda: now[0]
    )
    try:
        source.begin(3)
        source.sample(0.0, snapshot(now[0]))
        ready_result(source)
        now[0] += 0.4 if mode == "late" else 0.02
        if mode == "normal":
            assert source.sample(0.02, snapshot(now[0]))[0] is None
            now[0] += 0.08
            targets, _ = source.sample(0.1, snapshot(now[0]))
            assert targets["custom"]["q"][0] > 0
            prediction = next(p for k, p in events if k == "policy_prediction")
            assert len(prediction["input_image_sha256"]) == 3
        else:
            with pytest.raises((ValueError, RuntimeError, TimeoutError)):
                source.sample(0.02, snapshot(now[0]))
    finally:
        source.close()
    assert not source.thread.is_alive()


def test_pending_timeout_and_cooperative_cancel_do_not_block_control():
    now = [10.0]
    initial = {"custom": {"q": [0.0], "gripper_m": 0.0}}
    source = AsyncPolicySource(
        MockPolicy(initial, action_spec=SPEC, delay_s=10.0),
        initial,
        1.0,
        lambda k, p: None,
        action_spec=SPEC,
        clock=lambda: now[0],
    )
    source.begin(1)
    before = time.monotonic()
    source.sample(0.0, snapshot(now[0]))
    assert time.monotonic() - before < 0.1
    now[0] += 0.31
    with pytest.raises(TimeoutError, match="request timeout"):
        source.sample(0.31, snapshot(now[0]))
    source.close()
    assert not source.thread.is_alive()


def test_consumed_policy_queue_does_not_hold_last_target_forever():
    now = [10.0]
    initial = {"custom": {"q": [0.0], "gripper_m": 0.0}}

    class Policy(MockPolicy):
        def infer(self, request, cancel):
            if request["request_id"] > 1:
                cancel.wait(10.0)
                return None
            return super().infer(request, cancel)

    source = AsyncPolicySource(
        Policy(initial, action_spec=SPEC, steps=2, delay_s=0.0),
        initial,
        2.0,
        lambda k, p: None,
        action_spec=SPEC,
        deadline_s=1.0,
        clock=lambda: now[0],
    )
    try:
        source.begin(1)
        source.sample(0.0, snapshot(now[0]))
        ready_result(source)
        now[0] = 10.02
        source.sample(0.02, snapshot(now[0]))
        now[0] = 10.14
        assert source.sample(0.14, snapshot(now[0]))[0] is not None
        now[0] = 10.4
        with pytest.raises(TimeoutError, match="queue exhausted"):
            source.sample(0.4, snapshot(now[0]))
    finally:
        source.close()


def test_http_mock_transports_real_rgb_bytes_and_reports_model_errors():
    from robot_infra.mock_policy_server import HTTPMockClient, make_policy_server

    initial = {"custom": {"q": [0.0], "gripper_m": 0.0}}
    policy = MockPolicy(initial, action_spec=SPEC, delay_s=0.0, fail_request=2)
    server = make_policy_server(("127.0.0.1", 0), policy)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
    thread.start()
    try:
        client = HTTPMockClient("http://127.0.0.1:" + str(server.server_port))
        request = {"request_id": 1, "epoch": 3, "elapsed_s": 0.0, "observation": snapshot(10.0)}
        result = client.infer(request, threading.Event())
        assert (
            result["epoch"] == 3
            and len(result["input_image_sha256"]) == 3
            and result["action_spec"] == SPEC
        )
        request["request_id"] = 2
        with pytest.raises(RuntimeError, match="HTTP 422"):
            client.infer(request, threading.Event())
    finally:
        server.policy_cancel.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_episode_source_requires_confirmed_initial_and_same_model(tmp_path):
    from test_data_workflows import episode, profile

    root = episode(tmp_path / "episode")
    model = {"sha256": "testhash", "base": "base", "tip": "tip", "joint_names": ["j1", "j2"]}
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["context"]["config"]["model"] = model
    (root / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="unknown initial"):
        ReplaySource.from_episode(
            root, profile(), model, {"custom": {}}, start_s=0.0, duration_s=0.5
        )
    source = ReplaySource.from_episode(
        root, profile(), model, {"custom": {}}, start_s=0.1, duration_s=0.5
    )
    assert source.initial["custom"] == {"q": [0.4, 0.5], "gripper_m": 0.0}
    with pytest.raises(ValueError, match="semantics differ"):
        ReplaySource.from_episode(
            root,
            profile(),
            {**model, "sha256": "other"},
            {"custom": {}},
            start_s=0.1,
            duration_s=0.5,
        )


def test_mock_cli_binding_end_to_end_and_invalid_semantics(tmp_path):
    root = Path(__file__).parents[1]
    cfg = root / "configs/workflows/mock-inference.example.json"
    c = json.loads(cfg.read_text())
    c["graph"] = str(root / "configs/graphs/a4s-mock-inference.json")
    c["profile"] = str(root / "configs/teleop-session.example.json")
    c["source"]["duration_s"] = 0.25
    path = tmp_path / "workflow.json"
    path.write_text(json.dumps(c))
    report = run_workflow(path, tmp_path / "out")
    assert (
        report["state"] == "COMPLETED"
        and report["physical_commands"] is False
        and not report["cleanup_errors"]
    )
    events = [
        json.loads(x) for x in (tmp_path / "out/events/events.jsonl").read_text().splitlines()
    ]
    assert any(e["kind"] == "policy_prediction" for e in events)
    assert json.loads((tmp_path / "out/events/journal-summary.json").read_text())["error"] is None
    c["graph"] = str(root / "configs/graphs/a4s-inference.json")
    path.write_text(json.dumps(c))
    with pytest.raises(ValueError, match="joint_absolute"):
        load_workflow(path)
