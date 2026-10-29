from dataclasses import replace

import pytest

from robot_infra.contracts import canonical
from robot_infra.fake import FakeRobot, ManualClock
from robot_infra.runtime import Session


def rejected(s, intent, reason):
    before = s.driver_calls
    result = s.submit(intent)
    assert result["decision"] == "rejected"
    assert result["reason"] == reason
    assert s.driver_calls == before
    return result


def test_normal_chain_and_replay_provenance(session):
    lease = session.acquire("teleop")
    intent = session.intent("teleop", (0.2, -0.1), lease)
    receipt = session.submit(intent)
    assert receipt["status"] == "executed"
    assert session.positions == (0.2, -0.1)
    assert session.driver_calls == 1
    assert [e["kind"] for e in session.events[-6:]] == [
        "intent",
        "mapped",
        "guard",
        "command",
        "receipt",
        "feedback",
    ]
    assert session.events[receipt["command_event_id"]]["payload"]["intent"]["seq"] == intent.seq
    with pytest.raises(PermissionError):
        session.acquire("policy")


@pytest.mark.parametrize(
    "changes,reason",
    [
        ({"lease_token": None}, "no_valid_lease"),
        ({"source": "undeclared"}, "no_valid_lease"),
        ({"schema_version": "0.2"}, "schema_or_session_mismatch"),
        ({"session_id": "another-session"}, "schema_or_session_mismatch"),
        ({"unit": "deg"}, "action_semantics_mismatch"),
        ({"frame_id": "world"}, "action_semantics_mismatch"),
        ({"joint_names": ("joint_b", "joint_a")}, "action_semantics_mismatch"),
        ({"clock_domain": "wall"}, "action_semantics_mismatch"),
        ({"seq": 0}, "duplicate_or_out_of_order"),
        ({"seq": True}, "duplicate_or_out_of_order"),
        ({"created_ns": -1}, "not_due_or_expired"),
        ({"target_ns": 1}, "not_due_or_expired"),
        ({"expires_ns": 0}, "not_due_or_expired"),
        ({"created_ns": 0.0}, "invalid_time"),
        ({"positions": (0.1,)}, "dimension_mismatch"),
        ({"positions": None}, "dimension_mismatch"),
        ({"positions": (True, 0.0)}, "non_finite_or_invalid_position"),
        ({"positions": (float("nan"), 0.0)}, "non_finite_or_invalid_position"),
        ({"positions": (float("inf"), 0.0)}, "non_finite_or_invalid_position"),
        ({"positions": (-float("inf"), 0.0)}, "non_finite_or_invalid_position"),
        ({"positions": (1.001, 0.0)}, "joint_limit"),
    ],
)
def test_guard_boundaries_never_write(session, changes, reason):
    lease = session.acquire("teleop")
    intent = session.intent("teleop", (0.1, 0.1), lease)
    rejected(session, replace(intent, **changes), reason)
    canonical(session.events)  # Invalid raw numbers are preserved as tags in valid JSON.


def test_missing_revoked_duplicate_and_out_of_order(session):
    rejected(session, session.intent("teleop", (0.0, 0.0)), "no_valid_lease")
    lease = session.acquire("teleop")
    older = session.intent("teleop", (0.1, 0.1), lease)
    newer = session.intent("teleop", (0.2, 0.2), lease)
    session.submit(newer)
    rejected(session, older, "duplicate_or_out_of_order")
    rejected(session, newer, "duplicate_or_out_of_order")
    session.release(lease)
    rejected(session, session.intent("teleop", (0.0, 0.0), lease), "old_epoch")


def test_stale_state_action_age_and_lease_expiration(session):
    lease = session.acquire("teleop")
    old = session.intent("teleop", (0.1, 0.1), lease, valid_for_ns=10**10)
    session.clock.advance(session.config["timing"]["max_state_age_ns"] + 1)
    rejected(session, session.intent("teleop", (0.1, 0.1), lease), "stale_state")
    session.observe()
    session.clock.advance(session.config["timing"]["max_action_age_ns"] + 1)
    rejected(session, old, "action_too_old")
    session.clock.advance(lease.expires_ns - session.clock.now_ns)
    session.observe()
    rejected(session, session.intent("teleop", (0.1, 0.1), lease), "lease_expired")
    assert session.lease is None


def test_expiry_tick_faults_without_automatic_resume(session):
    lease = session.acquire("policy")
    request = session.request_policy(delay_ns=1)
    session.clock.advance(lease.expires_ns)
    session.tick()
    assert session.state == "faulted" and session.lease is None
    assert session.pending_count == 0
    assert session.complete_policy(request)["reason"] == "old_epoch"
    with pytest.raises(ValueError):
        session.start()
    session.reset()
    session.start()
    assert session.lease is None


def test_late_policy_after_takeover_cannot_revive_and_human_can_write(session):
    session.acquire("policy")
    request = session.request_policy(delay_ns=20_000_000)
    human = session.acquire("teleop", takeover=True)
    assert session.pending_count == 0
    with pytest.raises(PermissionError):
        session.acquire("policy", takeover=True)
    session.clock.advance(20_000_000)
    assert session.complete_policy(request)["reason"] == "old_epoch"
    assert session.driver_calls == 0
    session.observe()
    assert session.submit(session.intent("teleop", (0.1, 0.1), human))["status"] == "executed"


def test_policy_chunk_timing_and_queue_cancel(session):
    session.acquire("policy")
    request = session.request_policy(delay_ns=20_000_000)
    with pytest.raises(ValueError, match="not ready"):
        session.complete_policy(request)
    session.clock.advance(20_000_000)
    assert session.complete_policy(request)["count"] == 2
    session.tick()
    assert session.positions == (0.3, -0.1) and session.driver_calls == 1
    assert session.queued_count == 1
    session.acquire("teleop", takeover=True)
    assert session.queued_count == 0
    session.clock.advance(20_000_000)
    session.tick()
    assert session.driver_calls == 1


def test_complete_chunk_corresponds_to_request_and_snapshot(session):
    session.acquire("policy")
    request = session.request_policy()
    session.complete_policy(request)
    session.tick()
    session.clock.advance(20_000_000)
    session.tick()
    receipts = [e for e in session.events if e["kind"] == "receipt"]
    assert [r["payload"]["intent"]["action_index"] for r in receipts] == [0, 1]
    assert all(r["payload"]["intent"]["request_id"] == request.request_id for r in receipts)
    chunk = next(e for e in session.events if e["kind"] == "chunk")
    assert chunk["payload"]["snapshot_id"] == request.snapshot_id
    assert session.driver_calls == 2
    assert session.complete_policy(request)["reason"] == "cancelled_or_unknown_request"


@pytest.mark.parametrize("transition", ["stop", "fault", "reset"])
def test_transitions_invalidate_pending_and_queue(session, transition):
    session.acquire("policy")
    request = session.request_policy()
    session.complete_policy(request)
    pending = session.request_policy(delay_ns=1)
    getattr(session, transition)()
    assert session.pending_count == session.queued_count == 0
    session.clock.advance(1)
    assert session.complete_policy(pending)["reason"] == "old_epoch"
    session.tick()
    assert session.driver_calls == 0
    assert session.lease is None


def test_deadline_and_invalid_chunk_stop_remaining_actions(session):
    session.acquire("policy")
    r = session.request_policy(delay_ns=10, deadline_ns=10)
    session.clock.advance(10)
    assert session.complete_policy(r)["reason"] == "policy_deadline"
    assert session.driver_calls == 0

    class BadPolicy:
        from robot_infra.fake import StubPolicy

        artifact = StubPolicy.artifact

        def predict(self):
            return ((99.0, 0.0), (0.0, 0.0))

    r = session.request_policy(BadPolicy())
    session.complete_policy(r)
    session.tick()
    assert session.state == "faulted"
    assert session.queued_count == 0 and session.driver_calls == 0


def test_exclusive_fake_writer_and_clock():
    device = FakeRobot((0.0, 0.0), object())
    with pytest.raises(PermissionError):
        device._execute((0.1, 0.1), object())
    assert device.calls == 0
    with pytest.raises(ValueError):
        ManualClock().advance(-1)


def test_recording_failure_revokes_control(config):
    def sink(event):
        if event["kind"] == "command":
            raise OSError("injected disk full")

    s = Session(config, sink=sink)
    s.start()
    lease = s.acquire("teleop")
    with pytest.raises(OSError, match="disk full"):
        s.submit(s.intent("teleop", (0.1, 0.1), lease))
    assert s.state == "faulted" and s.lease is None and s.driver_calls == 0


def test_finalize_is_terminal_and_fault_outcome(session):
    session.fault()
    session.finalize()
    assert session.events[-1]["payload"]["outcome"] == "aborted"
    for operation in (session.start, session.reset, session.finalize):
        with pytest.raises(ValueError):
            operation()


def test_explicit_renew(session):
    lease = session.acquire("teleop")
    session.clock.advance(1)
    renewed = session.renew(lease)
    assert renewed.expires_ns == lease.expires_ns + 1 and renewed.epoch == lease.epoch
    with pytest.raises(PermissionError):
        session.release(lease)


@pytest.mark.parametrize("operation", ["acquire", "renew"])
def test_expired_owner_cannot_silently_rearm(session, operation):
    lease = session.acquire("teleop")
    session.clock.advance(lease.expires_ns)
    with pytest.raises(PermissionError, match="reset/start"):
        session.acquire("teleop") if operation == "acquire" else session.renew(lease)
    assert session.state == "faulted" and session.lease is None
    assert session.driver_calls == 0
