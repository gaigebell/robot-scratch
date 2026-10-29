"""Single-threaded fake session. All command writes converge at _dispatch."""

import math
import uuid
from collections import deque
from dataclasses import replace

from . import CONTRACT_VERSION
from .contracts import JointIntent, Lease, PolicyRequest, digest, wire
from .fake import FakeCamera, FakeRobot, ManualClock, StubPolicy
from .graph import validate_graph


class Session:
    def __init__(self, config: dict, *, clock=None, session_id=None, sink=None):
        self.plan = validate_graph(config)  # Must precede any device construction.
        self.config = self.plan["config"]
        self.clock = clock or ManualClock()
        if self.clock.domain != self.config["timing"]["clock_domain"]:
            raise ValueError("clock domain mismatch")
        self.session_id = session_id or uuid.uuid4().hex
        self._sink = sink
        self.events = []
        self.state = "created"
        self.epoch = 0
        self.lease = None
        self._lease_counter = 0
        self._seq = {}
        self._last_accepted_seq = {}
        self._pending = {}
        self._queue = deque()
        self._request_counter = 0
        self._snapshot_counter = 0
        self._last_snapshot = None
        self._state_sample_ns = 0
        self._write_capability = object()
        self._robot = FakeRobot(self.config["asset"]["initial_positions"], self._write_capability)
        self._camera = FakeCamera()
        self._emit(
            "manifest",
            {
                "mode": "fake",
                "config": self.config,
                "config_hash": self.plan["config_hash"],
                "asset_hash": digest(self.config["asset"]),
                "policy_artifact": StubPolicy.artifact,
                "contract_subset": "M1-native-joint-position",
                "clock_domain": self.clock.domain,
            },
        )

    @property
    def driver_calls(self):
        return self._robot.calls

    @property
    def positions(self):
        return self._robot.positions

    @property
    def queued_count(self):
        return len(self._queue)

    @property
    def pending_count(self):
        return len(self._pending)

    def _emit(self, kind, payload):
        if self.state == "closed":
            raise ValueError("session finalized")
        event = {
            "schema_version": CONTRACT_VERSION,
            "session_id": self.session_id,
            "event_id": len(self.events),
            "epoch": self.epoch,
            "kind": kind,
            "time_ns": self.clock.now_ns,
            "clock_domain": self.clock.domain,
            "payload": wire(payload),
        }
        # Persist before publishing to consumers. A sink failure invalidates control.
        if self._sink is not None:
            try:
                self._sink(event)
            except Exception:
                self.epoch += 1
                self.lease = None
                self._pending.clear()
                self._queue.clear()
                self._robot.stop()
                self.state = "faulted"
                raise
        self.events.append(event)
        return event

    def start(self):
        if self.state not in ("created", "stopped"):
            raise ValueError("start requires created/stopped; fault requires reset first")
        validate_graph(self.config)
        self.state = "running"
        self._emit("lifecycle", {"state": self.state, "reason": "preflight_passed"})
        self.observe()

    def _invalidate(self, reason):
        old = self.epoch
        pending, queued = len(self._pending), len(self._queue)
        self.epoch += 1
        self.lease = None
        self._pending.clear()
        self._queue.clear()
        self._robot.stop()
        self._emit(
            "ownership",
            {
                "reason": reason,
                "old_epoch": old,
                "pending_cancelled": pending,
                "queued_cancelled": queued,
            },
        )

    def acquire(self, source: str, *, takeover=False):
        if self.state != "running" or source not in ("teleop", "policy"):
            raise ValueError("running session and a declared source required")
        if self.lease is not None:
            if self.clock.now_ns >= self.lease.expires_ns:
                self.fault("lease_expired")
                raise PermissionError("expired lease requires reset/start before acquiring control")
            elif source == self.lease.source:
                raise ValueError("owner already active; use explicit renew")
            elif source == "teleop" and takeover:
                self._invalidate("human_takeover")
            else:
                raise PermissionError("owner active; policy cannot preempt human")
        self._lease_counter += 1
        self.lease = Lease(
            f"fake-lease-{self._lease_counter}",
            self.session_id,
            source,
            self.epoch,
            self.clock.now_ns + self.config["timing"]["lease_ttl_ns"],
        )
        self._emit("ownership", {"reason": "granted", "lease": self.lease})
        return self.lease

    def renew(self, lease: Lease):
        if self.lease == lease and self.clock.now_ns >= lease.expires_ns:
            self.fault("lease_expired")
            raise PermissionError("expired lease requires reset/start")
        if self.state != "running" or self.lease != lease or self.clock.now_ns >= lease.expires_ns:
            raise PermissionError("cannot renew invalid/expired lease")
        self.lease = replace(
            lease, expires_ns=self.clock.now_ns + self.config["timing"]["lease_ttl_ns"]
        )
        self._emit("ownership", {"reason": "renewed", "lease": self.lease})
        return self.lease

    def release(self, lease: Lease):
        if self.state != "running" or self.lease != lease:
            raise PermissionError("cannot release another owner's lease")
        self._invalidate("released")

    def stop(self, reason="user_stop"):
        if self.state not in ("running", "created"):
            return
        self.state = "stopped"
        self._invalidate(reason)
        self._emit("lifecycle", {"state": self.state, "reason": reason})

    def fault(self, reason="injected_fault"):
        if self.state == "closed":
            raise ValueError("session closed")
        self.state = "faulted"
        self._invalidate(reason)
        self._emit("lifecycle", {"state": self.state, "reason": reason})

    def reset(self):
        if self.state == "closed":
            raise ValueError("session closed")
        self.state = "created"
        self._invalidate("reset")
        self._last_snapshot = None
        self._emit(
            "lifecycle", {"state": self.state, "reason": "reset_requires_start_and_new_lease"}
        )

    def observe(self):
        if self.state != "running":
            raise ValueError("observations require running session")
        now = self.clock.now_ns
        self._state_sample_ns = now
        self._snapshot_counter += 1
        state_event = self._emit(
            "state",
            {
                "positions": self.positions,
                "joint_names": self.config["asset"]["joint_names"],
                "unit": "rad",
                "frame_id": "fake_base",
                "device_time_ns": now,
                "host_monotonic_ns": now,
                "receive_ns": now,
                "clock_domain": self.clock.domain,
            },
        )
        camera_event = self._emit(
            "camera", self._camera.sample(now, digest(self.config["asset"]["calibration"]))
        )
        self._last_snapshot = {
            "snapshot_id": f"snapshot-{self._snapshot_counter}",
            "created_ns": now,
            "epoch": self.epoch,
            "state_event_id": state_event["event_id"],
            "camera_event_id": camera_event["event_id"],
            "valid": True,
        }
        self._emit("snapshot", self._last_snapshot)
        return dict(self._last_snapshot)

    def intent(self, source, positions, lease=None, *, valid_for_ns=None):
        self._seq[source] = self._seq.get(source, 0) + 1
        now = self.clock.now_ns
        return JointIntent(
            self.session_id,
            source,
            self._seq[source],
            lease.epoch if lease else self.epoch,
            lease.token if lease else None,
            tuple(positions),
            tuple(self.config["asset"]["joint_names"]),
            "rad",
            "fake_base",
            self.clock.domain,
            now,
            now,
            now
            + (
                self.config["timing"]["max_action_age_ns"] if valid_for_ns is None else valid_for_ns
            ),
        )

    def _reject(self, reason, intent=None, **extra):
        return self._emit(
            "guard", {"decision": "rejected", "reason": reason, "intent": intent, **extra}
        )["payload"]

    def submit(self, intent: JointIntent):
        raw = self._emit("intent", {"intent": intent})
        # Explicit identity map for this one action space, never an inferred transform.
        self._emit(
            "mapped",
            {"intent_event_id": raw["event_id"], "mapper": "joint-identity-v1", "intent": intent},
        )
        return self._dispatch(intent)

    def _dispatch(self, intent):
        now = self.clock.now_ns
        asset, timing = self.config["asset"], self.config["timing"]
        reason = None
        if self.state != "running":
            reason = "not_running"
        elif intent.schema_version != CONTRACT_VERSION or intent.session_id != self.session_id:
            reason = "schema_or_session_mismatch"
        elif type(intent.epoch) is not int or intent.epoch != self.epoch:
            reason = "old_epoch"
        elif (
            self.lease is None
            or intent.lease_token != self.lease.token
            or intent.source != self.lease.source
        ):
            reason = "no_valid_lease"
        elif now >= self.lease.expires_ns:
            self.fault("lease_expired")
            reason = "lease_expired"
        elif (
            intent.clock_domain != self.clock.domain
            or intent.unit != asset["unit"]
            or intent.frame_id != asset["frame_id"]
            or list(intent.joint_names) != asset["joint_names"]
        ):
            reason = "action_semantics_mismatch"
        elif type(intent.seq) is not int or intent.seq <= self._last_accepted_seq.get(
            (intent.source, intent.epoch), 0
        ):
            reason = "duplicate_or_out_of_order"
        elif not all(
            type(t) is int for t in (intent.created_ns, intent.target_ns, intent.expires_ns)
        ):
            reason = "invalid_time"
        elif not 0 <= intent.created_ns <= intent.target_ns <= now < intent.expires_ns:
            reason = "not_due_or_expired"
        elif now - intent.created_ns > timing["max_action_age_ns"]:
            reason = "action_too_old"
        elif now - self._state_sample_ns > timing["max_state_age_ns"]:
            reason = "stale_state"
        elif not isinstance(intent.positions, (tuple, list)) or len(intent.positions) != len(
            asset["joint_names"]
        ):
            reason = "dimension_mismatch"
        elif any(type(p) not in (int, float) or not math.isfinite(p) for p in intent.positions):
            reason = "non_finite_or_invalid_position"
        elif any(not low <= p <= high for p, (low, high) in zip(intent.positions, asset["limits"])):
            reason = "joint_limit"
        if reason:
            return self._reject(reason, intent)
        guard = self._emit("guard", {"decision": "accepted", "intent": intent})
        self._last_accepted_seq[(intent.source, intent.epoch)] = intent.seq
        # Audit command before write; receipt after synchronous fake execution.
        command = self._emit(
            "command",
            {"intent": intent, "status": "scheduled", "guard_event_id": guard["event_id"]},
        )
        self._robot._execute(intent.positions, self._write_capability)
        self._state_sample_ns = now
        receipt = self._emit(
            "receipt",
            {
                "intent": intent,
                "command_event_id": command["event_id"],
                "status": "executed",
                "sent_ns": now,
                "acknowledged_ns": now,
                "executed_ns": now,
                "device": "fake_robot",
            },
        )
        self._emit(
            "feedback",
            {
                "receipt_event_id": receipt["event_id"],
                "positions": self.positions,
                "device_time_ns": now,
                "host_monotonic_ns": now,
                "receive_ns": now,
                "clock_domain": self.clock.domain,
            },
        )
        return receipt["payload"]

    def request_policy(self, policy=None, *, delay_ns=0, deadline_ns=None):
        if self.state != "running" or self.lease is None or self.lease.source != "policy":
            raise PermissionError("policy lease required")
        if type(delay_ns) is not int or delay_ns < 0:
            raise ValueError("delay must be nonnegative integer nanoseconds")
        if deadline_ns is not None and (
            type(deadline_ns) is not int or deadline_ns <= self.clock.now_ns
        ):
            raise ValueError("deadline must be in the future")
        snapshot = self.observe()
        policy = policy or StubPolicy()
        for key in ("unit", "frame_id", "joint_names"):
            if policy.artifact.get(key) != self.config["asset"][key]:
                raise ValueError(f"policy artifact {key} mismatch")
        if self._pending:
            raise ValueError("M1 permits only one in-flight request")
        self._request_counter += 1
        rid = f"request-{self._request_counter}"
        request = PolicyRequest(
            rid,
            snapshot["snapshot_id"],
            self.lease,
            self.clock.now_ns,
            self.clock.now_ns + delay_ns,
            deadline_ns
            if deadline_ns is not None
            else self.clock.now_ns + self.config["timing"]["max_action_age_ns"],
            tuple(tuple(p) for p in policy.predict()),
            self.config["timing"]["period_ns"],
        )
        self._pending[rid] = request
        self._emit("policy_request", {"request": request, "artifact_hash": digest(policy.artifact)})
        return request

    def complete_policy(self, request: PolicyRequest):
        if self.clock.now_ns < request.ready_ns:
            raise ValueError("controlled result not ready yet")
        self._emit("policy_output", {"request": request, "generated_ns": self.clock.now_ns})
        known = self._pending.pop(request.request_id, None)
        if request.lease.epoch != self.epoch:
            return self._reject("old_epoch", request_id=request.request_id)
        if self.state != "running" or known != request:
            return self._reject("cancelled_or_unknown_request", request_id=request.request_id)
        if self.clock.now_ns >= request.deadline_ns:
            return self._reject("policy_deadline", request_id=request.request_id)
        chunk_id = f"chunk-{request.request_id}"
        actions = []
        for index, positions in enumerate(request.positions):
            intent = self.intent("policy", positions, request.lease)
            intent = replace(
                intent,
                created_ns=request.created_ns,
                target_ns=self.clock.now_ns + index * request.dt_ns,
                expires_ns=request.deadline_ns,
                request_id=request.request_id,
                chunk_id=chunk_id,
                action_index=index,
            )
            actions.append(intent)
        # One request at a time in M1; no silent overlapping action fusion.
        if self._queue:
            return self._reject("queue_busy", request_id=request.request_id)
        self._queue.extend(actions)
        self._emit(
            "chunk",
            {
                "request_id": request.request_id,
                "snapshot_id": request.snapshot_id,
                "chunk_id": chunk_id,
                "actions": actions,
                "profile": "native",
            },
        )
        return {"decision": "queued", "count": len(actions)}

    def tick(self):
        if self.state != "running":
            return
        if self.lease and self.clock.now_ns >= self.lease.expires_ns:
            self.fault("lease_expired")
            return
        self.observe()
        while self._queue and self._queue[0].target_ns <= self.clock.now_ns:
            intent = self._queue.popleft()
            self._emit("scheduled", {"intent": intent})
            result = self.submit(intent)
            if result.get("decision") == "rejected":
                self.fault("guard_rejected_chunk")
                break

    def finalize(self):
        if self.state == "closed":
            raise ValueError("already finalized")
        outcome = "aborted" if self.state == "faulted" else "complete"
        if self.state in ("running", "created"):
            self.stop("finalize")
        self._emit(
            "finalize",
            {
                "outcome": outcome,
                "event_count": len(self.events),
                "events_hash": digest(self.events),
                "driver_calls": self.driver_calls,
            },
        )
        self.state = "closed"
