"""One-group, N-endpoint FAKE execution. Real watchdog/ROS/IK are not provided."""

import uuid
from collections import Counter, OrderedDict
from dataclasses import replace

from .collection import EVENT_VERSION, CommandSet, EndpointTarget, _finite, validate_collection
from .contracts import Lease, wire
from .fake import ManualClock


class FakeBatchBackend:
    """A physical-I/O grouping analogue: e.g. arm and gripper in one message."""

    simulation = True

    def __init__(self, endpoints, capability):
        self.values = {e["id"]: tuple(e["initial"]) for e in endpoints}
        self._capability = capability
        self.calls = self.stop_calls = 0

    def read(self, now_ns):
        return {"positions": dict(self.values), "received_ns": now_ns, "device_time_ns": now_ns}

    def send(self, targets, capability):
        if capability is not self._capability or set(targets) != set(self.values):
            raise PermissionError("exclusive full-batch writer required")
        self.values = {k: tuple(v) for k, v in targets.items()}
        self.calls += 1
        return "executed_fake"

    def stop(self):
        self.stop_calls += 1


class CollectionSession:
    def __init__(
        self, config, *, sink=None, clock=None, session_id=None, backend_factory=FakeBatchBackend
    ):
        self.plan = validate_collection(config)  # Before factory/any device construction.
        self.config = self.plan["config"]
        self.clock = clock or ManualClock()
        if self.clock.domain != self.config["clock_domain"]:
            raise ValueError("clock domain mismatch")
        self.session_id = session_id or uuid.uuid4().hex
        self.sink, self.state, self.epoch, self.lease = sink, "created", 0, None
        self.event_count, self.counts = 0, Counter()
        self._capability, self._seq, self._accepted = object(), {}, {}
        self._snapshots, self._states = OrderedDict(), {}
        self._snapshot_count = self._lease_count = 0
        self._started = self._had_fault = False
        self.backends = {}
        for backend, members in self.plan["batches"].items():
            instance = backend_factory(
                [self.plan["endpoints"][e] for e in members], self._capability
            )
            if not isinstance(instance, FakeBatchBackend) or instance.simulation is not True:
                raise ValueError("only fake batch backends allowed in this slice")
            self.backends[backend] = instance
        self.emit(
            "manifest",
            {
                "mode": "fake",
                "config": self.config,
                "config_hash": self.plan["config_hash"],
                "execution": "synthetic_joint_targets; no Cartesian mapper or IK",
            },
        )

    def emit(self, kind, payload):
        if self.state == "closed":
            raise ValueError("session closed")
        event = {
            "schema_version": EVENT_VERSION,
            "session_id": self.session_id,
            "event_id": self.event_count,
            "kind": kind,
            "time_ns": self.clock.now_ns,
            "clock_domain": self.clock.domain,
            "epoch": self.epoch,
            "payload": wire(payload),
        }
        try:
            if self.sink:
                self.sink(event)
        except Exception:
            self.epoch += 1
            self.lease = None
            self.state = "faulted"
            self._had_fault = True
            for backend in self.backends.values():
                backend.stop()
            raise
        self.event_count += 1
        self.counts[kind] += 1
        return event

    def start(self):
        if self.state not in ("created", "stopped"):
            raise ValueError("start requires created/stopped; fault requires reset")
        self.state = "running"
        self._started = True
        self.emit("lifecycle", {"state": self.state})
        return self.observe()

    def _invalidate(self, reason):
        self.epoch += 1
        self.lease = None
        self._snapshots.clear()
        self._accepted.clear()
        for backend in self.backends.values():
            backend.stop()
        self.emit("ownership", {"reason": reason})

    def acquire(self, source, *, takeover=False):
        if self.state != "running" or source not in self.plan["sources"]:
            raise ValueError("running session and declared source required")
        self.watchdog_tick()
        if self.state != "running":
            raise PermissionError("expired owner requires reset/start")
        if self.lease:
            if (
                not takeover
                or self.plan["sources"][source] <= self.plan["sources"][self.lease.source]
            ):
                raise PermissionError(
                    "owner active; only higher-priority explicit takeover allowed"
                )
            self._invalidate("source_takeover")
        self._lease_count += 1
        self.lease = Lease(
            f"fake-group-{self._lease_count}",
            self.session_id,
            source,
            self.epoch,
            self.clock.now_ns + self.config["timing"]["lease_ttl_ns"],
        )
        self.emit(
            "ownership",
            {"reason": "granted", "group": self.config["group"]["id"], "lease": self.lease},
        )
        return self.lease

    def renew(self, lease):
        self.watchdog_tick()
        if self.state != "running" or self.lease != lease:
            raise PermissionError("invalid lease")
        self.lease = replace(
            lease, expires_ns=self.clock.now_ns + self.config["timing"]["lease_ttl_ns"]
        )
        self.emit("ownership", {"reason": "renewed", "lease": self.lease})
        return self.lease

    def release(self, lease):
        if self.state != "running" or lease != self.lease:
            raise PermissionError("invalid lease")
        self._invalidate("released")

    def stop(self, reason="user_stop"):
        if self.state == "closed":
            raise ValueError("session closed")
        if self.state == "faulted":
            return  # Do not relabel a fault as a successful stop.
        self.state = "stopped"
        self._invalidate(reason)
        self.emit("lifecycle", {"state": self.state, "reason": reason})

    def fault(self, reason):
        if self.state == "closed":
            raise ValueError("session closed")
        self.state = "faulted"
        self._had_fault = True
        self._invalidate(reason)
        self.emit("lifecycle", {"state": self.state, "reason": reason})

    def reset(self):
        if self.state == "closed":
            raise ValueError("session closed")
        self.state = "created"
        self._invalidate("reset_requires_start_and_new_owner")

    def watchdog_tick(self):
        """Explicit fake polling only; never a claimed real-time watchdog."""
        if self.state == "running" and self.lease and self.clock.now_ns >= self.lease.expires_ns:
            self.fault("lease_expired")

    def observe(self):
        if self.state != "running":
            raise ValueError("observations require running")
        states, cameras = {}, {}
        for backend_id, backend in self.backends.items():
            sample = backend.read(self.clock.now_ns)
            event = self.emit("state", {"backend": backend_id, **sample})
            for endpoint, values in sample["positions"].items():
                self._states[endpoint] = (values, sample["received_ns"])
            states[backend_id] = event["event_id"]
        for role in self.config["cameras"]:
            value = self._snapshot_count % 256
            event = self.emit(
                "camera",
                {
                    "role": role,
                    "encoding": "rgb8",
                    "width": 2,
                    "height": 2,
                    "data_hex": bytes([value, 0, 0] * 4).hex(),
                    "synthetic": True,
                    "device_time_ns": self.clock.now_ns,
                    "received_ns": self.clock.now_ns,
                },
            )
            cameras[role] = event["event_id"]
        self._snapshot_count += 1
        snapshot = {
            "snapshot_id": f"snapshot-{self._snapshot_count}",
            "created_ns": self.clock.now_ns,
            "epoch": self.epoch,
            "state_event_ids": states,
            "camera_event_ids": cameras,
        }
        self._snapshots[snapshot["snapshot_id"]] = snapshot
        if len(self._snapshots) > 32:
            self._snapshots.popitem(last=False)
        self.emit("snapshot", snapshot)
        return dict(snapshot)

    def intent(self, source, values, lease, snapshot):
        self._seq[source] = self._seq.get(source, 0) + 1
        targets = tuple(
            EndpointTarget(
                k,
                tuple(self.plan["endpoints"][k]["names"]),
                tuple(v),
                self.plan["endpoints"][k]["unit"],
                self.plan["endpoints"][k]["frame_id"],
            )
            for k, v in values.items()
        )
        return CommandSet(
            self.session_id,
            source,
            self._seq[source],
            lease.epoch,
            lease.token,
            snapshot["snapshot_id"],
            snapshot["created_ns"],
            self.clock.now_ns,
            snapshot["created_ns"] + self.config["timing"]["max_action_age_ns"],
            targets,
        )

    def _reason(self, intent, *, check_sequence=True):
        now = self.clock.now_ns
        if not isinstance(intent, CommandSet):
            return "invalid_command_set"
        if self.state != "running":
            return "not_running"
        if intent.schema_version != EVENT_VERSION or intent.session_id != self.session_id:
            return "schema_or_session_mismatch"
        if type(intent.epoch) is not int or intent.epoch != self.epoch:
            return "old_epoch"
        if not self.lease or (intent.lease_token, intent.source) != (
            self.lease.token,
            self.lease.source,
        ):
            return "no_valid_lease"
        if now >= self.lease.expires_ns:
            return "lease_expired"
        if type(intent.seq) is not int or (
            check_sequence and intent.seq <= self._accepted.get((intent.source, intent.epoch), 0)
        ):
            return "sequence"
        if not all(
            type(x) is int for x in (intent.created_ns, intent.target_ns, intent.expires_ns)
        ):
            return "invalid_time"
        if not 0 <= intent.created_ns <= intent.target_ns <= now < intent.expires_ns:
            return "not_due_or_expired"
        if now - intent.created_ns > self.config["timing"]["max_action_age_ns"]:
            return "action_too_old"
        if not isinstance(intent.snapshot_id, str):
            return "unknown_snapshot"
        snapshot = self._snapshots.get(intent.snapshot_id)
        if (
            not snapshot
            or snapshot["epoch"] != intent.epoch
            or snapshot["created_ns"] != intent.created_ns
        ):
            return "unknown_snapshot"
        if not isinstance(intent.targets, (tuple, list)) or any(
            not isinstance(t, EndpointTarget) for t in intent.targets
        ):
            return "invalid_targets"
        ids = [t.endpoint_id for t in intent.targets]
        if any(not isinstance(x, str) for x in ids):
            return "invalid_target_id"
        if len(ids) != len(set(ids)) or set(ids) != set(self.plan["endpoints"]):
            return "incomplete_or_duplicate_group"
        for target in intent.targets:
            spec = self.plan["endpoints"][target.endpoint_id]
            if (target.names, target.unit, target.frame_id) != (
                tuple(spec["names"]),
                spec["unit"],
                spec["frame_id"],
            ):
                return "endpoint_semantics"
            if not isinstance(target.positions, (tuple, list)) or len(target.positions) != len(
                spec["names"]
            ):
                return "dimension"
            if any(not _finite(x) for x in target.positions):
                return "non_finite_position"
            if any(
                not low <= x <= high for x, (low, high) in zip(target.positions, spec["limits"])
            ):
                return "limit"
            state = self._states.get(target.endpoint_id)
            if (
                state is None
                or not 0 <= now - state[1] <= self.config["timing"]["max_state_age_ns"]
            ):
                return "stale_feedback"
            if any(
                abs(x - old) > spec["max_step"] + 1e-12
                for x, old in zip(target.positions, state[0])
            ):
                return "max_step"
        return None

    def submit(self, intent):
        self.emit("intent", {"intent": intent})
        self.emit("motion_result", {"profile": "synthetic-native-joint-identity", "intent": intent})
        reason = self._reason(intent)
        guard = self.emit(
            "guard",
            {"decision": "rejected" if reason else "accepted", "reason": reason, "intent": intent},
        )
        if reason:
            if reason == "lease_expired":
                self.fault(reason)
            return {"decision": "rejected", "reason": reason}
        self._accepted[intent.source, intent.epoch] = intent.seq
        targets = {t.endpoint_id: t.positions for t in intent.targets}
        receipts = []
        for backend_id, members in self.plan["batches"].items():
            # A previous send/read can take time: recheck at each actual write.
            reason = self._reason(intent, check_sequence=False)
            guard = self.emit(
                "guard",
                {
                    "decision": "rejected" if reason else "accepted",
                    "reason": reason,
                    "intent": intent,
                },
            )
            if reason:
                self.fault("pre_send_" + reason)
                return {"decision": "faulted", "reason": reason, "receipts": receipts}
            batch = {e: targets[e] for e in members}
            command = self.emit(
                "command",
                {"backend": backend_id, "targets": batch, "guard_event_id": guard["event_id"]},
            )
            try:
                status = self.backends[backend_id].send(batch, self._capability)
                if status != "executed_fake":
                    raise ValueError("fake backend returned unsupported execution status")
            except Exception as exc:
                receipt = self.emit(
                    "receipt",
                    {
                        "backend": backend_id,
                        "command_event_id": command["event_id"],
                        "status": "failed_execution_unknown",
                        "error": str(exc),
                    },
                )
                receipts.append(receipt["payload"])
                self.fault("partial_batch_failure")
                return {"decision": "faulted", "receipts": receipts}
            receipt = self.emit(
                "receipt",
                {
                    "backend": backend_id,
                    "command_event_id": command["event_id"],
                    "status": status,
                    "sent_ns": self.clock.now_ns,
                },
            )
            receipts.append(receipt["payload"])
            try:
                feedback = self.backends[backend_id].read(self.clock.now_ns)
            except Exception:
                self.fault("feedback_read_failed_after_send")
                raise
            self.emit(
                "feedback",
                {"backend": backend_id, "receipt_event_id": receipt["event_id"], **feedback},
            )
            for endpoint, values in feedback["positions"].items():
                self._states[endpoint] = (values, feedback["received_ns"])
        return {"decision": "executed_fake", "receipts": receipts}

    def finish(self):
        if self.state == "closed":
            raise ValueError("session closed")
        if not self._started:
            raise ValueError("cannot finalize an unstarted collection session")
        outcome = "aborted" if self._had_fault else "complete"
        if self.state not in ("stopped", "faulted"):
            self.stop("finalize")
        self.state = "closed"
        return {
            "outcome": outcome,
            "events_before_finalize": self.event_count,
            "counts": dict(self.counts),
            "backend_calls": {b: x.calls for b, x in self.backends.items()},
        }
