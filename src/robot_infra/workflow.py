"""Native action sources and a shared execution session, independent of ROS/VR."""

import hashlib
import math
import queue
import threading
import time
from copy import deepcopy

from .episode_reader import episode_manifest
from .training_view import command_records, journal_command_records, published_targets


def finite(value, name, low=0.0, inclusive=False):
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or (value < low if inclusive else value <= low)
    ):
        raise ValueError("invalid " + name)
    return value


def checked_targets(targets, gateway):
    if not isinstance(targets, dict) or set(targets) != set(gateway.config["arms"]):
        raise ValueError("action arm set mismatch")
    for side, value in targets.items():
        if (
            set(value) != {"q", "gripper_m"}
            or not isinstance(value["q"], (list, tuple))
            or len(value["q"]) != len(gateway.model.names)
        ):
            raise ValueError("native target shape mismatch")
        if any(
            type(x) not in (int, float) or not math.isfinite(x)
            for x in (*value["q"], value["gripper_m"])
        ):
            raise ValueError("nonfinite action")
        if any(not lo <= x <= hi for x, (lo, hi) in zip(value["q"], gateway.model.limits)):
            raise ValueError("source target outside joint limits")
        lo, hi = gateway.config["arms"][side]["gripper"]["limits_m"]
        if not lo <= value["gripper_m"] <= hi:
            raise ValueError("source target outside gripper limits")
    return deepcopy(targets)


class ReplaySource:
    """Original-time confirmed targets, rebased only at a declared clip boundary."""

    def __init__(self, initial, events, duration_s, *, max_lag_s=0.15, provenance=None):
        self.initial = deepcopy(initial)
        self.events = sorted(events, key=lambda e: e["time_s"])
        self.duration_s = finite(duration_s, "replay duration")
        self.max_lag_s = finite(max_lag_s, "replay max lag")
        self.provenance = provenance or {}
        self.index = 0
        self.targets = deepcopy(initial)
        self.last_elapsed = -1.0
        self.closed = False
        if any(
            not math.isfinite(e["time_s"]) or not 0 <= e["time_s"] <= duration_s
            for e in self.events
        ):
            raise ValueError("invalid replay event time")

    @classmethod
    def from_episode(
        cls,
        root,
        profile,
        model,
        arms,
        *,
        start_s,
        duration_s,
        speed=1.0,
        journal_paths=(),
        max_lag_s=0.15,
    ):
        from .training_view import validate_training_profile

        root, m = episode_manifest(root)
        c = validate_training_profile(profile)
        if m["status"] != "saved" or not m.get("qc_passed"):
            raise ValueError("physical replay requires saved QC-passing episode")
        with (root / "streams.mcap").open("rb") as f:
            if hashlib.file_digest(f, "sha256").hexdigest() != m["mcap_sha256"]:
                raise ValueError("source MCAP hash mismatch")
        recorded = m.get("context", {}).get("config", {}).get("model", {})
        if any(recorded.get(k) != model[k] for k in ("sha256", "base", "tip", "joint_names")):
            raise ValueError("recorded/runtime model semantics differ")
        expected = {(s, field) for s in arms for field in ("q", "gripper_m")}
        if {(e["side"], e["command_field"]) for e in c["endpoints"]} != expected or len(
            c["endpoints"]
        ) != len(expected):
            raise ValueError("explicit arm/gripper endpoint mapping required")
        for e in c["endpoints"]:
            if e["command_field"] == "q" and e["names"] != model["joint_names"]:
                raise ValueError("recorded joint order mismatch")
        finite(start_s, "clip start", inclusive=True)
        finite(duration_s, "clip duration")
        finite(speed, "replay speed")
        if start_s + duration_s > (m["end_monotonic_ns"] - m["start_monotonic_ns"]) / 1e9:
            raise ValueError("clip exceeds episode")
        targets = published_targets(command_records(root), c["endpoints"])
        seeds = (
            published_targets(journal_command_records(journal_paths), c["endpoints"])
            if journal_paths
            else {}
        )
        start = m["start_monotonic_ns"] + round(start_s * 1e9)
        end = start + round(duration_s * 1e9)
        initial = {s: {} for s in arms}
        events = []
        for e in c["endpoints"]:
            name, side, field = e["id"], e["side"], e["command_field"]
            rows = list(targets.get(name, []))
            candidates = [r for r in seeds.get(name, []) if r["time_ns"] <= start]
            value = m.get("context", {}).get("initial_targets", {}).get(side, {}).get(field)
            if value and value["time_ns"] <= start:
                candidates.append(
                    {
                        "time_ns": value["time_ns"],
                        "values": value["values"],
                        "evidence": {"kind": "manifest_initial_target"},
                    }
                )
            candidates += [r for r in rows if r["time_ns"] <= start]
            if not candidates:
                raise ValueError(
                    "unknown initial target: "
                    + name
                    + "; provide prior journal or choose later clip"
                )
            seed = max(candidates, key=lambda r: r["time_ns"])
            initial[side][field] = seed["values"] if field == "q" else seed["values"][0]
            for row in rows:
                if start < row["time_ns"] < end:
                    events.append(
                        {
                            "time_s": (row["time_ns"] - start) / 1e9 / speed,
                            "side": side,
                            "field": field,
                            "value": row["values"] if field == "q" else row["values"][0],
                            "evidence": row["evidence"],
                        }
                    )
        provenance = {
            "episode": str(root),
            "mcap_sha256": m["mcap_sha256"],
            "start_s": start_s,
            "source_duration_s": duration_s,
            "speed": speed,
            "initial_targets": initial,
            "journal_paths": [str(p) for p in journal_paths],
            "model": recorded,
        }
        return cls(initial, events, duration_s / speed, max_lag_s=max_lag_s, provenance=provenance)

    def begin(self, epoch):
        self.epoch = epoch

    def sample(self, elapsed_s, snapshot):
        if self.closed or elapsed_s < self.last_elapsed:
            raise ValueError("replay requires an active monotonic clock")
        if self.last_elapsed >= 0 and elapsed_s - self.last_elapsed > self.max_lag_s:
            raise TimeoutError("replay scheduler stalled; refusing catch-up burst")
        self.last_elapsed = elapsed_s
        due = []
        while self.index < len(self.events) and self.events[self.index]["time_s"] <= elapsed_s:
            event = self.events[self.index]
            if elapsed_s - event["time_s"] > self.max_lag_s:
                raise TimeoutError("expired replay target")
            self.targets[event["side"]][event["field"]] = deepcopy(event["value"])
            due.append(event)
            self.index += 1
        return deepcopy(self.targets), due

    def close(self):
        self.closed = True


class MockPolicy:
    """Joint-position mock consuming actual snapshot pixels and measured state."""

    def __init__(
        self,
        initial,
        *,
        action_spec=None,
        steps=12,
        action_dt_s=0.05,
        lead_s=0.08,
        delay_s=0.01,
        amplitude_rad=0.02,
        fail_request=None,
    ):
        self.action_spec = deepcopy(action_spec)
        self.initial = deepcopy(initial)
        self.steps = steps
        self.dt = finite(action_dt_s, "action dt")
        self.lead = finite(lead_s, "policy lead")
        self.delay = finite(delay_s, "mock delay", inclusive=True)
        self.amplitude = finite(amplitude_rad, "mock amplitude", inclusive=True)
        self.fail_request = fail_request
        if type(steps) is not int or not 2 <= steps <= 1000:
            raise ValueError("invalid chunk size")

    def infer(self, request, cancel):
        if cancel.wait(self.delay):
            return None
        if request["request_id"] == self.fail_request:
            raise RuntimeError("injected mock model error")
        observation = request["observation"]
        if not observation["states"] or len(observation["images"]) != 3:
            raise ValueError("mock requires measured state and three RGB images")
        image_hashes = {
            role: hashlib.sha256(v["data"]).hexdigest() for role, v in observation["images"].items()
        }
        for value in observation["images"].values():
            if value["encoding"] != "rgb8" or len(value["data"]) != math.prod(value["shape"]):
                raise ValueError("invalid RGB snapshot")
        actions = []
        for i in range(self.steps):
            stamp = request["elapsed_s"] + self.lead + i * self.dt
            targets = deepcopy(self.initial)
            for side in targets:
                targets[side]["q"][0] += self.amplitude * math.sin(2 * math.pi * stamp / 4)
            actions.append({"time_s": stamp, "targets": targets})
        return {
            "schema_version": "policy-chunk/1",
            "request_id": request["request_id"],
            "epoch": request["epoch"],
            "observation_ns": observation["time_ns"],
            "action_encoding": "joint_absolute",
            "actions": actions,
            "input_image_sha256": image_hashes,
            "action_spec": self.action_spec,
        }


class AsyncPolicySource:
    """One bounded background inference request. Control/watchdog never waits."""

    def __init__(
        self,
        policy,
        initial,
        duration_s,
        emit,
        *,
        action_spec,
        deadline_s=0.3,
        observation_max_age_s=0.4,
        max_gap_s=0.15,
        refill_horizon_s=0.3,
        clock=time.monotonic,
    ):
        self.action_spec = deepcopy(action_spec)
        self.policy, self.initial, self.duration_s, self.emit, self.clock = (
            policy,
            deepcopy(initial),
            finite(duration_s, "policy duration"),
            emit,
            clock,
        )
        self.deadline = finite(deadline_s, "inference deadline")
        self.max_age = finite(observation_max_age_s, "observation max age")
        self.max_gap = finite(max_gap_s, "queue max gap")
        self.refill = finite(refill_horizon_s, "refill horizon")
        self.requests = queue.Queue(1)
        self.results = queue.Queue(1)
        self.cancel = threading.Event()
        self.pending = None
        self.request_id = 0
        self.epoch = None
        self.actions = []
        self.last_applied = None
        self.thread = threading.Thread(target=self._work, name="policy-worker", daemon=True)
        self.thread.start()

    def _work(self):
        while not self.cancel.is_set():
            try:
                request = self.requests.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                result = self.policy.infer(request, self.cancel)
                error = None
            except Exception as exc:
                result = None
                error = str(exc)
            if not self.cancel.is_set():
                self.results.put((request, result, error))

    def begin(self, epoch):
        self.epoch = epoch

    def sample(self, elapsed_s, snapshot):
        now = self.clock()
        events = []
        try:
            request, result, error = self.results.get_nowait()
        except queue.Empty:
            pass
        else:
            pending = self.pending
            self.pending = None
            if (
                pending is None
                or request["request_id"] != pending["request_id"]
                or request["epoch"] != self.epoch
            ):
                self.emit("policy_rejected", {"reason": "old_request_or_epoch"})
                raise RuntimeError("obsolete policy result")
            if error:
                raise RuntimeError("policy worker: " + error)
            if now - request["submitted_s"] > self.deadline:
                raise TimeoutError("late policy result")
            if (
                result is None
                or result.get("schema_version") != "policy-chunk/1"
                or result.get("request_id") != request["request_id"]
                or result.get("epoch") != self.epoch
                or result.get("observation_ns") != request["observation"]["time_ns"]
                or result.get("action_encoding") != "joint_absolute"
            ):
                raise ValueError("policy result identity/semantics mismatch")
            if result.get("action_spec") != self.action_spec:
                raise ValueError("policy coordinate names/frames/units mismatch")
            if int(now * 1e9) - result["observation_ns"] > self.max_age * 1e9:
                raise TimeoutError("stale policy observation")
            actions = result["actions"]
            if (
                not actions
                or any(not math.isfinite(a["time_s"]) for a in actions)
                or any(b["time_s"] <= a["time_s"] for a, b in zip(actions, actions[1:]))
            ):
                raise ValueError("invalid policy action timing")
            if actions[0]["time_s"] <= elapsed_s:
                raise TimeoutError("policy chunk starts in the past")
            if any(b["time_s"] - a["time_s"] > self.max_gap for a, b in zip(actions, actions[1:])):
                raise ValueError("policy action spacing exceeds queue budget")
            # New prediction replaces only the not-yet-committed future suffix.
            self.actions = [
                a for a in self.actions if a["time_s"] < actions[0]["time_s"]
            ] + deepcopy(actions)
            self.emit("policy_prediction", result)
            events.append({"request_id": request["request_id"], "accepted_actions": len(actions)})
        if self.pending and now - self.pending["submitted_s"] > self.deadline:
            raise TimeoutError("policy request timeout")
        if int(now * 1e9) - snapshot["time_ns"] > self.max_age * 1e9:
            raise TimeoutError("stale inference snapshot")
        if self.pending is None and (
            not self.actions or self.actions[-1]["time_s"] - elapsed_s < self.refill
        ):
            self.request_id += 1
            request = {
                "request_id": self.request_id,
                "epoch": self.epoch,
                "elapsed_s": elapsed_s,
                "submitted_s": now,
                "observation": deepcopy(snapshot),
            }
            self.pending = request
            self.requests.put_nowait(request)
            self.emit(
                "policy_request",
                {
                    "request_id": self.request_id,
                    "epoch": self.epoch,
                    "observation_ns": snapshot["time_ns"],
                },
            )
        due = [a for a in self.actions if a["time_s"] <= elapsed_s]
        self.actions = [a for a in self.actions if a["time_s"] > elapsed_s]
        if due:
            self.last_applied = due[-1]
            events.append({"applied_policy_time_s": self.last_applied["time_s"]})
        if self.last_applied:
            if elapsed_s - self.last_applied["time_s"] > self.max_gap:
                raise TimeoutError("policy queue exhausted")
            return deepcopy(self.last_applied["targets"]), events
        if elapsed_s > self.deadline + self.max_gap:
            raise TimeoutError("policy produced no actionable chunk")
        return None, events

    def close(self):
        self.epoch = None
        self.actions = []
        self.cancel.set()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            raise RuntimeError("policy worker did not stop")


class ExecutionSession:
    """Source-neutral ownership, start alignment, playback and stop lifecycle."""

    def __init__(self, gateway, source, settings, *, clock=time.monotonic):
        self.g, self.source, self.settings, self.clock = gateway, source, settings, clock
        self.state = "IDLE"
        self.reason = "not_started"
        self.seq = 0
        self.started = None
        self.settled = None
        self.last_jaws = {}
        self.last_tick = None

    def _transition(self, state, reason):
        self.state, self.reason = state, reason
        self.g.emit("workflow_state", {"state": state, "reason": reason})

    def start(self):
        if self.state != "IDLE":
            raise RuntimeError("session already started")
        checked_targets(self.source.initial, self.g)
        with self.g.lock:
            self.g.arm("workflow", enable_hardware=False)
            self.initial = {s: list(self.g.hold_target(v["q"])) for s, v in self.g.states.items()}
            for side in self.g.config["arms"]:
                self.g.activate(side)
            self.epoch = self.g.epoch
        self.source.begin(self.epoch)
        self.started = self.clock()
        self.last_tick = self.started
        self._transition("ALIGNING", "moving_to_explicit_initial_joint_targets; grippers unchanged")

    def _submit(self, targets, *, jaws):
        checked_targets(targets, self.g)
        self.seq += 1
        values = {s: v["q"] for s, v in targets.items()}
        widths = {s: targets[s]["gripper_m"] for s in jaws}
        result = self.g.submit(
            self.g.command(values, self.seq, {s: v["gripper_m"] for s, v in targets.items()}),
            active_sides=set(values),
            jaw_sides=set(widths),
        )
        if not result["accepted"]:
            raise RuntimeError("gateway rejected: " + result["reason"])
        for side, width in widths.items():
            self.last_jaws[side] = width

    def tick(self, snapshot=None):
        if self.state not in ("ALIGNING", "RUNNING"):
            return
        now = self.clock()
        try:
            if now - self.last_tick > self.settings["max_tick_gap_s"]:
                raise TimeoutError("execution scheduler stalled")
            self.last_tick = now
            with self.g.lock:
                if not self.g.enabled or self.g.epoch != self.epoch:
                    raise RuntimeError("ownership revoked")
                self.g.renew("workflow", self.epoch)
                if not self.g._fresh():
                    raise TimeoutError("stale feedback")
                if self.state == "ALIGNING":
                    elapsed = now - self.started
                    if elapsed > self.settings["alignment_timeout_s"]:
                        raise TimeoutError("start alignment feedback timeout")
                    p = min(1.0, elapsed / self.settings["alignment_duration_s"])
                    p = p * p * (3 - 2 * p)
                    targets = deepcopy(self.source.initial)
                    for s in targets:
                        targets[s]["q"] = [
                            a + (b - a) * p for a, b in zip(self.initial[s], targets[s]["q"])
                        ]
                    self._submit(targets, jaws=set())
                    error = max(
                        abs(a - b)
                        for s, v in self.source.initial.items()
                        for a, b in zip(v["q"], self.g.states[s]["q"])
                    )
                    if p >= 1 and error <= self.settings["alignment_tolerance_rad"]:
                        if self.settled is None:
                            self.settled = now
                        if now - self.settled >= self.settings["alignment_settle_s"]:
                            self.running_since = now
                            self._transition("RUNNING", "start_aligned")
                    else:
                        self.settled = None
                else:
                    elapsed = now - self.running_since
                    if elapsed >= self.source.duration_s:
                        self.stop("completed")
                        return
                    targets, events = self.source.sample(elapsed, snapshot)
                    if events:
                        self.g.emit("source_progress", {"elapsed_s": elapsed, "events": events})
                    if targets is not None:
                        jaws = {
                            s for s, v in targets.items() if self.last_jaws.get(s) != v["gripper_m"]
                        }
                        self._submit(targets, jaws=jaws)
        except Exception as exc:
            self.stop(str(exc), fault=True)

    def stop(self, reason="operator_stop", fault=False):
        if self.state in ("COMPLETED", "STOPPED", "FAULT", "CLOSED"):
            return
        self.g.stop(reason, fault=fault)
        self._transition(
            "FAULT" if fault else ("COMPLETED" if reason == "completed" else "STOPPED"), reason
        )

    def close(self):
        try:
            self.stop("shutdown")
        finally:
            self.source.close()
