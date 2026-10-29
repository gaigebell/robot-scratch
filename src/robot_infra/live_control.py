"""Device-local bounded joint gateway and relative-position teleop mapper.

No ROS imports. The fake collection runtime stays fake-only. This experimental
live slice shares named CommandSet semantics, but has a real monotonic clock.
"""

import math
import threading
import time
from dataclasses import asdict

from .collection import CommandSet, EndpointTarget
from .webxr_input import validate_packet


class LiveGateway:
    """Arbitrate the only normal command path to the bound backend.

    A private capability prevents applications from acting as backend writers.
    Ownership, epoch, sequence and fresh feedback are checked before a target
    is published; a published receipt must not be mistaken for a motor ACK.
    """

    def __init__(self, config, model, backend, emit, *, clock=time.monotonic):
        if config["schema_version"] not in ("live-teleop/1", "live-teleop/2"):
            raise ValueError("unsupported live profile")
        self.config, self.model, self.backend, self.emit, self.clock = (
            config,
            model,
            backend,
            emit,
            clock,
        )
        self.lock = threading.RLock()
        self.capability = object()
        backend.bind(self.capability)
        self.epoch, self.owner, self.expiry, self.seq = 0, None, 0.0, -1
        self.states, self.initial, self.bounds, self.last_sent = {}, {}, {}, {}
        self.published_targets = {}
        self.enabled, self.faulted = False, False
        self.last_reason = "not_armed"
        self.active_sides = set()

    def feedback(self, side, q, gripper, device_stamp_ns):
        with self.lock:
            if (
                side not in self.config["arms"]
                or len(q) != len(self.model.names)
                or not all(math.isfinite(v) for v in (*q, gripper))
            ):
                raise ValueError("invalid feedback")
            now = self.clock()
            old = self.states.get(side)
            # Repeated ROS publications do not make frozen CAN feedback fresh.
            changed = (
                now
                if old is None or old["device_stamp_ns"] != device_stamp_ns
                else old["device_changed_s"]
            )
            self.states[side] = {
                "q": tuple(q),
                "gripper": gripper,
                "received_s": now,
                "device_stamp_ns": device_stamp_ns,
                "device_changed_s": changed,
            }
            self.emit("feedback", {"side": side, **self.states[side]})

    def _fresh(self):
        now = self.clock()
        return set(self.states) == set(self.config["arms"]) and all(
            0 <= now - s["received_s"] < self.config["feedback_timeout_s"]
            and 0 <= now - s["device_changed_s"] < self.config["feedback_timeout_s"]
            for s in self.states.values()
        )

    def arm(self, owner, *, enable_hardware=True):
        with self.lock:
            if self.faulted or self.owner is not None or not self._fresh():
                raise ValueError("arm requires idle, fresh feedback and no latched fault")
            if not isinstance(owner, str) or not 1 <= len(owner) <= 128:
                raise ValueError("invalid owner")
            # An out-of-limit measured hold may be clamped by a downstream
            # controller. Reject it before enabling instead of assuming no jump.
            for side, state in self.states.items():
                q = state["q"]
                if any(
                    max(lo - v, v - hi, 0) > self.config.get("measurement_tolerance_rad", 0.0)
                    for v, (lo, hi) in zip(q, self.model.limits)
                ):
                    raise ValueError(
                        "initial state outside model limits; resolve zero/calibration before enable"
                    )
                self.initial[side] = q
                self.bounds[side] = list(self.model.limits)
            self.epoch += 1
            self.owner, self.expiry, self.seq = (
                owner,
                self.clock() + self.config["input_timeout_s"],
                -1,
            )
            self.enabled = True
            self.last_reason = "armed"
            self.last_sent = {
                side: (self.hold_target(s["q"]), self.clock()) for side, s in self.states.items()
            }
            self.emit(
                "ownership",
                {"owner": owner, "epoch": self.epoch, "bounds": self.bounds, "reason": "granted"},
            )
            # Enable is guarded by the same group state and exclusive backend.
            try:
                if enable_hardware:
                    for side in self.config["arms"]:
                        self.activate(side)
            except Exception:
                self.stop("enable_failure", fault=True)
                raise
            return self.epoch

    def activate(self, side):
        with self.lock:
            if (
                not self.enabled
                or side not in self.config["arms"]
                or not self._fresh()
                or self.clock() > self.expiry
            ):
                raise PermissionError("activation requires fresh owned group")
            if side in self.active_sides:
                return
            self.emit(
                "guard",
                {"decision": "accepted", "operation": "enable", "side": side, "epoch": self.epoch},
            )
            self.backend.enable(side, self.capability)
            self.active_sides.add(side)
            self.emit(
                "receipt",
                {"side": side, "operation": "enable", "status": "published_not_acknowledged"},
            )
            self._hold("initial_hold", sides={side})

    def release(self, side):
        with self.lock:
            if side in self.active_sides:
                self._hold("clutch_release_" + side, sides={side})
                self.active_sides.discard(side)

    def renew(self, owner, epoch):
        with self.lock:
            if owner != self.owner or epoch != self.epoch or self.clock() > self.expiry:
                raise PermissionError("invalid or expired ownership")
            self.expiry = self.clock() + self.config["input_timeout_s"]

    def _reason(self, command, check_seq=True):
        if not self.enabled or command.source != self.owner or command.epoch != self.epoch:
            return "ownership"
        now = self.clock()
        if now > self.expiry:
            return "input_timeout"
        if not self._fresh():
            return "stale_feedback"
        if check_seq and command.seq <= self.seq:
            return "sequence"
        if not command.created_ns <= int(now * 1e9) <= command.expires_ns:
            return "command_expired"
        targets = {t.endpoint_id: t for t in command.targets}
        expected = {s + ".arm" for s in self.config["arms"]}
        if self.config.get("gripper_control"):
            expected |= {s + ".gripper" for s in self.config["arms"]}
        if len(targets) != len(command.targets) or set(targets) != expected:
            return "endpoint_set"
        for side in self.config["arms"]:
            t = targets[side + ".arm"]
            q = t.positions
            if (
                t.names != tuple(self.model.names)
                or t.unit != "rad"
                or t.frame_id != side + "." + self.model.base
                or len(q) != len(self.model.names)
                or not all(math.isfinite(v) for v in q)
            ):
                return "joint_semantics"
            if any(v < lo - 1e-9 or v > hi + 1e-9 for v, (lo, hi) in zip(q, self.bounds[side])):
                return "joint_limit"
            if self.config["max_joint_excursion_rad"] is not None and any(
                abs(a - b) > self.config["max_joint_excursion_rad"]
                for a, b in zip(q, self.initial[side])
            ):
                return "test_envelope"
            if self.config["max_tracking_error_rad"] is not None and any(
                abs(a - b) > self.config["max_tracking_error_rad"]
                for a, b in zip(q, self.states[side]["q"])
            ):
                return "tracking_error"
            last, when = self.last_sent[side]
            speed = self.config["max_joint_speed_rad_s"]
            if speed is not None:
                allowed = speed * max(0.0, min(0.1, now - when)) + 1e-6
                if any(abs(a - b) > allowed for a, b in zip(q, last)):
                    return "joint_speed"
            if self.config.get("gripper_control"):
                jaw = targets[side + ".gripper"]
                low, high = self.config["arms"][side]["gripper"]["limits_m"]
                if (
                    jaw.names != ("width",)
                    or jaw.unit != "m"
                    or jaw.frame_id != side + "." + self.model.tip
                    or len(jaw.positions) != 1
                    or not math.isfinite(jaw.positions[0])
                ):
                    return "gripper_semantics"
                if not low <= jaw.positions[0] <= high:
                    return "gripper_limit"
        return None

    def submit(self, command, *, active_sides=None, jaw_sides=None):
        with self.lock:
            active_sides = set(self.config["arms"]) if active_sides is None else set(active_sides)
            jaw_sides = set(active_sides) if jaw_sides is None else set(jaw_sides)
            independent = self.config.get("gripper_control_mode", "clutch") == "independent_trigger"
            if not active_sides <= set(self.config["arms"]) or not jaw_sides <= (
                set(self.config["arms"]) if independent else active_sides
            ):
                raise ValueError("invalid active mask")
            if (
                self.config["schema_version"] == "live-teleop/2"
                and not active_sides <= self.active_sides
            ):
                raise PermissionError("backend not activated by session")
            reason = self._reason(command)
            self.emit(
                "guard",
                {
                    "decision": "rejected" if reason else "accepted",
                    "reason": reason,
                    "command": asdict(command),
                    "active_sides": sorted(active_sides),
                    "jaw_sides": sorted(jaw_sides),
                },
            )
            self.last_reason = reason or "following"
            if reason:
                return {"accepted": False, "reason": reason}
            self.seq = command.seq
            targets = {t.endpoint_id: t.positions for t in command.targets}
            try:
                for side in self.config["arms"]:
                    if side not in active_sides and side not in jaw_sides:
                        continue
                    reason = self._reason(command, False)
                    if reason:
                        self.stop("pre_send_" + reason, fault=True)
                        return {"accepted": False, "reason": reason}
                    if side in active_sides:
                        q = targets[side + ".arm"]
                        gripper = (
                            targets[side + ".gripper"][0]
                            if self.config.get("gripper_control")
                            and side in jaw_sides
                            and not independent
                            else (
                                None
                                if self.config["schema_version"] == "live-teleop/2"
                                else self.states[side]["gripper"]
                            )
                        )
                        self.emit(
                            "command",
                            {
                                "side": side,
                                "q": q,
                                "gripper_m": gripper,
                                "seq": command.seq,
                                "source": command.source,
                            },
                        )
                        self.backend.send(side, q, gripper, self.capability)
                        self.remember_target(side, "q", q)
                        if gripper is not None:
                            self.remember_target(side, "gripper_m", [gripper])
                        self.last_sent[side] = (q, self.clock())
                    if independent and side in jaw_sides:
                        width = targets[side + ".gripper"][0]
                        self.emit(
                            "command",
                            {
                                "side": side,
                                "operation": "gripper_only",
                                "gripper_m": width,
                                "seq": command.seq,
                                "source": command.source,
                            },
                        )
                        self.backend.send_gripper(side, width, self.capability)
                        self.remember_target(side, "gripper_m", [width])
                    self.emit(
                        "receipt",
                        {"side": side, "seq": command.seq, "status": "published_not_acknowledged"},
                    )
            except Exception:
                self.stop("partial_send_failure", fault=True)
                raise
            return {"accepted": True, "status": "published_not_acknowledged"}

    def _hold(self, reason, sides=None):
        if not self._fresh():
            self._stop_emit(
                "stop", {"reason": reason, "result": "no_fresh_feedback_hold_unavailable"}
            )
            return
        for side, state in self.states.items():
            selected = (
                self.active_sides
                if self.config["schema_version"] == "live-teleop/2"
                else set(self.states)
            )
            if side not in (selected if sides is None else sides):
                continue
            # Stop uses a fresh measured hold, never home/disable/gripper-close.
            self._stop_emit(
                "guard",
                {
                    "decision": "accepted",
                    "operation": "measured_hold",
                    "side": side,
                    "reason": reason,
                },
            )
            q = self.hold_target(state["q"])
            self.backend.send(
                side,
                q,
                None if self.config["schema_version"] == "live-teleop/2" else state["gripper"],
                self.capability,
            )
            self.remember_target(side, "q", q)
            self._stop_emit(
                "command",
                {
                    "side": side,
                    "q": q,
                    "observed_q": state["q"],
                    "operation": "measured_hold",
                    "reason": reason,
                },
            )
        self._stop_emit(
            "stop", {"reason": reason, "result": "measured_hold_published_not_acknowledged"}
        )

    def _stop_emit(self, kind, payload):
        try:
            self.emit(kind, payload)
        except Exception:
            self.faulted = True  # A failed journal must not prevent a hold attempt.

    def remember_target(self, side, field, values):
        # Evidence of a returned publish call, not a hardware execution ACK.
        self.published_targets.setdefault(side, {})[field] = {
            "values": list(values),
            "time_ns": int(self.clock() * 1e9),
        }

    def hold_target(self, q):
        tolerance = self.config.get("measurement_tolerance_rad", 0.0)
        if any(max(lo - v, v - hi, 0) > tolerance for v, (lo, hi) in zip(q, self.model.limits)):
            raise ValueError("measured state outside hold tolerance")
        return tuple(max(lo, min(hi, v)) for v, (lo, hi) in zip(q, self.model.limits))

    def stop(self, reason, fault=False):
        with self.lock:
            had_control = self.enabled
            self.epoch += 1
            self.owner, self.enabled, self.expiry = None, False, 0.0
            self.faulted |= fault
            self.last_reason = reason
            self._stop_emit("ownership", {"owner": None, "epoch": self.epoch, "reason": reason})
            if had_control:
                try:
                    self._hold(reason)
                except Exception as exc:
                    self.faulted = True
                    self._stop_emit(
                        "stop", {"reason": reason, "result": "hold_failed", "error": str(exc)}
                    )
            self.active_sides.clear()

    def watchdog(self):
        with self.lock:
            if self.enabled and (self.clock() > self.expiry or not self._fresh()):
                self.stop("input_timeout" if self.clock() > self.expiry else "stale_feedback")

    def command(self, values, seq, grippers=None):
        now = int(self.clock() * 1e9)
        targets = [
            EndpointTarget(
                s + ".arm",
                tuple(self.model.names),
                tuple(values[s]),
                "rad",
                s + "." + self.model.base,
            )
            for s in self.config["arms"]
        ]
        if self.config.get("gripper_control"):
            for s in self.config["arms"]:
                lo, hi = self.config["arms"][s]["gripper"]["limits_m"]
                width = (grippers or {}).get(s, max(lo, min(hi, self.states[s]["gripper"])))
                targets.append(
                    EndpointTarget(
                        s + ".gripper", ("width",), (width,), "m", s + "." + self.model.tip
                    )
                )
        return CommandSet(
            "live-device",
            self.owner,
            seq,
            self.epoch,
            str(self.epoch),
            "fresh-device-feedback",
            now,
            now,
            now + 100_000_000,
            tuple(targets),
        )


class RelativePositionMapper:
    def __init__(self, gateway):
        self.gateway, self.anchors = gateway, {}
        self.reference = None
        self.last_seq = -1
        self.command_seq = 0
        self.active = {}

    def reset(self):
        self.anchors.clear()
        self.reference = None
        self.last_seq = -1

    def process(self, packet):
        validate_packet(packet)
        g = self.gateway
        with g.lock:
            if packet["session_id"] != g.owner:
                raise PermissionError("input session does not own robot")
            if packet["seq"] <= self.last_seq:
                raise ValueError("reordered input")
            self.last_seq = packet["seq"]
            reference = (packet["session_id"], packet["epoch"], packet["reference_space"])
            if self.reference is not None and reference != self.reference:
                g.stop("reference_reset")
                self.reset()
                return
            self.reference = reference
            if (
                packet["event"] != "frame"
                or packet.get("visibility") != "visible"
                or not packet.get("head")
            ):
                g.stop("tracking_or_lifecycle")
                self.reset()
                return
            g.renew(packet["session_id"], g.epoch)
            if not g._fresh():
                g.stop("stale_feedback")
                return
            hands = {}
            for source in packet["inputs"]:
                side = source["handedness"]
                if side in hands:
                    raise ValueError("duplicate handedness")
                hands[side] = source
            values, active = {}, {}
            for side in g.config["arms"]:
                state = g.states[side]
                q = state["q"]
                values[side] = g.hold_target(q)
                hand = hands.get(side, {})
                buttons = (hand.get("gamepad") or {}).get("buttons", [])
                grip = hand.get("grip")
                index = g.config["clutch_button"]
                engaged = (
                    grip is not None
                    and not grip.get("emulated_position", False)
                    and len(buttons) > index
                    and buttons[index]["value"] >= 0.7
                )
                active[side] = engaged
                if not engaged:
                    if side in self.anchors:
                        # One measured hold on release, then rebase rate limiter.
                        g._hold("clutch_release_" + side)
                        g.last_sent = {
                            s: (g.hold_target(v["q"]), g.clock()) for s, v in g.states.items()
                        }
                    self.anchors.pop(side, None)
                    continue
                if side not in self.anchors:
                    self.anchors[side] = (grip["position_m"], g.model.fk(q), hand.get("source_id"))
                    g.emit("mapping_anchor", {"side": side, "vr": grip, "q": q})
                p0, pose0, source_id = self.anchors[side]
                if source_id != hand.get("source_id"):
                    g.stop("controller_replaced")
                    self.reset()
                    return
                delta = [a - b for a, b in zip(grip["position_m"], p0)]
                mapped = [
                    sum(a * b for a, b in zip(row, delta)) * g.config["scale"]
                    for row in g.config["alignment"]
                ]
                if (
                    g.config["max_cartesian_delta_m"] is not None
                    and math.sqrt(sum(v * v for v in mapped)) > g.config["max_cartesian_delta_m"]
                ):
                    g.emit(
                        "motion_result",
                        {"side": side, "accepted": False, "reason": "clutch_cartesian_envelope"},
                    )
                    continue
                target = [row.copy() for row in pose0]
                for i in range(3):
                    target[i][3] += mapped[i]
                result = g.model.ik(
                    target,
                    q,
                    limits=g.bounds[side],
                    max_ms=g.config["ik_deadline_ms"],
                    seed_tolerance_rad=g.config.get("measurement_tolerance_rad", 0.0),
                )
                g.emit(
                    "intent",
                    {
                        "side": side,
                        "input_seq": packet["seq"],
                        "target": target,
                        "position_only": True,
                    },
                )
                g.emit("motion_result", {"side": side, **asdict(result)})
                if result.accepted:
                    last, when = g.last_sent[side]
                    step = (
                        g.config["max_joint_speed_rad_s"]
                        * max(0.0, min(0.08, g.clock() - when))
                        * 0.9
                    )
                    diff = max(abs(a - b) for a, b in zip(result.positions, last))
                    ratio = min(1.0, step / max(1e-12, diff))
                    values[side] = tuple(
                        a + ratio * (b - a) for a, b in zip(last, result.positions)
                    )
            self.active = active
            self.command_seq += 1
            outcome = g.submit(g.command(values, self.command_seq))
            g.emit("mapping", {"input_seq": packet["seq"], "active": active, "outcome": outcome})
            return outcome
