"""Device-independent teleop workflow. HTTP never performs motion or IK.

The control thread is the sole normal command producer. IK is submitted to
latest-target workers. VR loss and explicit pause invalidate all old tokens.
"""

import math
import threading
import time
from collections import deque
from copy import deepcopy

from .webxr_input import validate_packet


def validate_session_profile(config):
    c = config
    from .diagnostics import diagnostic_config

    diagnostic_config(c.get("diagnostics"))
    from .pose_feedback import validate_pose_sources

    validate_pose_sources(c.get("pose_feedback"))
    if c.get("schema_version") != "live-teleop/2" or not c.get("gripper_control"):
        raise ValueError("session workflow requires live-teleop/2 with explicit grippers")
    if c.get("ik", {}).get("backend") != "pinocchio-scipy":
        raise ValueError("v2 session requires explicit Pinocchio/SciPy backend")
    if type(c.get("rotation_control")) is not bool:
        raise ValueError("explicit rotation mode required")
    tolerance = c.get("measurement_tolerance_rad")
    if (
        type(tolerance) not in (int, float)
        or not math.isfinite(tolerance)
        or not 0 <= tolerance <= 0.1
    ):
        raise ValueError("invalid feedback measurement tolerance")
    for name in ("deadline_ms", "max_iterations", "hang_timeout_s"):
        value = c["ik"].get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("invalid IK setting: " + name)
    if type(c["ik"].get("allow_best_effort")) is not bool:
        raise ValueError("explicit IK best-effort mode required")
    for value in c["ik"].get("weights", {}).values():
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError("invalid IK weight")
    if not isinstance(c.get("arms"), dict) or not c["arms"]:
        raise ValueError("named execution arms required")
    for key in (
        "max_cartesian_delta_m",
        "max_joint_excursion_rad",
        "max_tracking_error_rad",
        "max_joint_speed_rad_s",
    ):
        if c.get(key) is not None:
            raise ValueError("free session does not use commissioning envelopes: " + key)
    for key in ("scale", "input_timeout_s", "feedback_timeout_s"):
        if type(c.get(key)) not in (int, float) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("invalid session parameter: " + key)
    if (
        type(c.get("driver_speed_percent")) not in (int, float)
        or not 1 <= c["driver_speed_percent"] <= 100
    ):
        raise ValueError("invalid native driver speed")
    names = c["model"]["joint_names"]
    native_topics = [a["native_pose_topic"] for a in c["arms"].values() if "native_pose_topic" in a]
    control_topics = [
        a[k]
        for a in c["arms"].values()
        for k in ("command_topic", "enable_topic", "gripper_command_topic")
        if k in a
    ]
    if any(
        not isinstance(t, str) or not t.startswith("/") or t in control_topics
        for t in native_topics
    ) or len(set(native_topics)) != len(native_topics):
        raise ValueError("distinct native pose observation topics required")
    for side, arm in c["arms"].items():
        home = arm.get("home_rad")
        if (
            not isinstance(home, list)
            or len(home) != len(names)
            or not all(type(v) in (int, float) and math.isfinite(v) for v in home)
        ):
            raise ValueError("explicit finite home required: " + side)
        limits = arm.get("gripper", {}).get("limits_m")
        if (
            not isinstance(limits, list)
            or len(limits) != 2
            or not all(type(v) in (int, float) and math.isfinite(v) for v in limits)
            or not 0 <= limits[0] < limits[1]
        ):
            raise ValueError("explicit gripper limits required: " + side)
        if arm["gripper"].get("trigger_pressed") not in ("open", "close"):
            raise ValueError("explicit trigger pressed action required: " + side)
        if arm["gripper"].get("teleop_start", "preserve") not in ("preserve", "follow_trigger"):
            raise ValueError("unsupported gripper teleop start behavior: " + side)
    session = c["session"]
    for key in (
        "control_hz",
        "home_duration_s",
        "home_timeout_s",
        "home_settle_s",
        "home_tolerance_rad",
        "result_max_age_s",
        "preflight_timeout_s",
    ):
        if (
            type(session.get(key)) not in (int, float)
            or not math.isfinite(session[key])
            or session[key] <= 0
        ):
            raise ValueError("invalid session timing: " + key)
    if session["home_timeout_s"] <= session["home_duration_s"]:
        raise ValueError("home timeout must allow trajectory and settling")
    if c.get("recording_enabled", False):
        from .real_episode import validate_recording

        validate_recording(c.get("recording", {}))
    transport = c.get("webxr_transport", {})
    if transport.get("input", "http") not in ("http", "websocket", "webrtc"):
        raise ValueError("unsupported input transport")
    if transport.get("input") == "websocket" and (
        type(transport.get("port")) is not int or not 1 <= transport["port"] <= 65535
    ):
        raise ValueError("explicit control port required")
    for key in ("preview_congestion_rtt_ms", "preview_congestion_fps"):
        if key in transport and (
            type(transport[key]) not in (int, float)
            or not math.isfinite(transport[key])
            or transport[key] <= 0
        ):
            raise ValueError("invalid transport setting")
    layout = c.get("vr_layout", {})
    if "hud_face_viewer" in layout and type(layout["hud_face_viewer"]) is not bool:
        raise ValueError("invalid HUD facing mode")
    if "hud_tilt_deg" in layout and (
        type(layout["hud_tilt_deg"]) not in (int, float)
        or not math.isfinite(layout["hud_tilt_deg"])
        or not -90 <= layout["hud_tilt_deg"] <= 90
    ):
        raise ValueError("invalid HUD tilt")
    buttons = c["buttons"]
    mode = c.get("gripper_control_mode", "clutch")
    if mode not in ("clutch", "independent_trigger"):
        raise ValueError("unsupported gripper control mode")
    if mode == "independent_trigger":
        topics = [
            a.get(k)
            for a in c["arms"].values()
            for k in ("command_topic", "enable_topic", "gripper_command_topic")
        ]
        if any(not isinstance(t, str) or not t.startswith("/") for t in topics) or len(
            set(topics)
        ) != len(topics):
            raise ValueError("distinct explicit arm/enable/gripper topics required")
    for key in ("primary", "discard"):
        if (
            buttons[key]["hand"] not in ("left", "right")
            or type(buttons[key]["index"]) is not int
            or not 0 <= buttons[key]["index"] < 64
        ):
            raise ValueError("invalid button binding")
    for key in ("clutch", "trigger"):
        if type(buttons[key]) is not int or not 0 <= buttons[key] < 64:
            raise ValueError("invalid analog button index")
    if buttons["primary"] == buttons["discard"]:
        raise ValueError("primary and discard buttons must differ")
    # Reuse the established transform validation, without importing ROS/math deps.
    from .live_teleop import validate_profile

    legacy = {
        **c,
        "schema_version": "live-teleop/1",
        "gripper_control": False,
        "max_joint_speed_rad_s": 1.0,
        "max_tracking_error_rad": 1.0,
        "measurement_tolerance_rad": 0.0,
        "clutch_button": buttons["clutch"],
        "ik_deadline_ms": c["ik"]["deadline_ms"],
    }
    # v1's fixed left/right restriction is avoided: validate just its rotation.
    if set(c["arms"]) == {"left", "right"}:
        validate_profile(legacy)
    else:
        raise ValueError("this deployed session currently binds left/right WebXR hands")
    return c


class TeleopSession:
    """Own the operator workflow while keeping ingress separate from control.

    Network callers only update input state under ``input_lock``. The control
    tick consumes that snapshot and ordered button edges, updates the state
    machine, and submits through the gateway. Solver tokens invalidate late
    results across pause, reference changes and ownership changes.
    """

    def __init__(
        self,
        gateway,
        workers,
        *,
        ready_check=lambda: (True, []),
        clock=time.monotonic,
        recorder=None,
        recording_context=lambda: {},
        metrics=None,
    ):
        self.metrics = metrics
        self.g, self.config, self.clock = gateway, gateway.config, clock
        self.workers, self.ready_check = workers, ready_check
        self.recorder, self.recording_context = recorder, recording_context
        self.finalize_started = None
        self.lock, self.input_lock = threading.RLock(), threading.Lock()
        self.state, self.reason, self.resume_state = "SELF_CHECK", "starting", "STANDBY"
        self.started = clock()
        self.viewer = self.reference = self.latest = None
        self.received_s = 0.0
        self.last_seq, self.previous_buttons = {}, {}
        self.edges, self.pause_requested = deque(maxlen=32), None
        self.neutral_required = True
        self.anchors, self.tokens, self.targets, self.requested = {}, {}, {}, {}
        self.token_counter = self.command_seq = self.trials = 0
        self.last_motion = {}
        self.jaw_baselines, self.jaw_targets = {}, {}
        self.home_started = self.settled_since = None
        self.closed, self.closing = False, threading.Event()
        self.thread = None

    def _transition(self, state, reason):
        self.state, self.reason = state, reason
        self.g.emit("session_state", {"state": state, "reason": reason, "trials": self.trials})

    def ingest(self, packet):
        """Validate and retain input; never perform IK or driver I/O here."""
        validate_packet(packet)
        now, sid = self.clock(), packet["session_id"]
        with self.input_lock:
            if (
                self.viewer is not None
                and sid != self.viewer
                and now - self.received_s < self.config["input_timeout_s"]
            ):
                raise PermissionError("another VR session owns the input workflow")
            if sid not in self.last_seq and len(self.last_seq) >= 16:
                raise ValueError("input session capacity reached")
            if packet["seq"] <= self.last_seq.get(sid, -1):
                raise ValueError("reordered input")
            self.last_seq[sid] = packet["seq"]
            hands = [
                h["handedness"] for h in packet["inputs"] if h["handedness"] in self.config["arms"]
            ]
            if len(hands) != len(set(hands)):
                raise ValueError("duplicate handedness")
            reference = (sid, packet["epoch"], packet["reference_space"])
            if self.reference is not None and reference != self.reference:
                self.pause_requested = "reference_or_browser_changed"
                self.previous_buttons.clear()
                self.neutral_required = True
            self.viewer, self.reference = sid, reference
            self.latest, self.received_s = packet, now
            if (
                packet["event"] != "frame"
                or packet.get("visibility") != "visible"
                or not packet.get("head")
            ):
                self.pause_requested = "vr_not_visible_or_tracked"
                self.previous_buttons.clear()
                self.neutral_required = True
                return
            values = self._buttons(packet)
            if self.neutral_required:
                # A held across entry/pause cannot start home or teleop.
                self.previous_buttons = values
                if not any(values.values()):
                    self.neutral_required = False
                return
            for name in ("primary", "discard"):
                if values[name] and not self.previous_buttons.get(name, False):
                    if len(self.edges) == self.edges.maxlen:
                        raise ValueError("button event queue full")
                    self.edges.append((name, reference, packet["seq"]))
            self.previous_buttons = values

    def _buttons(self, packet):
        hands = {x["handedness"]: x for x in packet["inputs"]}
        values = {}
        for key in ("primary", "discard"):
            binding = self.config["buttons"][key]
            values[key] = self._value(hands.get(binding["hand"], {}), binding["index"]) >= 0.7
        for side in self.config["arms"]:
            values[side + ".clutch"] = (
                self._value(hands.get(side, {}), self.config["buttons"]["clutch"]) >= 0.7
            )
        return values

    @staticmethod
    def _value(hand, index):
        buttons = (hand.get("gamepad") or {}).get("buttons", [])
        return buttons[index]["value"] if len(buttons) > index else 0.0

    def request_pause(self, reason="operator_pause"):
        with self.input_lock:
            self.pause_requested = reason

    def _clear_motion(self):
        # Clearing pending work alone is insufficient: an already running IK
        # process can still return, so advance the token generation as well.
        self.anchors.clear()
        self.tokens.clear()
        self.targets.clear()
        self.requested.clear()
        self.token_counter += 1
        self.jaw_baselines.clear()
        self.jaw_targets.clear()
        for worker in self.workers.values():
            worker.clear()

    def _pause(self, reason, fault=False):
        if self.recorder and self.recorder.snapshot()["state"] in ("STARTING", "RECORDING"):
            self.recorder.finish(reason)
            self.finalize_started = self.clock()
        if self.state not in ("PAUSED", "FAULT", "CLOSED"):
            self.resume_state = (
                self.state if self.state in ("READY", "TELEOP", "STANDBY") else "STANDBY"
            )
        self.g.stop(reason, fault=fault)
        self._clear_motion()
        with self.input_lock:
            self.edges.clear()
            self.neutral_required = True
        self._transition("FAULT" if fault else "PAUSED", reason)

    def _acquire(self):
        with self.g.lock:
            self.g.arm(self.viewer, enable_hardware=False)

    def _start_home(self):
        self._acquire()
        with self.g.lock:
            self.home_from = {s: self.g.hold_target(v["q"]) for s, v in self.g.states.items()}
            for s in self.config["arms"]:
                self.g.activate(s)
        self.home_started, self.settled_since = self.clock(), None
        self._transition("HOMING", "moving_to_configured_home")

    def _start_teleop(self):
        if self.recorder and self.recorder.snapshot()["state"] == "IDLE":
            self.recorder.start(self.recording_context())
            self._transition("PREPARING", "opening_episode")
            return
        self._clear_motion()
        self._acquire()
        self._transition("TELEOP", "squeeze_to_follow")
        self.g.emit("teleop_started", {"trial": self.trials + 1, "recording": bool(self.recorder)})

    def _primary(self):
        if self.state == "STANDBY":
            self._start_home()
        elif self.state == "READY":
            self._start_teleop()
        elif self.state == "TELEOP":
            self.g.stop("teleop_ended")
            self._clear_motion()
            self.trials += 1
            self.g.emit("teleop_ended", {"trial": self.trials, "recording": bool(self.recorder)})
            if self.recorder:
                self.recorder.finish()
                self.finalize_started = self.clock()
                self._transition("FINALIZING", "draining_and_checking_episode")
            else:
                self._transition("STANDBY", "press_A_to_home")
        elif self.state == "REVIEW":
            self.recorder.decide(True)
            self._transition("COMMITTING", "saving_episode")
        elif self.state == "PAUSED":
            if self.recorder and self.recorder.snapshot()["state"] != "IDLE":
                self._transition("FINALIZING", "waiting_for_episode_review")
                return
            # The rising edge is already an explicit operator action after a
            # neutral frame. Apply the displayed phase's A action directly;
            # an invisible resume-only step made a received A look ineffective.
            if self.resume_state == "STANDBY":
                self._start_home()
            else:
                self._start_teleop()

    def tick(self):
        with self.lock:
            if self.closed:
                return
            for worker in self.workers.values():
                worker.poll()
            errors = [w.error for w in self.workers.values() if w.error]
            if errors and self.state != "FAULT":
                self._pause("solver: " + str(errors[0]), fault=True)
            if self.g.faulted and self.state != "FAULT":
                self._pause("gateway_fault", fault=True)
            if self.recorder:
                recording = self.recorder.snapshot()
                phase = recording["state"]
                if phase == "ERROR" and self.state != "FAULT":
                    self._pause("recorder: " + str(recording["error"]), fault=True)
                if self.state == "TELEOP" and phase in ("FINALIZING", "REVIEW"):
                    self.g.stop("recorder_interrupted")
                    self._clear_motion()
                    self.finalize_started = self.clock()
                    self._transition("FINALIZING", "recorder_interrupted")
                if self.state == "PREPARING" and phase == "RECORDING":
                    self._start_teleop()
                elif phase == "REVIEW" and self.state in ("FINALIZING", "PAUSED"):
                    self._transition("REVIEW", "A_save_X_discard")
                elif self.state == "COMMITTING" and phase == "IDLE":
                    self._transition("STANDBY", "press_A_to_home")
                if (
                    self.state == "FINALIZING"
                    and self.finalize_started is not None
                    and self.clock() - self.finalize_started
                    > self.config["recording"]["flush_timeout_s"]
                ):
                    self._pause("recorder_flush_timeout", fault=True)
            if self.state == "SELF_CHECK":
                ok, missing = self.ready_check()
                with self.g.lock:
                    feedback_ready = self.g._fresh()
                all_ready = ok and feedback_ready and all(w.ready for w in self.workers.values())
                if all_ready:
                    self._transition("STANDBY", "press_A_to_home")
                else:
                    self.reason = "waiting: " + ",".join(
                        missing
                        + ([] if feedback_ready else ["arm_feedback"])
                        + ([] if all(w.ready for w in self.workers.values()) else ["IK_workers"])
                    )
                    if self.clock() - self.started > self.config["session"]["preflight_timeout_s"]:
                        self._pause(self.reason, fault=True)
                return
            with self.input_lock:
                packet, received = self.latest, self.received_s
                pause, self.pause_requested = self.pause_requested, None
                edges = list(self.edges)
                self.edges.clear()
                neutral = self.neutral_required
            if pause and self.state not in ("PAUSED", "FAULT"):
                self._pause(pause)
                return
            if self.state in ("FAULT", "CLOSED"):
                return
            fresh = packet is not None and self.clock() - received < self.config["input_timeout_s"]
            if self.state == "PREPARING" and not fresh:
                self._pause("input_timeout_before_recording")
                return
            if self.state in ("HOMING", "TELEOP"):
                if not fresh or not self.g.enabled:
                    self._pause("input_timeout_or_revoked")
                    return
                with self.g.lock:
                    if not self.g._fresh():
                        self._pause("stale_feedback")
                        return
                    self.g.renew(self.viewer, self.g.epoch)
            if not fresh:
                return
            for key, reference, seq in edges:
                if reference != self.reference:
                    continue
                if key == "primary":
                    self._primary()
                elif self.state == "REVIEW":
                    self.recorder.decide(False)
                    self._transition("COMMITTING", "discarding_episode")
                else:
                    self.g.emit(
                        "button_ignored", {"button": "X", "reason": "not_in_review", "seq": seq}
                    )
            if self.state == "HOMING":
                self._home_tick()
            elif self.state == "TELEOP":
                self._teleop_tick(packet, neutral)

    def _submit(self, targets, active, jaws=None):
        with self.g.lock:
            self.command_seq += 1
            command = self.g.command(targets, self.command_seq, jaws)
            result = self.g.submit(command, active_sides=active, jaw_sides=set(jaws or {}))
            if not result["accepted"]:
                self.reason = result["reason"]

    def _home_tick(self):
        now = self.clock()
        settings = self.config["session"]
        elapsed = now - self.home_started
        if elapsed > settings["home_timeout_s"]:
            self._pause("home_feedback_timeout", fault=True)
            return
        progress = min(1.0, elapsed / settings["home_duration_s"])
        blend = progress * progress * (3 - 2 * progress)
        targets = {
            s: tuple(a + (b - a) * blend for a, b in zip(self.home_from[s], arm["home_rad"]))
            for s, arm in self.config["arms"].items()
        }
        self._submit(targets, set(self.config["arms"]))
        with self.g.lock:
            error = max(
                abs(a - b)
                for s, arm in self.config["arms"].items()
                for a, b in zip(self.g.states[s]["q"], arm["home_rad"])
            )
        if progress >= 1 and error <= settings["home_tolerance_rad"]:
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= settings["home_settle_s"]:
                self.g.stop("home_reached")
                self._transition("READY", "press_A_to_start")
        else:
            self.settled_since = None

    def _teleop_tick(self, packet, neutral):
        hands = {x["handedness"]: x for x in packet["inputs"]}
        active, jaws = set(), {}
        with self.g.lock:
            values = {s: self.g.hold_target(v["q"]) for s, v in self.g.states.items()}
            epoch = self.g.epoch
        for side, arm in self.config["arms"].items():
            hand = hands.get(side, {})
            grip = hand.get("grip")
            lo, hi = arm["gripper"]["limits_m"]
            trigger = self._value(hand, self.config["buttons"]["trigger"])
            width = (
                lo + trigger * (hi - lo)
                if arm["gripper"]["trigger_pressed"] == "open"
                else hi - trigger * (hi - lo)
            )
            independent = self.config.get("gripper_control_mode", "clutch") == "independent_trigger"
            trigger_available = (
                len((hand.get("gamepad") or {}).get("buttons", []))
                > self.config["buttons"]["trigger"]
            )
            if independent and trigger_available and not neutral:
                source = hand.get("source_id")
                baseline = self.jaw_baselines.get(side)
                if baseline is None or baseline[0] != source:
                    self.jaw_baselines[side] = (source, trigger)
                    self.jaw_targets.pop(side, None)
                    if arm["gripper"].get("teleop_start", "preserve") == "follow_trigger":
                        self.jaw_targets[side] = width
                elif abs(trigger - baseline[1]) > 0.01:
                    self.jaw_targets[side] = width
                if side in self.jaw_targets:
                    self.jaw_targets[side] = width
                    jaws[side] = width
            engaged = (
                not neutral
                and grip is not None
                and not grip.get("emulated_position", False)
                and self._value(hand, self.config["buttons"]["clutch"]) >= 0.7
            )
            if not engaged:
                if side in self.anchors:
                    self.g.release(side)
                    self.anchors.pop(side, None)
                    self.tokens.pop(side, None)
                    self.targets.pop(side, None)
                    self.workers[side].clear()
                continue
            active.add(side)
            if side not in self.anchors:
                self.g.activate(side)
                with self.g.lock:
                    q = self.g.states[side]["q"]
                self.token_counter += 1
                self.tokens[side] = f"{epoch}:{self.token_counter}:{packet['epoch']}"
                self.anchors[side] = (deepcopy(grip), self.g.model.fk(q), hand.get("source_id"))
                self.g.emit(
                    "mapping_anchor",
                    {"side": side, "token": self.tokens[side], "q": q, "hand": grip},
                )
            hand0, pose0, source_id = self.anchors[side]
            if source_id != hand.get("source_id"):
                self._pause("controller_replaced")
                return
            if self.requested.get(side) != packet["seq"]:
                target = self.g.model.relative_target(
                    pose0,
                    hand0,
                    grip,
                    self.config["alignment"],
                    self.config["scale"],
                    self.config["rotation_control"],
                )
                with self.g.lock:
                    seed = self.g.states[side]["q"]
                goal = {
                    "seq": packet["seq"],
                    "token": self.tokens[side],
                    "created_s": self.clock(),
                    "target": target,
                    "seed": seed,
                }
                self.g.emit("intent", {"side": side, **goal})
                self.workers[side].request(goal)
                self.requested[side] = packet["seq"]
            result = self.workers[side].take_result()
            if result is not None:
                usable = (
                    result["token"] == self.tokens[side]
                    and 0
                    <= self.clock() - result["created_s"]
                    < self.config["session"]["result_max_age_s"]
                )
                self.g.emit("motion_result", {"side": side, "usable": usable, **result})
                if usable:
                    self.last_motion[side] = result["result"]
                    if result["result"]["accepted"]:
                        self.targets[side] = tuple(result["result"]["positions"])
            values[side] = self.targets.get(side, values[side])
            if not independent:
                jaws[side] = width
        if active or jaws:
            self._submit(values, active, jaws)

    def snapshot(self):
        with self.lock:
            recording = self.recorder.snapshot() if self.recorder else None
            home = None
            if self.state == "HOMING":
                now = self.clock()
                with self.g.lock:
                    errors = {
                        side: max(
                            abs(q - target)
                            for q, target in zip(self.g.states[side]["q"], arm["home_rad"])
                        )
                        for side, arm in self.config["arms"].items()
                        if side in self.g.states
                    }
                home = {
                    "elapsed_s": now - self.home_started,
                    "error_rad": errors,
                    "tolerance_rad": self.config["session"]["home_tolerance_rad"],
                    "settled_s": 0.0 if self.settled_since is None else now - self.settled_since,
                }
            return {
                "state": self.state,
                "reason": self.reason,
                "resume_state": self.resume_state,
                "home": home,
                "active_sides": sorted(self.anchors),
                "completed_trials": self.trials,
                "saved_episodes": recording["saved_episodes"] if recording else None,
                "recording": bool(recording and recording["state"] == "RECORDING"),
                "episode": recording,
                "last_motion": deepcopy(self.last_motion),
                "workers": {
                    s: {"ready": w.ready, "error": w.error} for s, w in self.workers.items()
                },
            }

    def start(self):
        def run():
            previous = None
            while not self.closing.is_set():
                began = time.monotonic_ns()
                if self.metrics and previous is not None:
                    self.metrics.observe("control.tick_interval", (began - previous) / 1e6)
                previous = began
                try:
                    self.tick()
                except Exception as exc:
                    with self.lock:
                        try:
                            self._pause("session_error: " + str(exc), fault=True)
                        except Exception:
                            self.g.stop("session_failure", fault=True)
                finally:
                    if self.metrics:
                        self.metrics.elapsed("control.tick_work", began)
                self.closing.wait(1 / self.config["session"]["control_hz"])

        self.thread = threading.Thread(target=run, name="teleop-control", daemon=True)
        self.thread.start()

    def close(self):
        if self.closed:
            return
        self.closing.set()
        if self.thread:
            self.thread.join(timeout=2)
        with self.lock:
            self.g.stop("session_closed")
            self._clear_motion()
            self.closed = True
            self._transition("CLOSED", "session_closed")
        for worker in self.workers.values():
            worker.close()
