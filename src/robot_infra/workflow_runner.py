"""Explicit native workflow binding; physical execution uses existing drivers."""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

from .diagnostics import EventJournal, diagnostic_config
from .experiment_graph import compile_experiment
from .live_control import LiveGateway
from .live_teleop import validate_profile
from .workflow import (
    AsyncPolicySource,
    ExecutionSession,
    MockPolicy,
    ReplaySource,
    checked_targets,
    finite,
)


def load_workflow(path, deployment=None):
    path = Path(path).expanduser().resolve()
    c = json.loads(path.read_text(encoding="utf-8-sig"))
    if (
        set(c)
        != {"schema_version", "graph", "backend", "profile", "source", "execution", "fake_device"}
        or c["schema_version"] != "workflow-session/1"
    ):
        raise ValueError("unsupported workflow session")

    def resolve(value):
        p = Path(value).expanduser()
        return str(p if p.is_absolute() else path.parent / p)

    if c["backend"] not in ("fake", "piper_ros"):
        raise ValueError("unsupported workflow backend")
    c["graph"] = resolve(c["graph"])
    graph = compile_experiment(json.loads(Path(c["graph"]).read_text(encoding="utf-8-sig")))
    if graph["mode"] not in ("replay", "inference") or graph["action_encoding"] != "joint_absolute":
        raise ValueError("native runner requires replay/inference joint_absolute graph")
    if deployment:
        from .deployment import load_deployment

        device = load_deployment(deployment)
        c["profile"] = device["config"]
    else:
        device = None
        c["profile"] = resolve(c["profile"])
    if c["backend"] == "piper_ros" and not device:
        raise ValueError("physical workflow requires explicit --deployment")
    profile = validate_profile(json.loads(Path(c["profile"]).read_text(encoding="utf-8-sig")))
    if profile.get("gripper_control_mode") != "independent_trigger":
        raise ValueError("native workflow requires independent gripper command channel")
    source = c["source"]
    kind = source.get("kind")
    if kind == "episode":
        if graph["mode"] != "replay":
            raise ValueError("episode source requires replay graph")
        if set(source) != {
            "kind",
            "episode",
            "training_config",
            "journals",
            "start_s",
            "duration_s",
            "speed",
            "max_lag_s",
        }:
            raise ValueError("invalid episode source config")
        for key in ("episode", "training_config"):
            source[key] = resolve(source[key])
        source["journals"] = [resolve(p) for p in source["journals"]]
    elif kind == "mock_policy":
        if graph["mode"] != "inference" or c["backend"] != "fake":
            raise ValueError("mock policy is fake-device-only in this slice")
        if set(source) != {
            "kind",
            "transport",
            "duration_s",
            "steps",
            "action_dt_s",
            "lead_s",
            "delay_s",
            "amplitude_rad",
            "fail_request",
            "deadline_s",
            "observation_max_age_s",
            "max_gap_s",
            "refill_horizon_s",
        }:
            raise ValueError("invalid mock policy config")
        transport = source["transport"]
        if transport == {"kind": "in_process"}:
            pass
        elif (
            isinstance(transport, dict)
            and set(transport) == {"kind", "endpoint", "timeout_s"}
            and transport["kind"] == "http_json"
        ):
            from .mock_policy_server import HTTPMockClient

            HTTPMockClient(transport["endpoint"], transport["timeout_s"])
            if transport["timeout_s"] > 1:
                raise ValueError("mock HTTP timeout must fit bounded worker shutdown")
        else:
            raise ValueError("unsupported policy transport")
    else:
        raise ValueError("unsupported source plugin")
    s = c["execution"]
    if set(s) != {
        "control_hz",
        "alignment_duration_s",
        "alignment_timeout_s",
        "alignment_settle_s",
        "alignment_tolerance_rad",
        "max_tick_gap_s",
        "readiness_timeout_s",
    }:
        raise ValueError("invalid execution settings")
    for k, v in s.items():
        finite(v, k)
    if (
        s["alignment_timeout_s"] <= s["alignment_duration_s"]
        or s["max_tick_gap_s"] >= profile["input_timeout_s"]
    ):
        raise ValueError("inconsistent execution deadlines")
    return c, profile, device, graph


def create_source(c, profile, emit):
    s = c["source"]
    if s["kind"] == "episode":
        training = json.loads(Path(s["training_config"]).read_text(encoding="utf-8-sig"))
        return ReplaySource.from_episode(
            s["episode"],
            training,
            profile["model"],
            profile["arms"],
            start_s=s["start_s"],
            duration_s=s["duration_s"],
            speed=s["speed"],
            journal_paths=s["journals"],
            max_lag_s=s["max_lag_s"],
        )
    initial = {
        side: {"q": list(arm["home_rad"]), "gripper_m": arm["gripper"]["limits_m"][0]}
        for side, arm in profile["arms"].items()
    }
    spec = {k: profile["model"][k] for k in ("joint_names", "base", "tip")}
    spec.update(joint_unit="rad", gripper_unit="m")
    policy = MockPolicy(
        initial,
        action_spec=spec,
        **{
            k: s[k]
            for k in ("steps", "action_dt_s", "lead_s", "delay_s", "amplitude_rad", "fail_request")
        },
    )
    if s["transport"]["kind"] == "http_json":
        from .mock_policy_server import HTTPMockClient

        policy = HTTPMockClient(s["transport"]["endpoint"], s["transport"]["timeout_s"])
    return AsyncPolicySource(
        policy,
        initial,
        s["duration_s"],
        emit,
        action_spec=spec,
        **{
            k: s[k]
            for k in ("deadline_s", "observation_max_age_s", "max_gap_s", "refill_horizon_s")
        },
    )


class FakeBackend:
    def __init__(self, profile):
        self.values = {
            s: {"q": list(a["home_rad"]), "gripper_m": a["gripper"]["limits_m"][0]}
            for s, a in profile["arms"].items()
        }

    def bind(self, cap):
        self.cap = cap

    def enable(self, side, cap):
        self._check(cap)

    def _check(self, cap):
        if cap is not self.cap:
            raise PermissionError("gateway capability required")

    def send(self, side, q, jaw, cap):
        self._check(cap)
        self.values[side]["q"] = list(q)
        if jaw is not None:
            self.values[side]["gripper_m"] = jaw

    def send_gripper(self, side, width, cap):
        self._check(cap)
        self.values[side]["gripper_m"] = width


def run_workflow(path, output, *, deployment=None, execute=False, check_only=False):
    c, profile, device, graph = load_workflow(path, deployment)
    physical = c["backend"] == "piper_ros"
    if physical and not execute and not check_only:
        raise ValueError(
            "physical execution requires --execute; use --check-only for passive validation"
        )
    if physical:
        from .kinematics import SerialModel

        model = SerialModel(**profile["model"])  # Joint metadata only; replay performs no IK.
        for side, arm in profile["arms"].items():
            interface = Path("/sys/class/net") / arm["can_interface"]
            if not interface.is_dir() or not int((interface / "flags").read_text(), 16) & 1:
                raise ValueError("CAN missing or DOWN: " + side)
    else:
        f = c["fake_device"]
        limits = f["joint_limits_rad"]
        if len(limits) != len(profile["model"]["joint_names"]) or any(
            len(x) != 2 or not all(math.isfinite(v) for v in x) or x[0] >= x[1] for x in limits
        ):
            raise ValueError("explicit fake joint limits required")
        if len(set(f["camera_roles"])) != 3:
            raise ValueError("three explicit fake RGB roles required")
        model = SimpleNamespace(
            names=profile["model"]["joint_names"],
            base=profile["model"]["base"],
            tip=profile["model"]["tip"],
            limits=limits,
        )
    # Preflight creates neither SDKs, ROS contexts nor hardware commands.
    source = create_source(c, profile, lambda k, p: None)
    try:
        checked_targets(source.initial, SimpleNamespace(config=profile, model=model))
        if isinstance(source, ReplaySource):
            for e in source.events:
                targets = deepcopy(source.initial)
                targets[e["side"]][e["field"]] = e["value"]
                checked_targets(targets, SimpleNamespace(config=profile, model=model))
        preflight = {
            "schema_version": "workflow-preflight/1",
            "backend": c["backend"],
            "mode": graph["mode"],
            "source_kind": c["source"]["kind"],
            "physical_commands": False,
            "initial_targets": source.initial,
            "duration_s": source.duration_s,
            "provenance": getattr(source, "provenance", {}),
            "bindings": "native session; no arbitrary graph interpreter; replay cameras not started",
        }
        if check_only:
            return preflight
    finally:
        source.close()
    root = Path(output).expanduser().resolve()
    if c["source"]["kind"] == "episode" and (
        root == Path(c["source"]["episode"]) or root.is_relative_to(Path(c["source"]["episode"]))
    ):
        raise ValueError("run output must be outside source episode")
    root.mkdir(parents=True, exist_ok=False)
    journal = EventJournal(root / "events", **diagnostic_config(profile.get("diagnostics")))
    children = []
    logs = []
    node = None
    context = None
    executor = None
    ros_thread = None
    watchdog = None
    stop = threading.Event()
    session = None
    source = None
    errors = []

    def emit(k, p):
        journal.emit(k, p)

    def terminate(signum, frame):
        raise KeyboardInterrupt

    old_signals = {}
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, terminate)
    result = {
        "state": "FAILED",
        "reason": "startup_failed",
        "physical_commands": physical,
        "backend": c["backend"],
    }
    try:
        effective = root / "effective-profile.json"
        effective.write_text(json.dumps(profile, indent=2), encoding="utf-8")
        (root / "workflow.json").write_text(json.dumps(c, indent=2), encoding="utf-8")
        (root / "preflight.json").write_text(json.dumps(preflight, indent=2), encoding="utf-8")
        from .provenance import runtime_identity

        emit(
            "workflow_manifest",
            {
                "config": c,
                "profile": profile,
                "runtime": runtime_identity(),
                "preflight": preflight,
                "observation_mode": "joint_feedback_only"
                if physical
                else "fake_state_and_three_rgb",
            },
        )
        if physical:
            import rclpy
            from piper_msgs.msg import PiperStatusMsg
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import JointState

            from .adapters.piper_ros import PiperROSBackend
            from .session_launcher import arm_arguments

            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(device["ros_domain_id"])
            os.environ["ROS_DOMAIN_ID"] = str(device["ros_domain_id"])
            context = Context()
            rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node("robot_infra_workflow_gateway", context=context)
            backend = PiperROSBackend(node, profile, model.names)
            gateway = LiveGateway(profile, model, backend, emit)
            statuses = {}
            for side, arm in profile["arms"].items():

                def feedback(m, s=side):
                    values = dict(zip(m.name, m.position))
                    if len(values) != len(m.name):
                        raise ValueError("duplicate feedback joints")
                    gateway.feedback(
                        s,
                        [values[n] for n in model.names],
                        values["gripper"],
                        m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                    )

                def status(m, s=side):
                    statuses[s] = {"arm_status": m.arm_status, "err_code": m.err_code}
                    if m.err_code or m.arm_status:
                        gateway.stop("driver_error_" + s, fault=True)

                node.create_subscription(JointState, arm["feedback_topic"], feedback, 1)
                node.create_subscription(PiperStatusMsg, arm["status_topic"], status, 1)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            def spin():
                try:
                    while not stop.is_set():
                        executor.spin_once(timeout_sec=0.02)
                except Exception as exc:
                    errors.append("ROS: " + str(exc))
                    gateway.stop("ROS executor failed", fault=True)

            ros_thread = threading.Thread(target=spin, name="workflow-ros", daemon=True)
            ros_thread.start()
            for side, arm in profile["arms"].items():
                log = (root / (side + "-driver.log")).open("x")
                logs.append(log)
                args = arm_arguments("unused", side, arm, gripper_enabled=True)
                command = [
                    sys.executable,
                    "-m",
                    "robot_infra.adapters.piper_driver",
                    "--config",
                    str(effective),
                    *args[1:],
                    "--log-level",
                    "warn",
                ]
                child = subprocess.Popen(
                    command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
                )
                children.append(child)
            (root / "processes.json").write_text(
                json.dumps(
                    {
                        "children": [p.pid for p in children],
                        "ros_domain_id": device["ros_domain_id"],
                    }
                )
            )
            deadline = time.monotonic() + c["execution"]["readiness_timeout_s"]
            while time.monotonic() < deadline:
                ok, missing = backend.ready()
                if errors or any(p.poll() is not None for p in children):
                    raise RuntimeError("workflow child failed during readiness")
                if (
                    ok
                    and gateway._fresh()
                    and set(statuses) == set(profile["arms"])
                    and not gateway.faulted
                ):
                    break
                time.sleep(0.05)
            else:
                raise TimeoutError("workflow feedback/driver readiness timeout: " + str(missing))
        else:
            backend = FakeBackend(profile)
            gateway = LiveGateway(profile, model, backend, emit)

            def feed():
                stamp = time.monotonic_ns()
                for side, v in backend.values.items():
                    gateway.feedback(side, v["q"], v["gripper_m"], stamp)

            feed()
        source = create_source(c, profile, emit)
        session = ExecutionSession(gateway, source, c["execution"])

        def guard():
            while not stop.wait(0.02):
                try:
                    gateway.watchdog()
                except Exception as exc:
                    errors.append("watchdog: " + str(exc))
                    return

        watchdog = threading.Thread(target=guard, name="workflow-watchdog", daemon=True)
        watchdog.start()
        print(
            json.dumps(
                {
                    "event": "workflow_starting",
                    "backend": c["backend"],
                    "output": str(root),
                    "duration_s": source.duration_s,
                    "initial_targets": source.initial,
                }
            ),
            flush=True,
        )
        session.start()
        period = 1 / c["execution"]["control_hz"]
        next_tick = time.monotonic()
        while session.state in ("ALIGNING", "RUNNING"):
            if errors or any(p.poll() is not None for p in children):
                session.stop("runtime child failed: " + str(errors), fault=True)
                break
            if not physical:
                feed()
            with gateway.lock:
                snapshot = {
                    "time_ns": time.monotonic_ns(),
                    "states": deepcopy(gateway.states),
                    "images": {},
                }
            if not physical:
                for i, role in enumerate(c["fake_device"]["camera_roles"]):
                    snapshot["images"][role] = {
                        "encoding": "rgb8",
                        "shape": [48, 64, 3],
                        "data": bytes([(i + int(time.monotonic() * 30)) % 256]) * 48 * 64 * 3,
                    }
            session.tick(snapshot)
            next_tick += period
            # No burst catch-up when the OS delays the local control loop.
            now = time.monotonic()
            if next_tick < now:
                next_tick = now
            stop.wait(max(0, next_tick - now))
        result.update(state=session.state, reason=session.reason, commands=session.seq)
    except KeyboardInterrupt:
        if session:
            session.stop("operator_interrupt")
        result.update(state="STOPPED", reason="operator_interrupt")
    except Exception as exc:
        if session:
            session.stop(str(exc), fault=True)
        result.update(state="FAULT", reason=str(exc))
    finally:
        if session:
            try:
                session.close()
            except Exception as exc:
                errors.append("session cleanup: " + str(exc))
        elif source:
            try:
                source.close()
            except Exception as exc:
                errors.append(str(exc))
        stop.set()
        for thread in (watchdog, ros_thread):
            if thread:
                thread.join(timeout=2.0)
                if thread.is_alive():
                    errors.append(thread.name + " still running")
        for close in (
            ([lambda: executor.shutdown(timeout_sec=1.0)] if executor else [])
            + ([node.destroy_node] if node else [])
            + ([context.try_shutdown] if context else [])
        ):
            try:
                close()
            except Exception as exc:
                errors.append("ROS cleanup: " + str(exc))
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=2)
        for log in logs:
            log.close()
        result.update(
            cleanup_errors=errors,
            child_exit_codes=[p.poll() for p in children],
            stop_semantics="fresh measured arm hold; no home, disable or jaw-close on exit",
        )
        if errors:
            result.update(state="FAULT")
        try:
            emit("workflow_ended", result)
        except Exception as exc:
            errors.append("journal emit: " + str(exc))
            result["state"] = "FAULT"
        try:
            journal.close()
        except Exception as exc:
            errors.append("journal cleanup: " + str(exc))
            result["state"] = "FAULT"
        (root / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
    return result
