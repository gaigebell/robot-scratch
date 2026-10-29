"""Camera-only profiling: no model, arm driver or command publisher is created."""

import argparse
import json
import math
import os
import signal
import ssl
import subprocess
import threading
import time
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

from .camera_config import bind_cameras
from .camera_ingress import CameraIngress, subscribe_cameras
from .deployment import load_deployment
from .diagnostics import EventJournal
from .image_codec import preview_jpeg
from .live_teleop import make_server
from .performance import Performance
from .preview import PreviewWorker
from .real_episode import EpisodeRecorder


class ProbeApp:
    """The web API observes inputs only; it has no actuator backend."""

    diagnostic_only = True

    def __init__(self, config, journal, metrics):
        self.config, self.journal, self.performance = config, journal, metrics
        self.images = {}
        self.video_lock = threading.Lock()
        self.preview = None
        self.recorder = None
        self.errors = []
        self.session = SimpleNamespace(viewer=None)
        self.last_input = None
        self.reason = None
        self.input_count = 0
        self.client = {}
        self.ingress = None
        self.gateway = SimpleNamespace(stop=lambda reason, **kw: self.pause(reason))

    def pause(self, reason):
        self.reason = reason

    def input(self, packet):
        self.input_count += 1
        self.last_input = time.monotonic()
        self.session.viewer = packet.get("session_id")
        self.journal.emit("raw_input", packet)
        transport = packet.get("transport", {})
        for key in ("previous_rtt_ms", "previous_server_input_ms"):
            value = transport.get(key)
            if type(value) in (int, float):
                self.performance.observe("client." + key, value)
        return {"accepted": True, "diagnostic_only": True, "robot_commands": False}

    def status(self):
        cameras = {}
        now = time.monotonic()
        with self.video_lock:
            for role, p in self.config["camera_pipeline"].items():
                ticks = self.ingress.camera_ticks.get(role, ())
                fps = (
                    (len(ticks) - 1) / (ticks[-1] - ticks[0])
                    if len(ticks) > 1 and ticks[-1] > ticks[0]
                    else 0.0
                )
                cameras[role] = {
                    "profile": p["capture"]["rgb"],
                    "pipeline": p,
                    "fps": fps,
                    "age_ms": (now - self.images[role]["received_s"]) * 1000
                    if role in self.images
                    else None,
                }
        return {
            "schema_version": "live-teleop-status/1",
            "server_s": now,
            "profile": self.config,
            "session": {
                "state": "DIAGNOSTIC",
                "reason": "camera/input diagnostics; no robot commands",
            },
            "arms": {},
            "errors": self.errors,
            "cameras": cameras,
            "recording": self.recorder.snapshot() if self.recorder else None,
            "diagnostic_only": True,
            "input_count": self.input_count,
        }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--deployment", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--duration", type=float, default=60)
    p.add_argument("--preview-mode", choices=["off", "encode", "archive", "demand"], default="off")
    p.add_argument(
        "--image-reliability", choices=["best_effort", "reliable"], default="best_effort"
    )
    p.add_argument("--image-queue-depth", type=int, default=5)
    p.add_argument(
        "--serve",
        action="store_true",
        help="expose preview and input echo only; never control arms",
    )
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8445)
    args = p.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 3600:
        p.error("duration must be 1..3600 seconds")
    root = Path(args.output).expanduser()
    root.mkdir(parents=True, exist_ok=False)
    d = load_deployment(args.deployment)
    cameras = json.loads(Path(d["camera_config"]).read_text(encoding="utf-8-sig"))
    from .adapters.realsense_ros import driver_arguments, validate_config

    validate_config(cameras)
    c = bind_cameras(json.loads(Path(d["config"]).read_text(encoding="utf-8-sig")), cameras)
    from .camera_ingress import camera_transport_config

    c["camera_transport"] = camera_transport_config(
        {"image_reliability": args.image_reliability, "image_queue_depth": args.image_queue_depth}
    )
    c["performance"] = c.get("performance", {})
    c["preview_runtime"] = {
        "enabled": args.preview_mode != "off",
        "on_demand": args.preview_mode == "demand",
        "archive_jpeg": args.preview_mode == "archive",
    }
    c["recording_enabled"] = (
        False  # recording is controlled by the diagnostic timer, never by VR buttons
    )
    (root / "effective-config.json").write_text(json.dumps(c, indent=2) + "\n", encoding="utf8")
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    context = Context()
    rclpy.init(
        context=context,
        domain_id=d["ros_domain_id"],
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = rclpy.create_node("robot_infra_camera_benchmark", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    journal = EventJournal(root / "diagnostics")
    metrics = Performance(c["performance"])
    app = ProbeApp(c, journal, metrics)
    settings = deepcopy(c["recording"])
    settings["required_events"] = []
    recorder = EpisodeRecorder(
        settings, root / "data", "camera_diagnostic", 0, "error", metrics=metrics
    )
    app.recorder = recorder

    def publish(role, meta, jpeg, seq):
        with app.video_lock:
            app.images[role] = {
                "jpeg": jpeg,
                "received_s": meta["host_monotonic_ns"] / 1e9,
                "seq": seq,
                "capture_sequence": meta["capture_sequence"],
            }

    preview = PreviewWorker(
        c["camera_pipeline"],
        preview_jpeg,
        publish,
        metrics,
        config=c["preview_runtime"],
        archive_root=journal.root,
        emit=journal.emit,
    )
    app.preview = preview
    ingress = CameraIngress(c["camera_pipeline"], metrics, recorder=recorder, preview=preview)
    app.ingress = ingress
    app.video_lock = ingress.lock
    subscribe_cameras(node, c, ingress, journal.emit, app.errors)
    stop = threading.Event()
    children = []
    logs = []
    server = None
    control = None
    threads = []
    result = {
        "schema_version": "camera-benchmark/1",
        "camera_only": True,
        "robot_commands": False,
        "preview_mode": args.preview_mode,
        "duration_s": args.duration,
    }

    def spin():
        try:
            while not stop.is_set():
                executor.spin_once(timeout_sec=0.05)
        except Exception as exc:
            app.errors.append("spin: " + str(exc))
            stop.set()

    def wait(predicate, timeout):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if stop.is_set():
                raise RuntimeError("probe interrupted")
            if app.errors or any(child.poll() is not None for child in children):
                raise RuntimeError("camera child/ROS error: " + str(app.errors))
            if recorder.state == "ERROR":
                raise RuntimeError(str(recorder.snapshot()))
            if predicate():
                return
            time.sleep(0.02)
        raise TimeoutError("probe phase timeout")

    def terminate(signum, frame):
        stop.set()

    previous = {sig: signal.signal(sig, terminate) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(d["ros_domain_id"])
        for camera in cameras["cameras"]:
            log = (root / (camera["role"] + ".log")).open("x")
            logs.append(log)
            children.append(
                subprocess.Popen(
                    driver_arguments(d["camera_driver"], camera),
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        thread = threading.Thread(target=spin, name="camera-ros-spin", daemon=True)
        thread.start()
        threads.append(thread)
        expected = {
            (r, s)
            for r, pipeline in c["camera_pipeline"].items()
            for s in ("rgb", "depth")
            if pipeline["capture"][s]
        }

        def ready():
            with ingress.lock:
                return (
                    expected <= set(ingress.stream_last)
                    and all(time.monotonic() - ingress.stream_last[k] < 1 for k in expected)
                    and len(ingress.camera_info) == len(expected)
                )

        wait(ready, 45)
        from rclpy.utilities import get_rmw_implementation_identifier

        result["rmw"] = get_rmw_implementation_identifier()
        result["camera_transport"] = c["camera_transport"]
        result["image_publishers"] = {}
        for role, pipeline in c["camera_pipeline"].items():
            for stream in ("rgb", "depth"):
                if not pipeline["capture"][stream]:
                    continue
                topic = (
                    c["camera_topics"][role]
                    if stream == "rgb"
                    else "/" + role + "/depth/image_rect_raw"
                )
                result["image_publishers"][topic] = [
                    {
                        "node": info.node_name,
                        "reliability": info.qos_profile.reliability.name,
                        "depth": info.qos_profile.depth,
                    }
                    for info in node.get_publishers_info_by_topic(topic)
                ]
        if args.serve:
            tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            tls.load_cert_chain(d["cert"], d["key"])
            server = make_server((args.host, args.port), app)
            server.socket = tls.wrap_socket(server.socket, server_side=True)
            thread = threading.Thread(target=server.serve_forever, name="camera-https", daemon=True)
            thread.start()
            threads.append(thread)
            if c.get("webxr_transport", {}).get("input") == "websocket":
                from .control_socket import ControlSocketServer

                control = ControlSocketServer(
                    app, args.host, c["webxr_transport"]["port"], tls, args.port
                )
            elif c.get("webxr_transport", {}).get("input") == "webrtc":
                from .control_webrtc import WebRTCInputServer

                control = WebRTCInputServer(app)
                app.webrtc = control
            print(
                json.dumps(
                    {
                        "ready": True,
                        "camera_only": True,
                        "port": args.port,
                        "duration_s": args.duration,
                    }
                ),
                flush=True,
            )
        recorder.start(
            {
                "camera_pipeline": c["camera_pipeline"],
                "arms": [],
                "camera_info": deepcopy(ingress.camera_info),
                "purpose": "camera-only performance diagnostic",
                "config": c,
            }
        )
        wait(lambda: recorder.state == "RECORDING", 20)
        began = time.monotonic()
        cpu = time.process_time()
        wait(lambda: time.monotonic() - began >= args.duration, args.duration + 5)
        result["recording_wall_s"] = time.monotonic() - began
        result["process_cpu_s"] = time.process_time() - cpu
        recorder.finish()
        wait(lambda: recorder.state == "REVIEW", 120)
        snap = recorder.snapshot()
        path = Path(snap["pending_path"])
        result["recording"] = snap
        from .episode_qc import quality_report

        result["deep_qc"] = quality_report(
            path,
            json.loads((path / "manifest.json").read_text()),
            deep=True,
            output_root=root / "deep-qc",
        )
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        stop.set()
        for name, fn in [
            ("control", lambda: control.close() if control else None),
            ("http", lambda: server.shutdown() if server else None),
            ("preview", preview.close),
            ("recorder", recorder.close),
        ]:
            try:
                fn()
            except Exception as exc:
                app.errors.append(name + " close: " + str(exc))
        if server:
            server.server_close()
        for thread in threads:
            thread.join(timeout=3)
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
        context.try_shutdown()
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=3)
        for log in logs:
            log.close()
        result.update(
            metrics=metrics.snapshot(),
            preview=preview.snapshot(),
            errors=app.errors,
            camera_exit_codes=[child.poll() for child in children],
            input_count=app.input_count,
            source_rewinds=dict(ingress.source_rewinds),
            thread_alive=[thread.name for thread in threads if thread.is_alive()],
        )
        journal.close()
        (root / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf8")
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    print(
        json.dumps(
            {
                "output": str(root),
                "error": result.get("error"),
                "qc_passed": result.get("deep_qc", {}).get("passed"),
                "camera_exit_codes": result["camera_exit_codes"],
                "robot_commands": False,
            }
        ),
        flush=True,
    )
    return (
        1
        if result.get("error")
        or app.errors
        or result["thread_alive"]
        or any(code != 0 for code in result["camera_exit_codes"])
        or not result.get("deep_qc", {}).get("passed")
        else 0
    )


if __name__ == "__main__":
    raise SystemExit(main())
