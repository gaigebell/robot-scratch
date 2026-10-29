"""Experimental bounded A4-S device-host teleop server; explicit live profile.

Run with ROS environment sourced. No vendor SDK imports or driver auto-launch.
"""

import argparse
import json
import math
import signal
import socket
import ssl
import threading
import time
from collections import Counter, deque
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .diagnostics import EventJournal, diagnostic_config
from .kinematics import SerialModel
from .live_control import LiveGateway, RelativePositionMapper
from .performance import Performance, performance_config
from .preview import PreviewWorker, preview_config


def validate_profile(c):
    from .camera_ingress import camera_transport_config

    camera_transport_config(c.get("camera_transport"))
    diagnostic_config(c.get("diagnostics"))
    performance_config(c.get("performance"))
    preview_config(c.get("preview_runtime"))
    from .pose_feedback import validate_pose_sources

    validate_pose_sources(c.get("pose_feedback"))
    if c.get("schema_version") == "live-teleop/2":
        from .teleop_session import validate_session_profile

        return validate_session_profile(c)
    if c.get("schema_version") != "live-teleop/1" or c.get("gripper_control") is not False:
        raise ValueError("this live slice requires explicit arm-only profile")
    if set(c["arms"]) != {"left", "right"}:
        raise ValueError("this slice requires named left/right arm endpoints")
    for k in (
        "scale",
        "max_joint_speed_rad_s",
        "max_tracking_error_rad",
        "feedback_timeout_s",
        "input_timeout_s",
        "ik_deadline_ms",
    ):
        if type(c[k]) not in (int, float) or not math.isfinite(c[k]) or c[k] <= 0:
            raise ValueError("invalid live bound: " + k)
    for k in ("max_cartesian_delta_m", "max_joint_excursion_rad"):
        if c[k] is not None and (
            type(c[k]) not in (int, float) or not math.isfinite(c[k]) or c[k] <= 0
        ):
            raise ValueError("test bound must be positive or explicitly null: " + k)
    tolerance = c.get("measurement_tolerance_rad", 0.0)
    if type(tolerance) not in (int, float) or not 0 <= tolerance <= 0.01:
        raise ValueError("measurement tolerance must be explicit and <= .01 rad")
    if type(c["clutch_button"]) is not int or not 0 <= c["clutch_button"] < 64:
        raise ValueError("explicit clutch button required")
    a = c["alignment"]
    if len(a) != 3 or any(
        len(r) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in r)
        for r in a
    ):
        raise ValueError("explicit finite 3x3 alignment required")
    if any(
        abs(sum(a[i][k] * a[j][k] for k in range(3)) - (1 if i == j else 0)) > 1e-6
        for i in range(3)
        for j in range(3)
    ):
        raise ValueError("alignment must be orthonormal")
    determinant = sum(
        a[0][i] * (a[1][(i + 1) % 3] * a[2][(i + 2) % 3] - a[1][(i + 2) % 3] * a[2][(i + 1) % 3])
        for i in range(3)
    )
    if abs(determinant - 1) > 1e-6:
        raise ValueError("alignment must be a rotation")
    if not 1 <= c["driver_speed_percent"] <= 100:
        raise ValueError("legacy driver speed must be 1..100 percent")
    topics = [v[k] for v in c["arms"].values() for k in ("command_topic", "enable_topic")]
    if len(set(topics)) != len(topics) or any(not x.startswith("/") for x in topics):
        raise ValueError("distinct absolute driver command topics required")
    return c


def create_model(config):
    if config["schema_version"] == "live-teleop/2":
        from .pinocchio_model import PinocchioModel

        return PinocchioModel(**config["model"], weights=config["ik"].get("weights"))
    return SerialModel(**config["model"])


class LiveApp:
    def __init__(self, config, model, backend, journal):
        self.config, self.model, self.journal = config, model, journal
        self.performance = Performance(config.get("performance"))
        self.preview = None
        self.recorder = None
        if config.get("recording_enabled"):
            from .real_episode import EpisodeRecorder

            self.recorder = EpisodeRecorder(
                config["recording"], **config["collection_request"], metrics=self.performance
            )
        self.gateway = LiveGateway(config, model, backend, self.emit)
        self.mapper = RelativePositionMapper(self.gateway)
        self.images, self.camera_info, self.status_messages = {}, {}, {}
        self.video_lock = threading.Lock()
        self.last_input, self.errors = None, []
        self.keep_running = True
        self.camera_ticks = {}
        self.preview_ticks = {}
        self.stream_last = {}
        self.observed_profiles = {}
        self.capture_sequences = Counter()
        self.input_service_ready = config.get("webxr_transport", {}).get("input", "http") == "http"
        self.session = None
        if config["schema_version"] == "live-teleop/2":
            from .solver_worker import SolverProcess
            from .teleop_session import TeleopSession

            workers = {}
            try:
                for side in config["arms"]:
                    workers[side] = SolverProcess(config["model"], config["ik"])
                self.session = TeleopSession(
                    self.gateway,
                    workers,
                    ready_check=self.preflight,
                    recorder=self.recorder,
                    recording_context=self.recording_context,
                    metrics=self.performance,
                )
                self.session.start()
            except Exception:
                for worker in workers.values():
                    worker.close()
                if self.recorder:
                    self.recorder.close()
                raise
        self.watchdog_thread = threading.Thread(target=self._watchdog, daemon=True)
        self.watchdog_thread.start()

    def emit(self, kind, payload):
        event = self.journal.emit(kind, payload)
        if self.recorder and kind != "camera_preview":
            self.recorder.event(kind, payload, event["host_monotonic_ns"], event["host_unix_ns"])

    def recording_context(self):
        with self.gateway.lock, self.video_lock:
            return {
                "camera_pipeline": self.config["camera_pipeline"],
                "arms": list(self.config["arms"]),
                "pose_feedback": self.config.get("pose_feedback", {}),
                "initial_targets": deepcopy(self.gateway.published_targets),
                "config": self.config,
                "camera_info": dict(self.camera_info),
                "arm_status": dict(self.status_messages),
                "clock_notes": "ROS header stamps and RealSense metadata preserved; alignment uses host received monotonic time",
            }

    def preflight(self):
        missing = []
        if not self.input_service_ready:
            missing.append("control_input_service")
        if hasattr(self.gateway.backend, "ready"):
            ok, errors = self.gateway.backend.ready()
            if not ok:
                missing += errors
        with self.video_lock:
            for role in self.config["camera_topics"]:
                # Capture readiness is independent of preview subscription/encoding.
                received = self.stream_last.get(
                    (role, "rgb"), self.images.get(role, {}).get("received_s", 0)
                )
                if time.monotonic() - received > 1.5:
                    missing.append(role)
            for role, p in self.config.get("camera_pipeline", {}).items():
                if (
                    p["capture"]["depth"]
                    and time.monotonic() - self.stream_last.get((role, "depth"), 0) > 1.5
                ):
                    missing.append(role + ".depth")
        return not missing, missing

    def pause(self, reason):
        if self.session:
            self.session.request_pause(reason)
        else:
            self.gateway.stop(reason)
            self.mapper.reset()

    def close(self):
        if self.session:
            self.session.close()
        else:
            self.gateway.stop("server_shutdown")
        self.keep_running = False
        try:
            if self.preview:
                self.preview.close()
        finally:
            if self.recorder:
                self.recorder.close()

    def _watchdog(self):
        while self.keep_running:
            try:
                self.gateway.watchdog()
            except Exception as exc:
                self.errors = (self.errors + [str(exc)])[-10:]
                # Even when the evidence sink fails, attempt a measured hold.
                with self.gateway.lock:
                    self.gateway.emit = lambda *args: None
                    self.gateway.stop("journal_or_watchdog_failure", fault=True)
            time.sleep(0.02)

    def status(self):
        g = self.gateway
        session = self.session.snapshot() if self.session else None
        with g.lock, self.video_lock:
            now = time.monotonic()
            return {
                "schema_version": "live-teleop-status/1",
                "server_s": now,
                "enabled": g.enabled,
                "owner": g.owner,
                "epoch": g.epoch,
                "faulted": g.faulted,
                "reason": g.last_reason,
                "input_age_ms": None if self.last_input is None else (now - self.last_input) * 1000,
                "arms": {
                    s: {
                        **v,
                        "age_ms": (now - v["device_changed_s"]) * 1000,
                        "fk_m": [r[3] for r in self.model.fk(v["q"])][:3],
                        "status": self.status_messages.get(s),
                        "nearest_limit": min(
                            (
                                {"joint": name, "remaining_rad": min(q - lo, hi - q)}
                                for name, q, (lo, hi) in zip(
                                    self.model.names, v["q"], self.model.limits
                                )
                            ),
                            key=lambda x: x["remaining_rad"],
                        ),
                    }
                    for s, v in g.states.items()
                },
                "cameras": {
                    s: {
                        "age_ms": (now - self.stream_last.get((s, "rgb"), v.get("received_s", 0)))
                        * 1000,
                        "frames": v.get("seq", 0),
                        "bytes": len(v.get("jpeg", b"")),
                        "fps": self._camera_fps(s),
                        "preview_fps": self._camera_fps(s, self.preview_ticks),
                        "profile": self.config.get("camera_profiles", {}).get(s),
                        "pipeline": self.config.get("camera_pipeline", {}).get(s),
                        "observed": {
                            stream: self.observed_profiles.get((s, stream))
                            for stream in ("rgb", "depth")
                        },
                    }
                    for s in self.config.get("camera_topics", self.images)
                    for v in [self.images.get(s, {})]
                },
                "active": self.mapper.active if session is None else session["active_sides"],
                "profile": self.config,
                "errors": self.errors,
                "session": session,
                "preview": self.preview.snapshot() if self.preview else None,
                "recording": self.recorder.snapshot()
                if self.recorder
                else {
                    "format": "live-teleop-event/1 JSONL + JPEG preview sidecars",
                    "raw_rgb_episode": False,
                },
            }

    def _camera_fps(self, role, source=None):
        ticks = (self.camera_ticks if source is None else source).get(role, ())
        return (
            (len(ticks) - 1) / (ticks[-1] - ticks[0])
            if len(ticks) > 1 and ticks[-1] > ticks[0]
            else 0.0
        )

    def input(self, packet):
        # A returned host timestamp bounds stalled HTTP transport without
        # pretending browser and host clocks have been synchronized.
        ticket = packet.get("last_server_s")
        if type(ticket) not in (int, float) or not 0 <= time.monotonic() - ticket < 0.8:
            raise ValueError("stale server freshness ticket")
        self.emit("raw_input", packet)
        self.last_input = time.monotonic()
        if self.session:
            self.session.ingest(packet)
            return {"accepted": True, "queued": True}
        return self.mapper.process(packet)


def make_server(address, app):
    page = (
        Path(__file__).parent
        / "web"
        / (
            "xr-session.html"
            if app.config["schema_version"] == "live-teleop/2"
            else "xr-teleop.html"
        )
    ).read_bytes()
    if getattr(app, "diagnostic_only", False):
        page = page.replace("双臂自由遥操".encode(), "相机与连接诊断（不控制机械臂）".encode())

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connection.settimeout(2)

        def log_message(self, *args):
            pass

        def reply(self, code, value, content_type="application/json"):
            body = (
                value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode()
            )
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Permissions-Policy", "xr-spatial-tracking=(self)")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                self.reply(200, page, "text/html; charset=utf-8")
            elif path == "/api/status":
                self.reply(200, app.status())
            elif path == "/api/performance":
                self.reply(
                    200,
                    {
                        "metrics": app.performance.snapshot(),
                        "preview": app.preview.snapshot() if app.preview else None,
                    },
                )
            elif path.startswith("/assets/"):
                relative = path.removeprefix("/assets/")
                allowed = {
                    "xr-session.js",
                    "xr-session.css",
                    "xr-transport.js",
                    "xr-scene.js",
                    "vendor/three.module.js",
                }
                if relative not in allowed:
                    self.reply(404, {"error": "unknown asset"})
                else:
                    data = (Path(__file__).parent / "web" / relative).read_bytes()
                    self.reply(
                        200, data, "text/css" if relative.endswith(".css") else "text/javascript"
                    )
            elif path.startswith("/camera/"):
                role = path.removeprefix("/camera/").removesuffix(".jpg")
                if app.preview:
                    app.preview.request(role)
                with app.video_lock:
                    frame = app.images.get(role)
                if frame and time.monotonic() - frame["received_s"] < 1.5:
                    # Age is measured entirely on the server, not across browser clocks.
                    body = frame["jpeg"]
                    began = time.monotonic_ns()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Frame-Sequence", str(frame["seq"]))
                    self.send_header("X-Capture-Sequence", str(frame.get("capture_sequence", "")))
                    self.send_header(
                        "X-Frame-Age-Ms", str((time.monotonic() - frame["received_s"]) * 1000)
                    )
                    self.end_headers()
                    self.wfile.write(body)
                    app.performance.elapsed("preview.http_write." + role, began)
                else:
                    self.reply(503, {"error": "camera unavailable or stale"})
            else:
                self.reply(404, {"error": "unknown path"})

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 65536:
                    raise ValueError("invalid body size")
                body = json.loads(self.rfile.read(length))
                if self.path == "/api/arm":
                    if app.session:
                        raise ValueError("use A button session workflow")
                    app.mapper.reset()
                    epoch = app.gateway.arm(body["session_id"])
                    self.reply(200, {"epoch": epoch, "server_s": time.monotonic()})
                elif self.path == "/api/stop":
                    app.pause(body.get("reason", "operator_pause"))
                    self.reply(200, {"pause_requested": True, "server_s": time.monotonic()})
                elif self.path == "/api/input":
                    result = app.input(body)
                    self.reply(
                        200,
                        {
                            "result": result,
                            "enabled": app.gateway.enabled,
                            "server_s": time.monotonic(),
                        },
                    )
                elif self.path == "/api/webrtc/offer":
                    from urllib.parse import urlsplit

                    origin = urlsplit(self.headers.get("Origin", ""))
                    if origin.scheme != "https" or origin.netloc != self.headers.get("Host"):
                        raise PermissionError("use this page origin for WebRTC negotiation")
                    listener = getattr(app, "webrtc", None)
                    if listener is None:
                        raise ValueError("WebRTC input not configured")
                    self.reply(200, listener.offer(body))
                elif self.path == "/api/client-event":
                    app.journal.emit("client_event", body)
                    self.reply(200, {"recorded": True, "server_s": time.monotonic()})
                else:
                    self.reply(404, {"error": "unknown path"})
            except (ValueError, KeyError, PermissionError) as exc:
                self.reply(400, {"error": str(exc), "server_s": time.monotonic()})
            except Exception as exc:
                app.gateway.stop("server_failure", fault=True)
                self.reply(503, {"error": str(exc)})

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--cert", required=True)
    p.add_argument("--key", required=True)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8445)
    p.add_argument("--duration", type=float, default=1800)
    args = p.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        p.error("duration must be nonnegative; 0 means until Ctrl+C")
    c = validate_profile(json.loads(Path(args.config).read_text(encoding="utf-8-sig")))
    model = create_model(c)
    import rclpy
    from piper_msgs.msg import PiperStatusMsg
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    from .adapters.piper_ros import PiperROSBackend
    from .image_codec import preview_jpeg

    # Let the application perform measured hold / journal finalize before the
    # ROS context is destroyed. Default ROS SIGINT closes that context early.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    node = rclpy.create_node("robot_infra_live_gateway")
    journal = EventJournal(args.output, **diagnostic_config(c.get("diagnostics")))
    app = LiveApp(c, model, PiperROSBackend(node, c, model.names), journal)
    app.emit(
        "manifest",
        {
            "config": c,
            "model_sha256": model.sha256,
            "mode": "physical_session" if app.session else "physical_arm_only",
            "raw_rgb_episode": bool(app.recorder),
        },
    )
    for side, arm in c["arms"].items():

        def joints(m, s=side):
            indexed = dict(zip(m.name, m.position))
            if len(indexed) != len(m.name):
                raise ValueError("duplicate feedback joint names")
            app.gateway.feedback(
                s,
                [indexed[n] for n in model.names],
                indexed["gripper"],
                m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
            )

        def status(m, s=side):
            app.status_messages[s] = {
                k: getattr(m, k) for k in ("ctrl_mode", "arm_status", "err_code")
            }
            if m.err_code or m.arm_status:
                app.gateway.stop("driver_error_" + s, fault=True)

        node.create_subscription(JointState, arm["feedback_topic"], joints, 1)
        node.create_subscription(PiperStatusMsg, arm["status_topic"], status, 1)
    from .adapters.ros_pose import subscribe_pose_feedback

    subscribe_pose_feedback(node, c.get("pose_feedback"), app.emit)
    from .camera_ingress import CameraIngress, subscribe_cameras

    pipeline = c.get("camera_pipeline") or {
        r: {
            "capture": {
                "rgb": c.get("camera_profiles", {}).get(
                    r, {"width": 640, "height": 480, "fps": 15}
                ),
                "depth": None,
            },
            "preview": {"width": 640, "height": 480, "fps": 8, "jpeg_quality": 75},
            "record": {},
        }
        for r in c["camera_topics"]
    }

    def publish_preview(role, meta, jpeg, seq):
        with app.video_lock:
            app.images[role] = {
                "jpeg": jpeg,
                "received_s": meta["host_monotonic_ns"] / 1e9,
                "seq": seq,
                "capture_sequence": meta["capture_sequence"],
            }
            app.preview_ticks.setdefault(role, deque(maxlen=90)).append(time.monotonic())

    app.preview = PreviewWorker(
        pipeline,
        preview_jpeg,
        publish_preview,
        app.performance,
        config=c.get("preview_runtime"),
        archive_root=journal.root,
        emit=journal.emit,
    )
    ingress = CameraIngress(pipeline, app.performance, recorder=app.recorder, preview=app.preview)
    app.video_lock = ingress.lock
    for key in (
        "stream_last",
        "observed_profiles",
        "capture_sequences",
        "camera_ticks",
        "camera_info",
    ):
        setattr(app, key, getattr(ingress, key))
    subscribe_cameras(node, c, ingress, app.emit, app.errors)

    def spin():
        try:
            while app.keep_running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as exc:
            if app.keep_running:
                app.errors = (app.errors + [str(exc)])[-10:]
                app.gateway.stop("ros_spin_failure", fault=True)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    server = make_server((args.host, args.port), app)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(args.cert, args.key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    timer = threading.Timer(args.duration, server.shutdown) if args.duration else None
    if timer:
        timer.daemon = True
        timer.start()
    control = None
    try:
        transport = c.get("webxr_transport", {})
        if transport.get("input") == "websocket":
            from .control_socket import ControlSocketServer

            control = ControlSocketServer(app, args.host, transport["port"], context, args.port)
            app.input_service_ready = True
        elif transport.get("input") == "webrtc":
            from .control_webrtc import WebRTCInputServer

            control = WebRTCInputServer(app)
            app.webrtc = control
            app.input_service_ready = True
        print(
            json.dumps(
                {
                    "address": server.server_address,
                    "mode": "teleop_session" if app.session else "live_bounded_arm_only",
                    "output": args.output,
                }
            ),
            flush=True,
        )
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if timer:
            timer.cancel()
        if control:
            control.close()
        try:
            app.close()
        except Exception as exc:
            app.errors = (app.errors + ["close: " + str(exc)])[-10:]
        server.server_close()
        app.watchdog_thread.join(2)
        spin_thread.join(2)
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            app.errors = (app.errors + [str(exc)])[-10:]
        finally:
            try:
                (journal.root / "performance.json").write_text(
                    json.dumps(
                        {"metrics": app.performance.snapshot(), "preview": app.preview.snapshot()},
                        indent=2,
                    )
                    + "\n",
                    encoding="utf8",
                )
                journal.emit("finalize", {"outcome": "diagnostic_only", "errors": app.errors})
            finally:
                journal.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
