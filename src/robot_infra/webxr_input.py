"""Independent HTTPS WebXR input recorder. No robot drivers or command ports."""

import argparse
import json
import math
import ssl
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

VERSION = "webxr-input/1"


def validate_packet(packet):
    if not isinstance(packet, dict) or packet.get("schema_version") != VERSION:
        raise ValueError("unsupported input schema")
    if not isinstance(packet.get("session_id"), str) or not 1 <= len(packet["session_id"]) <= 128:
        raise ValueError("session_id required")
    if type(packet.get("seq")) is not int or packet["seq"] < 0:
        raise ValueError("invalid sequence")
    if packet.get("event") not in {
        "capabilities",
        "session_start",
        "frame",
        "reset",
        "session_end",
        "error",
        "visibility",
    }:
        raise ValueError("unknown event")
    if packet.get("reference_space") not in {"none", "local", "local-floor"}:
        raise ValueError("reference space must be explicit")
    if type(packet.get("epoch")) is not int or packet["epoch"] < 0:
        raise ValueError("invalid reference epoch")
    for field in ("client_time_ms", "client_unix_ms"):
        v = packet.get(field)
        if type(v) not in (int, float) or not math.isfinite(v) or v < 0:
            raise ValueError("invalid client time")

    def pose(value):
        if value is None:
            return
        if not isinstance(value, dict):
            raise ValueError("pose must be null or an object")
        for key, count in (("position_m", 3), ("orientation_xyzw", 4)):
            values = value.get(key)
            if (
                not isinstance(values, list)
                or len(values) != count
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
            ):
                raise ValueError("invalid pose vector")

    pose(packet.get("head"))
    sources = packet.get("inputs")
    if not isinstance(sources, list) or len(sources) > 8:
        raise ValueError("invalid input sources")
    for source in sources:
        if not isinstance(source, dict) or source.get("handedness") not in {
            "left",
            "right",
            "none",
        }:
            raise ValueError("invalid handedness")
        pose(source.get("grip"))
        pose(source.get("target_ray"))
        gamepad = source.get("gamepad")
        if gamepad is not None:
            if (
                not isinstance(gamepad, dict)
                or not isinstance(gamepad.get("buttons"), list)
                or len(gamepad["buttons"]) > 64
            ):
                raise ValueError("invalid buttons")
            for button in gamepad["buttons"]:
                if (
                    not isinstance(button, dict)
                    or type(button.get("value")) not in (int, float)
                    or not math.isfinite(button["value"])
                    or not 0 <= button["value"] <= 1
                    or type(button.get("pressed")) is not bool
                    or type(button.get("touched")) is not bool
                ):
                    raise ValueError("invalid button")
            axes = gamepad.get("axes")
            if (
                not isinstance(axes, list)
                or len(axes) > 32
                or any(
                    type(v) not in (int, float) or not math.isfinite(v) or not -1 <= v <= 1
                    for v in axes
                )
            ):
                raise ValueError("invalid axes")


class InputRecorder:
    def __init__(self, path, *, max_sessions=16, max_bytes=128 * 1024 * 1024):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x", encoding="utf-8")
        self.lock = threading.Lock()
        self.sessions = {}
        self.max_sessions, self.max_bytes = max_sessions, max_bytes
        self.events = self.bytes = 0
        self.closed = False

    def accept(self, packet):
        validate_packet(packet)
        with self.lock:
            if self.closed:
                raise ValueError("recorder closed")
            sid = packet["session_id"]
            previous = self.sessions.get(sid)
            if previous and packet["seq"] <= previous["seq"]:
                raise ValueError("duplicate or reordered sequence")
            if previous is None and len(self.sessions) >= self.max_sessions:
                raise ValueError("session capacity reached")
            received = time.monotonic_ns()
            event = {
                "schema_version": VERSION,
                "host_received_utc": datetime.now(timezone.utc).isoformat(),
                "host_monotonic_ns": received,
                "clock_relation": "unmeasured",
                "raw_input": packet,
            }
            line = json.dumps(event, ensure_ascii=True, allow_nan=False) + "\n"
            if self.bytes + len(line.encode("utf-8")) > self.max_bytes:
                raise ValueError("recording capacity reached")
            self.stream.write(line)
            self.stream.flush()
            self.bytes += len(line.encode("utf-8"))
            self.events += 1
            self.sessions[sid] = {
                "seq": packet["seq"],
                "received_ns": received,
                "latest": packet,
                "events": (previous["events"] if previous else 0) + 1,
                "sequence_gaps": (previous["sequence_gaps"] if previous else 0)
                + (max(0, packet["seq"] - previous["seq"] - 1) if previous else 0),
            }

    def status(self):
        with self.lock:
            now = time.monotonic_ns()
            sessions = {
                sid: {
                    **state,
                    "age_ms": (now - state["received_ns"]) / 1e6,
                    "fresh": now - state["received_ns"] < 500_000_000
                    and state["latest"]["event"] == "frame",
                }
                for sid, state in self.sessions.items()
            }
            return {
                "mode": "input_only",
                "robot_commands": "unavailable",
                "events": self.events,
                "bytes": self.bytes,
                "closed": self.closed,
                "sessions": sessions,
            }

    def close(self):
        with self.lock:
            if not self.closed:
                self.stream.close()
                self.closed = True


def make_server(address, recorder):
    page = (Path(__file__).parent / "web" / "xr-input.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.settimeout(5)

        def log_message(self, *args):
            pass

        def reply(self, code, value, content_type="application/json"):
            body = (
                value
                if isinstance(value, bytes)
                else json.dumps(value, ensure_ascii=True, allow_nan=False).encode()
            )
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Permissions-Policy", "xr-spatial-tracking=(self)")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.reply(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/status":
                self.reply(200, recorder.status())
            else:
                self.reply(404, {"error": "unknown path"})

        def do_POST(self):
            if self.path != "/api/input":
                self.reply(404, {"error": "unknown path"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 65536:
                    raise ValueError("input body must be 1..65536 bytes")
                packet = json.loads(self.rfile.read(size))
                recorder.accept(packet)
                self.reply(200, {"accepted": True})
            except (ValueError, UnicodeDecodeError) as exc:
                self.reply(400, {"error": str(exc)})
            except OSError:
                self.reply(503, {"error": "recording I/O failed"})

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8445)
    parser.add_argument("--cert", required=True)
    parser.add_argument("--key", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=1800)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be positive and finite")
    recorder = InputRecorder(args.output)
    server = None
    timer = None
    try:
        server = make_server((args.host, args.port), recorder)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.cert, args.key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
        timer = threading.Timer(args.duration, server.shutdown)
        timer.daemon = True
        timer.start()
        print(
            json.dumps(
                {
                    "mode": "input_only",
                    "address": server.server_address,
                    "duration_s": args.duration,
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
        if server:
            server.server_close()
        recorder.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
