"""Small CPU-only HTTP policy fixture. Never imports a robot driver."""

import argparse
import base64
import http.client
import json
import threading
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .workflow import MockPolicy, finite


def wire_request(request, decode=False):
    value = deepcopy(request)
    for image in value["observation"]["images"].values():
        if decode:
            image["data"] = base64.b64decode(image.pop("data_base64"), validate=True)
        else:
            image["data_base64"] = base64.b64encode(image.pop("data")).decode("ascii")
    return value


class HTTPMockClient:
    def __init__(self, endpoint, timeout_s=0.4):
        self.url = urlsplit(endpoint)
        self.timeout = finite(timeout_s, "HTTP policy timeout")
        if (
            self.url.scheme != "http"
            or not self.url.hostname
            or self.url.path not in ("", "/")
            or self.url.query
            or self.url.username
        ):
            raise ValueError("mock endpoint must be an explicit http origin")

    def infer(self, request, cancel):
        if cancel.is_set():
            return None
        connection = http.client.HTTPConnection(
            self.url.hostname, self.url.port or 80, timeout=self.timeout
        )
        try:
            body = json.dumps(wire_request(request), allow_nan=False).encode()
            connection.request("POST", "/infer", body, {"Content-Type": "application/json"})
            response = connection.getresponse()
            data = response.read(2_000_001)
            if len(data) > 2_000_000:
                raise ValueError("oversized policy response")
            if response.status != 200:
                raise RuntimeError(
                    "policy HTTP "
                    + str(response.status)
                    + ": "
                    + data[:200].decode(errors="replace")
                )
            return None if cancel.is_set() else json.loads(data)
        finally:
            connection.close()


def make_policy_server(address, policy):
    cancel = threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path != "/healthz":
                self.send_error(404)
                return
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"mock":true,"hardware":false}')

        def do_POST(self):
            try:
                if self.path != "/infer":
                    self.send_error(404)
                    return
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2_000_000:
                    raise ValueError("invalid policy request size")
                request = wire_request(json.loads(self.rfile.read(size)), decode=True)
                result = policy.infer(request, cancel)
                data = json.dumps(result, allow_nan=False).encode()
                code = 200
            except Exception as exc:
                data = json.dumps({"error": str(exc)}).encode()
                code = 422
            try:
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    server.policy_cancel = cancel
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workflow", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29057)
    args = parser.parse_args(argv)
    path = Path(args.workflow).resolve()
    c = json.loads(path.read_text(encoding="utf-8-sig"))
    profile = Path(c["profile"])
    profile = profile if profile.is_absolute() else path.parent / profile
    p = json.loads(profile.read_text(encoding="utf-8-sig"))
    s = c["source"]
    initial = {
        side: {"q": list(a["home_rad"]), "gripper_m": a["gripper"]["limits_m"][0]}
        for side, a in p["arms"].items()
    }
    spec = {k: p["model"][k] for k in ("joint_names", "base", "tip")}
    spec.update(joint_unit="rad", gripper_unit="m")
    policy = MockPolicy(
        initial,
        action_spec=spec,
        **{
            k: s[k]
            for k in ("steps", "action_dt_s", "lead_s", "delay_s", "amplitude_rad", "fail_request")
        },
    )
    server = make_policy_server((args.host, args.port), policy)
    print(
        json.dumps(
            {"event": "mock_policy_listening", "address": server.server_address, "hardware": False}
        ),
        flush=True,
    )
    import signal

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        server.serve_forever(poll_interval=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        server.policy_cancel.set()
        server.server_close()


if __name__ == "__main__":
    main()
