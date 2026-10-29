"""Bounded WSS echo profiling using the production listener; no robot or ROS."""

import argparse
import json
import math
import ssl
import threading
import time
from pathlib import Path
from types import SimpleNamespace


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["server", "client"])
    p.add_argument("--output", required=True)
    p.add_argument("--cert", required=True)
    p.add_argument("--key")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8446)
    p.add_argument("--web-port", type=int, default=8445)
    p.add_argument("--duration", type=float, default=30)
    p.add_argument("--hz", type=float, default=30)
    p.add_argument("--url")
    p.add_argument("--origin")
    p.add_argument("--payload-bytes", type=int, default=2000)
    args = p.parse_args(argv)
    if not 1 <= args.duration <= 3600 or not math.isfinite(args.duration):
        p.error("duration must be 1..3600")
    if not 1 <= args.hz <= 240 or not 0 <= args.payload_bytes <= 32000:
        p.error("invalid rate/payload size")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        p.error("output already exists")
    result = {
        "mode": args.mode,
        "robot_commands": False,
        "schema_version": "input-transport-benchmark/1",
    }
    if args.mode == "server":
        if not args.key:
            p.error("server requires --key")
        from robot_infra.control_socket import ControlSocketServer

        class App:
            session = SimpleNamespace(viewer=None)

            def __init__(self):
                self.count = 0

            def input(self, packet):
                self.count += 1
                return {"accepted": True, "diagnostic_only": True}

            def pause(self, reason):
                pass

        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(args.cert, args.key)
        app = App()
        server = ControlSocketServer(app, args.host, args.port, ctx, args.web_port)
        try:
            print(
                json.dumps({"ready": True, "port": server.port, "robot_commands": False}),
                flush=True,
            )
            threading.Event().wait(args.duration)
        except KeyboardInterrupt:
            pass
        finally:
            server.close()
        result["inputs"] = app.count
    else:
        if not args.url or not args.origin:
            p.error("client requires --url and --origin")
        from websockets.sync.client import connect

        ctx = ssl.create_default_context(cafile=args.cert)
        samples = []
        errors = []
        server_ms = []
        began = time.monotonic()
        try:
            with connect(
                args.url, origin=args.origin, ssl=ctx, proxy=None, open_timeout=5, close_timeout=1
            ) as ws:
                assert json.loads(ws.recv(timeout=5))["type"] == "ready"
                while time.monotonic() - began < args.duration:
                    packet = {
                        "session_id": "network-diagnostic",
                        "seq": len(samples),
                        "padding": "x" * args.payload_bytes,
                    }
                    sent = time.monotonic()
                    ws.send(json.dumps(packet))
                    reply = json.loads(ws.recv(timeout=1))
                    rtt = (time.monotonic() - sent) * 1000
                    if reply.get("type") != "ack" or reply.get("seq") != packet["seq"]:
                        raise ValueError("bad ACK")
                    samples.append(rtt)
                    server_ms.append(reply["server_timing"]["input_ms"])
                    time.sleep(max(0, 1 / args.hz - (time.monotonic() - sent)))
        except Exception as exc:
            errors.append(str(exc))

        def stats(values):
            values = sorted(values)
            return (
                {
                    "samples": len(values),
                    "p50_ms": values[math.ceil(len(values) * 0.5) - 1],
                    "p95_ms": values[math.ceil(len(values) * 0.95) - 1],
                    "max_ms": max(values),
                }
                if values
                else None
            )

        result.update(
            url=args.url,
            elapsed_s=time.monotonic() - began,
            rtt=stats(samples),
            server_input=stats(server_ms),
            errors=errors,
            rtt_samples_ms=samples,
            payload_padding_bytes=args.payload_bytes,
            input_hz_target=args.hz,
            clock="client monotonic RTT; no cross-host clock subtraction",
        )
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf8")
    print(json.dumps({k: v for k, v in result.items() if k != "rtt_samples_ms"}), flush=True)
    return 1 if result.get("errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
