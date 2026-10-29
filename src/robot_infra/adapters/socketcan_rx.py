"""Receive-only classic SocketCAN samples and explicit Piper-v2 joint decoding.

No SDK, send API, mode change, interface activation or motor enable is used.
"""

import argparse
import json
import math
import select
import socket
import struct
import time
from datetime import datetime, timezone
from pathlib import Path


class PiperV2Joints:
    """Feedback 0x2A5..0x2A7: two signed big-endian 0.001 degree values.

    Matches the audited Piper SDK0.6.1 feedback contract; raw frames are retained.
    Per-pair receipt times are preserved, rather than pretending atomic state.
    """

    def __init__(self):
        self.pairs = {}

    def update(self, can_id, payload, received_ns):
        if can_id not in (0x2A5, 0x2A6, 0x2A7) or len(payload) != 8:
            return None
        self.pairs[can_id] = (struct.unpack(">ii", payload), received_ns)
        if len(self.pairs) < 3:
            return None
        values = [value for key in sorted(self.pairs) for value in self.pairs[key][0]]
        times = [self.pairs[key][1] for key in sorted(self.pairs)]
        return {
            "decoder": "piper-v2-joints/sdk0.6.1",
            "joint_names": ["joint" + str(i) for i in range(1, 7)],
            "raw_millidegrees": values,
            "position_rad": [x * math.pi / 180000 for x in values],
            "pair_received_monotonic_ns": times,
            "assembly_skew_ns": max(times) - min(times),
            "timestamp_domain": "host_receive; no device clock in these payloads",
        }


def receive_sample(channels, output, duration=3, max_frames=600, decoders=None):
    if not channels or len(set(channels.values())) != len(channels):
        raise ValueError("explicit unique interface per role required")
    if not 0 < duration <= 30 or type(max_frames) is not int or not 1 <= max_frames <= 10000:
        raise ValueError("invalid receive sample bounds")
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    state = {
        role: {
            "interface": name,
            "frames_seen": 0,
            "frames_saved": 0,
            "ids": {},
            "joint_state": None,
            "error": None,
        }
        for role, name in channels.items()
    }
    sockets = {}
    decoders = decoders or {}
    if not set(decoders) <= channels.keys():
        raise ValueError("decoder roles must name configured channels")
    try:
        for role, name in channels.items():
            connection = None
            try:
                connection = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                sockets[connection] = role
                connection.bind((name,))
                connection.setblocking(False)
            except (OSError, AttributeError) as exc:
                state[role]["error"] = str(exc)
                if connection is not None and connection in sockets:
                    sockets.pop(connection)
                    connection.close()
        with (root / "raw-can.jsonl").open("x", encoding="utf-8") as stream:
            deadline = time.monotonic() + duration
            while sockets and time.monotonic() < deadline:
                readable, _, _ = select.select(
                    list(sockets), [], [], min(0.1, max(0, deadline - time.monotonic()))
                )
                for connection in readable:
                    role = sockets[connection]
                    try:
                        frame = connection.recv(16)
                    except BlockingIOError:
                        continue
                    except OSError as exc:
                        state[role]["error"] = str(exc)
                        sockets.pop(connection)
                        connection.close()
                        continue
                    if len(frame) != 16:
                        continue
                    raw_id, size, data = struct.unpack("=IB3x8s", frame)
                    if size > 8:
                        continue
                    received = time.monotonic_ns()
                    data = data[:size]
                    item = state[role]
                    item["frames_seen"] += 1
                    key = hex(raw_id)
                    item["ids"][key] = item["ids"].get(key, 0) + 1
                    # Flags are retained in raw_id; only standard data feedback decoded.
                    decoded = (
                        decoders[role].update(raw_id, data, received) if role in decoders else None
                    )
                    if decoded:
                        item["joint_state"] = decoded
                    if item["frames_saved"] < max_frames:
                        stream.write(
                            json.dumps(
                                {
                                    "schema_version": "can-rx-sample/1",
                                    "role": role,
                                    "interface": item["interface"],
                                    "raw_can_id": raw_id,
                                    "data_hex": data.hex(),
                                    "host_monotonic_ns": received,
                                    "host_received_utc": datetime.now(timezone.utc).isoformat(),
                                }
                            )
                            + "\n"
                        )
                        item["frames_saved"] += 1
    finally:
        for connection in sockets:
            connection.close()
    report = {
        "schema_version": "can-rx-sample/1",
        "mode": "receive_only",
        "sdk_initialization": "not_run",
        "socket_send_calls": 0,
        "channels": state,
        "all_have_frames": all(x["frames_seen"] > 0 for x in state.values()),
        "all_have_joint_feedback": all(x["joint_state"] is not None for x in state.values()),
    }
    (root / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", action="append", required=True, metavar="ROLE=INTERFACE")
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=3)
    parser.add_argument("--decoder", choices=("none", "piper-v2-joints"), default="none")
    args = parser.parse_args(argv)
    channels = {}
    for text in args.channel:
        role, sep, interface = text.partition("=")
        if not sep or not role or not interface or role in channels:
            parser.error("unique ROLE=INTERFACE entries required")
        channels[role] = interface
    decoders = {r: PiperV2Joints() for r in channels} if args.decoder == "piper-v2-joints" else None
    report = receive_sample(channels, args.output, args.duration, decoders=decoders)
    print(json.dumps(report))
    return (
        0 if (report["all_have_joint_feedback"] if decoders else report["all_have_frames"]) else 3
    )


if __name__ == "__main__":
    raise SystemExit(main())
