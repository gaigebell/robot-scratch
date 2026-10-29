"""Bounded diagnostic image samples, independent of camera drivers and ROS."""

import hashlib
import json
import struct
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path


def rgb_png(data, width, height, step, encoding):
    if (
        encoding not in {"rgb8", "bgr8"}
        or min(width, height) < 1
        or step < width * 3
        or len(data) != step * height
    ):
        raise ValueError("unsupported or inconsistent RGB image")
    rows = []
    for y in range(height):
        row = bytearray(data[y * step : y * step + width * 3])
        if encoding == "bgr8":
            row[0::3], row[2::3] = row[2::3], row[0::3]
        rows.append(b"\0" + row)

    def chunk(kind, body):
        return (
            struct.pack(">I", len(body))
            + kind
            + body
            + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)
        )

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(b"".join(rows), 2))
        + chunk(b"IEND", b"")
    )


class ImageSampleRecorder:
    def __init__(self, root, roles, max_frames=3):
        if type(max_frames) is not int or not 1 <= max_frames <= 100:
            raise ValueError("max_frames must be 1..100")
        if (
            not roles
            or len(set(roles)) != len(roles)
            or any(not isinstance(r, str) or not r.replace("_", "").isalnum() for r in roles)
        ):
            raise ValueError("unique alphanumeric camera roles required")
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.max_frames = max_frames
        self.state = {
            r: {
                "frames_seen": 0,
                "frames_saved": 0,
                "metadata_seen": 0,
                "camera_info": None,
                "errors": [],
            }
            for r in roles
        }
        self.index = (self.root / "observations.jsonl").open("x", encoding="utf-8")

    def event(self, role, kind, value):
        item = {
            "schema_version": "camera-sample/1",
            "role": role,
            "kind": kind,
            "host_received_utc": datetime.now(timezone.utc).isoformat(),
            "host_monotonic_ns": time.monotonic_ns(),
            "value": value,
        }
        self.index.write(json.dumps(item, allow_nan=False) + "\n")
        self.index.flush()

    def frame(self, role, data, width, height, step, encoding, header):
        state = self.state[role]
        state["frames_seen"] += 1
        if state["frames_saved"] >= self.max_frames:
            return
        preview = rgb_png(data, width, height, step, encoding)
        folder = self.root / role
        folder.mkdir(exist_ok=True)
        stem = str(state["frames_saved"]).zfill(4)
        raw = folder / (stem + ".raw")
        raw.write_bytes(data)
        (folder / (stem + ".png")).write_bytes(preview)
        self.event(
            role,
            "image",
            {
                "width": width,
                "height": height,
                "step": step,
                "encoding": encoding,
                "header": header,
                "raw_file": str(raw.relative_to(self.root)),
                "sha256": hashlib.sha256(data).hexdigest(),
                "timestamp_domain": "ros_header; device clock requires metadata",
            },
        )
        state["frames_saved"] += 1

    def camera_info(self, role, info):
        if self.state[role]["camera_info"] is None:
            self.event(role, "camera_info", info)
        self.state[role]["camera_info"] = info

    def metadata(self, role, header, raw_json):
        state = self.state[role]
        state["metadata_seen"] += 1
        if state["metadata_seen"] <= self.max_frames * 4:
            self.event(role, "device_metadata", {"header": header, "raw_json": raw_json})

    def close(self):
        self.index.close()
        result = {
            "schema_version": "camera-sample/1",
            "purpose": "bounded_diagnostic_sample_not_episode",
            "roles": self.state,
            "all_have_frames": all(x["frames_saved"] > 0 for x in self.state.values()),
        }
        (self.root / "summary.json").write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        return result
