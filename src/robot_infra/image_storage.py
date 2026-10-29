"""Robot/transport-independent image persistence backends.

Called only by the episode disk worker. Source clocks never become video clocks.
"""

import hashlib
import json
import os
from copy import deepcopy
from dataclasses import dataclass
from fractions import Fraction
from io import BytesIO
from pathlib import Path
from typing import Protocol

from .image_codec import encode_image, validate_image


def storage_config(value=None):
    value = {} if value is None else value
    if not isinstance(value, dict) or set(value) - {"image_backend", "video"}:
        raise ValueError("invalid image storage configuration")
    backend = value.get("image_backend", "raw_mcap")
    if backend not in ("raw_mcap", "mp4"):
        raise ValueError("unsupported image storage backend")
    defaults = {
        "codec": "libx264",
        "pixel_format": "yuv420p",
        "options": {"crf": "20", "preset": "veryfast"},
        "gop_size": 30,
        "threads": 1,
    }
    supplied = value.get("video", {})
    if not isinstance(supplied, dict) or set(supplied) - set(defaults):
        raise ValueError("invalid video encoder configuration")
    video = {**defaults, **deepcopy(supplied)}
    if any(not isinstance(video[k], str) or not video[k] for k in ("codec", "pixel_format")):
        raise ValueError("explicit encoder and pixel format required")
    for key, maximum in [("gop_size", 10000), ("threads", 64)]:
        if type(video[key]) is not int or not 1 <= video[key] <= maximum:
            raise ValueError("invalid encoder " + key)
    if not isinstance(video["options"], dict) or any(
        not isinstance(k, str) or not k or not isinstance(v, str)
        for k, v in video["options"].items()
    ):
        raise ValueError("encoder options must be string pairs")
    return {"image_backend": backend, "video": video}


def av_module():
    try:
        import av
    except ImportError as exc:
        raise ValueError(
            "MP4 storage requires the optional mp4 extra (PyAV); install robot-infra[mp4] in the device environment"
        ) from exc
    return av


def configure_encoder(stream, profile, settings):
    stream.width = profile["width"]
    stream.height = profile["height"]
    stream.pix_fmt = settings["pixel_format"]
    context = stream.codec_context
    context.time_base = Fraction(1, profile["fps"])
    context.gop_size = settings["gop_size"]
    context.max_b_frames = 0
    context.thread_count = settings["threads"]
    context.options = dict(settings["options"])


def check_image_storage(value=None, pipeline=None):
    config = storage_config(value)
    if config["image_backend"] == "raw_mcap":
        return config
    av = av_module()
    settings = config["video"]
    try:
        codec = av.Codec(settings["codec"], "w")
        if codec.type != "video":
            raise ValueError("video encoder required")
        profiles = {
            tuple(p["record"]["rgb"][k] for k in ("width", "height", "fps"))
            for p in (pipeline or {}).values()
            if p["record"].get("rgb")
        }
        for width, height, fps in profiles:
            if settings["pixel_format"] == "yuv420p" and (width % 2 or height % 2):
                raise ValueError(
                    "yuv420p requires even recording dimensions; choose an explicit compatible profile"
                )
            # Exercise the encoder AND MP4 muxer in memory before devices start.
            with av.open(BytesIO(), mode="w", format="mp4") as container:
                stream = container.add_stream(settings["codec"], rate=fps)
                configure_encoder(stream, {"width": width, "height": height, "fps": fps}, settings)
                container.start_encoding()
                if stream.codec_context.options:
                    raise ValueError(
                        "unrecognized encoder options: " + ",".join(stream.codec_context.options)
                    )
    except Exception as exc:
        raise ValueError("MP4 encoder preflight failed: " + str(exc)) from exc
    return config


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass
class StoredFrame:
    kind: str
    body: bytes
    index: dict


class ImageStorage(Protocol):
    def frame(self, name, meta, raw, profile) -> StoredFrame: ...
    def finish(self) -> dict: ...
    def abort(self) -> None: ...


class RawMCAPStorage:
    def frame(self, name, meta, raw, profile):
        return StoredFrame("image", encode_image(meta, raw), {})

    def finish(self):
        return {}

    def abort(self):
        pass


class MP4Storage:
    def __init__(self, root, config):
        self.root = Path(root)
        self.config = config
        self.av = av_module()
        self.videos = {}

    def frame(self, name, meta, raw, profile):
        # Native depth remains byte-exact in MCAP, independent of RGB selection.
        if meta["encoding"] not in ("rgb8", "bgr8"):
            return RawMCAPStorage().frame(name, meta, raw, profile)
        validate_image(meta, raw)
        geometry = (meta["width"], meta["height"], profile["fps"])
        if name not in self.videos:
            relative = "videos/stream_" + hashlib.sha256(name.encode()).hexdigest()[:32] + ".mp4"
            path = self.root / relative
            path.parent.mkdir(exist_ok=True)
            if path.exists():
                raise ValueError("video path already exists")
            container = self.av.open(str(path), mode="w", format="mp4")
            try:
                stream = container.add_stream(self.config["video"]["codec"], rate=profile["fps"])
                configure_encoder(stream, profile, self.config["video"])
                container.start_encoding()
                if stream.codec_context.options:
                    raise ValueError("unrecognized encoder options")
            except BaseException:
                container.close()
                raise
            self.videos[name] = {
                "path": relative,
                "container": container,
                "stream": stream,
                "count": 0,
                "geometry": geometry,
            }
        value = self.videos[name]
        if value["geometry"] != geometry:
            raise ValueError("image geometry changed within video stream")
        index = value["count"]
        frame = self.av.VideoFrame(
            meta["width"], meta["height"], "rgb24" if meta["encoding"] == "rgb8" else "bgr24"
        )
        plane = frame.planes[0]
        pixels = bytearray(plane.buffer_size)
        row_bytes = meta["width"] * 3
        for row in range(meta["height"]):
            pixels[row * plane.line_size : row * plane.line_size + row_bytes] = raw[
                row * meta["step"] : row * meta["step"] + row_bytes
            ]
        plane.update(pixels)
        frame.pts = index
        frame.time_base = Fraction(1, profile["fps"])
        for packet in value["stream"].encode(frame):
            value["container"].mux(packet)
        value["count"] += 1
        media = {
            "path": value["path"],
            "frame_index": index,
            "pts": index,
            "time_base": [1, profile["fps"]],
        }
        body = json.dumps(
            {"schema_version": "robot-infra-video-frame/1", "image": meta, "media": media},
            allow_nan=False,
            separators=(",", ":"),
        ).encode()
        return StoredFrame("video", body, {"media": media})

    def finish(self):
        assets = {}
        for name, value in self.videos.items():
            container = value["container"]
            for packet in value["stream"].encode():
                container.mux(packet)
            container.close()
            value["container"] = None
            path = self.root / value["path"]
            with path.open("r+b") as output:
                output.flush()
                os.fsync(output.fileno())
            with self.av.open(str(path)) as source:
                stream = source.streams.video[0]
                codec = stream.codec_context.name
                if stream.frames != value["count"]:
                    raise ValueError("MP4 finalized frame count mismatch")
            width, height, fps = value["geometry"]
            assets[name] = {
                "path": value["path"],
                "frames": value["count"],
                "width": width,
                "height": height,
                "fps": fps,
                "encoder": self.config["video"]["codec"],
                "codec": codec,
                "pixel_format": self.config["video"]["pixel_format"],
                "pts_policy": "dense_cfr; original source/receive times in MCAP and index",
                "time_base": [1, fps],
                "bytes": path.stat().st_size,
                "sha256": file_sha256(path),
            }
        return assets

    def abort(self):
        for value in self.videos.values():
            if value["container"] is not None:
                try:
                    value["container"].close()
                except Exception:
                    pass
                value["container"] = None


def create_image_storage(root, config) -> ImageStorage:
    return RawMCAPStorage() if config["image_backend"] == "raw_mcap" else MP4Storage(root, config)
