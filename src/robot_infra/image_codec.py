"""ROS-independent lossless raw-image envelope and optional RGB resizing."""

import json
import struct
from io import BytesIO


def validate_image(meta, raw):
    sizes = {"rgb8": 3, "bgr8": 3, "16UC1": 2, "mono16": 2, "32FC1": 4}
    if meta.get("encoding") not in sizes:
        raise ValueError("unsupported raw image encoding")
    if any(type(meta.get(k)) is not int or meta[k] < 1 for k in ("width", "height", "step")):
        raise ValueError("invalid image dimensions")
    if (
        meta["step"] < meta["width"] * sizes[meta["encoding"]]
        or len(raw) != meta["step"] * meta["height"]
    ):
        raise ValueError("image stride/size mismatch")


def encode_image(meta, raw):
    validate_image(meta, raw)
    header = json.dumps(meta, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return struct.pack("<I", len(header)) + header + raw


def decode_image(data):
    if len(data) < 4:
        raise ValueError("truncated image envelope")
    length = struct.unpack("<I", data[:4])[0]
    if length > 1024 * 1024 or len(data) < 4 + length:
        raise ValueError("invalid image header length")
    meta = json.loads(data[4 : 4 + length])
    raw = data[4 + length :]
    validate_image(meta, raw)
    return meta, raw


def rgb_image(meta, raw):
    from PIL import Image

    validate_image(meta, raw)
    if meta["encoding"] not in ("rgb8", "bgr8"):
        raise ValueError("RGB required")
    return Image.frombytes(
        "RGB",
        (meta["width"], meta["height"]),
        raw,
        "raw",
        "RGB" if meta["encoding"] == "rgb8" else "BGR",
        meta["step"],
        1,
    )


def resize_rgb(meta, raw, width, height):
    if (width, height) == (meta["width"], meta["height"]):
        return meta, raw
    from PIL import Image

    result = rgb_image(meta, raw).resize((width, height), Image.Resampling.BILINEAR)
    return {
        **meta,
        "source_geometry": {k: meta[k] for k in ("width", "height", "step", "encoding")},
        "resize": {
            "method": "bilinear",
            "scale_x": width / meta["width"],
            "scale_y": height / meta["height"],
        },
        "width": width,
        "height": height,
        "step": width * 3,
        "encoding": "rgb8",
    }, result.tobytes()


def preview_jpeg(meta, raw, settings):
    from PIL import Image

    image = rgb_image(meta, raw)
    if image.size != (settings["width"], settings["height"]):
        image = image.resize((settings["width"], settings["height"]), Image.Resampling.BILINEAR)
    out = BytesIO()
    image.save(out, format="JPEG", quality=settings["jpeg_quality"])
    return out.getvalue()
