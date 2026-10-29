"""Offline external-media integrity checks, shared by episode QC/readers."""

import re
from fractions import Fraction
from pathlib import Path, PurePosixPath

from .image_storage import av_module, file_sha256


def media_path(root, relative):
    if not isinstance(relative, str) or not relative or "\\" in relative:
        raise ValueError("invalid video path")
    p = PurePosixPath(relative)
    if p.is_absolute() or ".." in p.parts:
        raise ValueError("video path must stay inside episode")
    root = Path(root).resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("video file missing or outside episode")
    return path


def inspect_media(root, assets, streams, *, deep=False):
    result = {}
    if not isinstance(assets, dict):
        raise ValueError("media assets must be a stream mapping")
    if not assets:
        if any(row.get("media") for rows in streams.values() for row in rows):
            raise ValueError("video reference has no media asset")
        return result
    av = av_module()
    for name, asset in assets.items():
        if not isinstance(name, str) or not isinstance(asset, dict):
            raise ValueError("invalid media asset")
        if any(
            type(asset.get(k)) is not int or asset[k] < 1
            for k in ("frames", "width", "height", "fps", "bytes")
        ):
            raise ValueError("invalid video dimensions/count/rate/size")
        if asset.get("time_base") != [1, asset["fps"]]:
            raise ValueError("unsupported video time base")
        if (
            any(
                not isinstance(asset.get(k), str) or not asset[k] for k in ("codec", "pixel_format")
            )
            or not isinstance(asset.get("sha256"), str)
            or not re.fullmatch("[a-f0-9]{64}", asset["sha256"])
        ):
            raise ValueError("invalid video codec or hash metadata")
        path = media_path(root, asset["path"])
        rows = streams.get(name, [])
        if path.stat().st_size != asset["bytes"]:
            raise ValueError("video size mismatch: " + name)
        if len(rows) != asset["frames"]:
            raise ValueError("video/index frame count mismatch: " + name)
        for i, row in enumerate(rows):
            ref = row.get("media")
            if ref != {
                "path": asset["path"],
                "frame_index": i,
                "pts": i,
                "time_base": asset["time_base"],
            }:
                raise ValueError("video frame index/PTS mapping mismatch: " + name)
        try:
            with av.open(str(path)) as source:
                if len(source.streams.video) != 1:
                    raise ValueError("exactly one video stream required")
                video = source.streams.video[0]
                if video.frames != asset["frames"] or (video.width, video.height) != (
                    asset["width"],
                    asset["height"],
                ):
                    raise ValueError("video header geometry/count mismatch")
                if (
                    video.average_rate != Fraction(asset["fps"])
                    or video.codec_context.name != asset["codec"]
                    or video.codec_context.format.name != asset["pixel_format"]
                ):
                    raise ValueError("video header codec/profile mismatch")
                decoded = 0
                if deep:
                    if file_sha256(path) != asset["sha256"]:
                        raise ValueError("video hash mismatch")
                    for frame in source.decode(video=0):
                        if (
                            (frame.width, frame.height) != (asset["width"], asset["height"])
                            or frame.pts is None
                            or frame.time_base is None
                        ):
                            raise ValueError("decoded video frame geometry/PTS missing")
                        if frame.pts * frame.time_base != Fraction(decoded, asset["fps"]):
                            raise ValueError("decoded video PTS mismatch")
                        decoded += 1
                    if decoded != asset["frames"]:
                        raise ValueError("decoded video frame count mismatch")
        except Exception as exc:
            raise ValueError("video integrity failed for " + name + ": " + str(exc)) from exc
        result[name] = {
            "path": asset["path"],
            "frames": asset["frames"],
            "bytes": asset["bytes"],
            "header_checked": True,
            "decoded_frames": decoded if deep else None,
            "deep_checked": deep,
        }
    if any(
        row.get("media") and name not in assets for name, rows in streams.items() for row in rows
    ):
        raise ValueError("unregistered video reference")
    return result
