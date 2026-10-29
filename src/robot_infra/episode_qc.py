"""Offline stream timing/QC and alignment indexes. Never constructs devices."""

import hashlib
import json
from bisect import bisect_left
from collections import defaultdict
from pathlib import Path

from .episode_store import write_json


def nearest(rows, stamp):
    if not rows:
        return None
    i = bisect_left(rows, stamp, key=lambda x: x["received_ns"])
    return min(
        rows[max(0, i - 1) : min(len(rows), i + 1)], key=lambda x: abs(x["received_ns"] - stamp)
    )


def quality_report(root, manifest, *, deep=False, output_root=None):
    root = Path(root)
    streams = defaultdict(list)
    issues = []
    output = root if output_root is None else Path(output_root)
    output.mkdir(parents=True, exist_ok=True)
    version = manifest.get("schema_version")
    if version not in ("real-episode/1", "real-episode/2"):
        raise ValueError("unsupported real episode manifest")
    with (root / "index.jsonl").open(encoding="utf-8") as source:
        for line in source:
            item = json.loads(line)
            streams[item["stream"]].append(item)
    from .media_qc import inspect_media

    media = inspect_media(root, manifest.get("media", {}), streams, deep=deep)
    if version == "real-episode/1" and media:
        raise ValueError("external media requires real-episode/2")
    if version == "real-episode/2":
        for name in manifest["expected_streams"]:
            if name.endswith("/rgb") and manifest.get("counts", {}).get(name) and name not in media:
                raise ValueError("video asset missing for RGB stream")
    cfg = manifest["recording"]
    expected = manifest["expected_streams"]
    duration = max(1e-9, (manifest["end_monotonic_ns"] - manifest["start_monotonic_ns"]) / 1e9)
    statistics = {}
    for kind in cfg.get("required_events", []):
        if not manifest.get("counts", {}).get("event/" + kind):
            issues.append("event/" + kind + ": missing")
    for name, definition in expected.items():
        rows = streams.get(name, [])
        stamps = [x["received_ns"] for x in rows]
        gaps = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
        fps = (
            (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
            if len(stamps) > 1 and stamps[-1] > stamps[0]
            else 0.0
        )
        stats = {
            "frames": len(rows),
            "fps_received": fps,
            "max_gap_ms": max(gaps, default=0) * 1000,
            "out_of_order": sum(b < a for a, b in zip(stamps, stamps[1:])),
            "profile_mismatches": sum(x.get("profile_match") is False for x in rows),
        }
        source = [x.get("source_stamp_ns") for x in rows if x.get("source_stamp_ns") is not None]
        stats["source_stamp_regressions"] = sum(b < a for a, b in zip(source, source[1:]))
        if name.startswith("pose_feedback/"):
            stats["unique_source_stamps"] = len(set(source))
            stats["non_unit_quaternions"] = sum(x.get("quaternion_unit") is False for x in rows)
            stats["zero_source_stamps"] = sum(x == 0 for x in source)
            if stats["non_unit_quaternions"]:
                issues.append(name + ": non-unit quaternion")
            if stats["zero_source_stamps"]:
                issues.append(name + ": zero source stamp")
            if len(source) > 1 and stats["unique_source_stamps"] == 1:
                issues.append(name + ": frozen source stamp")
        statistics[name] = stats
        if not rows:
            issues.append(name + ": missing")
        if stats["out_of_order"] or stats["source_stamp_regressions"]:
            issues.append(name + ": timestamp regression")
        if stats["profile_mismatches"]:
            issues.append(name + ": capture profile mismatch")
        if definition.get("fps"):
            target = definition["fps"]
            if (
                fps < target * cfg["qc"]["min_fps_ratio"]
                or len(rows) < duration * target * cfg["qc"]["min_fps_ratio"]
            ):
                issues.append(name + ": low capture rate/coverage")
            if max(gaps, default=0) > cfg["qc"]["max_frame_gap_s"]:
                issues.append(name + ": frame gap")
    alignment = cfg["alignment"]
    reference = alignment["reference_stream"]
    bad, total = 0, 0
    for rows in streams.values():
        rows.sort(key=lambda x: x["received_ns"])
    with (output / "alignment.jsonl").open("w", encoding="utf-8") as alignment_file:
        for anchor in streams.get(reference, []):
            matches = {}
            valid = True
            for name in expected:
                row = nearest(streams.get(name, []), anchor["received_ns"])
                delta = None if row is None else row["received_ns"] - anchor["received_ns"]
                ok = delta is not None and abs(delta) <= alignment["max_delta_s"] * 1e9
                matches[name] = {
                    "sequence": row["sequence"] if row else None,
                    "delta_ns": delta,
                    "valid": ok,
                }
                valid &= ok
            total += 1
            bad += not valid
            alignment_file.write(
                json.dumps(
                    {
                        "reference_sequence": anchor["sequence"],
                        "received_ns": anchor["received_ns"],
                        "clock_domain": "host_received_monotonic",
                        "valid": valid,
                        "matches": matches,
                    }
                )
                + "\n"
            )
    if not total or bad / max(1, total) > cfg["qc"]["max_unaligned_ratio"]:
        issues.append("alignment: missing or excessive invalid rows")
    if manifest.get("interrupted"):
        issues.append("episode interrupted: " + manifest["interrupted"])
    if manifest.get("dropped"):
        issues.append("recording queue overflow")
    if duration < cfg["qc"]["min_duration_s"]:
        issues.append("episode shorter than minimum duration")
    if manifest.get("writer_error"):
        issues.append("writer failure: " + manifest["writer_error"])
    if manifest.get("counts", {}).get("event/observation_error"):
        issues.append("observation error: inspect recorded event")
    if deep:
        from mcap.reader import NonSeekingReader

        from .image_codec import decode_image

        path = root / "streams.mcap"
        digest = hashlib.sha256()
        if (
            hashlib.sha256((root / "index.jsonl").read_bytes()).hexdigest()
            != manifest["index_sha256"]
        ):
            raise ValueError("stream index hash mismatch")
        with path.open("rb") as source:
            for block in iter(lambda: source.read(4 * 1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != manifest["mcap_sha256"]:
            raise ValueError("MCAP hash mismatch")
        with path.open("rb") as source:
            source.seek(-8, 2)
            if source.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("missing MCAP footer")
            source.seek(0)
            reader = NonSeekingReader(
                source, validate_crcs=True, record_size_limit=128 * 1024 * 1024
            )
            if reader.get_header().profile != "robot-infra-" + version:
                raise ValueError("unsupported real episode MCAP profile")
            source.seek(0)
            reader = NonSeekingReader(
                source, validate_crcs=True, record_size_limit=128 * 1024 * 1024
            )
            count = 0
            video_references = 0
            indexed = {
                row["sequence"]: row
                for rows in streams.values()
                for row in rows
                if row.get("media")
            }
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                if channel.message_encoding == "robot-infra-raw-image/1":
                    decode_image(message.data)
                elif channel.message_encoding == "json":
                    value = json.loads(message.data)
                    if value.get("schema_version") == "robot-infra-video-frame/1":
                        row = indexed.get(message.sequence)
                        if (
                            row is None
                            or row["stream"] != "/".join(channel.topic.split("/")[1:])
                            or row["media"] != value["media"]
                            or row["source_stamp_ns"] != value["image"]["stamp_ns"]
                            or row["received_ns"] != value["image"]["host_monotonic_ns"]
                        ):
                            raise ValueError("MCAP/video index reference mismatch")
                        video_references += 1
                else:
                    raise ValueError("unsupported MCAP channel encoding")
                count += 1
            if count != manifest["mcap_messages"]:
                raise ValueError("MCAP message count mismatch")
            if video_references != len(indexed):
                raise ValueError("MCAP/video reference count mismatch")
    result = {
        "schema_version": "episode-qc/1",
        "passed": not issues,
        "issues": issues,
        "duration_s": duration,
        "streams": statistics,
        "media": media,
        "alignment": {
            "reference": reference,
            "rows": total,
            "invalid_rows": bad,
            "clock_domain": "host_received_monotonic",
            "exposure_synchronization_verified": False,
        },
        "deep_integrity_checked": deep,
    }
    write_json(output / "qc.json", result)
    return result
