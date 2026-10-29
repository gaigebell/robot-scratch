"""Read-only comparison of recorded image gaps and device metadata stamps."""

import argparse
import json
import math
from pathlib import Path

from mcap.reader import make_reader


def audit(episode, *, gap_ms=150, max_gaps=32, metadata_limit=128):
    episode = Path(episode)
    manifest = json.loads((episode / "manifest.json").read_text())
    previous = {}
    gaps = []
    omitted = 0
    with (episode / "index.jsonl").open(encoding="utf8") as f:
        for line in f:
            row = json.loads(line)
            name = row["stream"]
            if not name.startswith("camera/"):
                continue
            old = previous.get(name)
            previous[name] = row
            if old is None or row["received_ns"] - old["received_ns"] <= gap_ms * 1e6:
                continue
            if len(gaps) >= max_gaps:
                omitted += 1
                continue
            p = manifest.get("camera_pipeline", {}).get(name.split("/")[1], {})
            capture = p.get("capture", {}).get(name.split("/")[-1]) or {}
            record = p.get("record", {}).get(name.split("/")[-1]) or {}
            gaps.append(
                {
                    "stream": name,
                    "receive_gap_ms": (row["received_ns"] - old["received_ns"]) / 1e6,
                    "before": old,
                    "after": row,
                    "same_capture_record_fps": capture.get("fps") == record.get("fps")
                    if capture and record
                    else None,
                    "writer_stats": manifest.get("frame_capture_stats", {}).get(name),
                    "metadata_between_count": 0,
                    "metadata_between": [],
                    "edge_metadata": {},
                }
            )
    if gaps:
        with (episode / "streams.mcap").open("rb") as f:
            for schema, channel, message in make_reader(f).iter_messages(
                topics=["/event/device_metadata"]
            ):
                event = json.loads(message.data)
                p = event["payload"]
                name = "camera/" + p["role"] + "/" + p["stream"]
                for gap in gaps:
                    if gap["stream"] != name:
                        continue
                    a, b = gap["before"].get("source_stamp_ns"), gap["after"].get("source_stamp_ns")
                    stamp = p.get("stamp_ns")
                    if a is None or b is None or stamp is None or not a <= stamp <= b:
                        continue
                    raw = json.loads(p["raw_json"])
                    info = {
                        "source_stamp_ns": stamp,
                        "host_received_ns": event["host_monotonic_ns"],
                        "metadata": {
                            k: raw[k]
                            for k in (
                                "frame_counter",
                                "frame_number",
                                "clock_domain",
                                "actual_fps",
                                "actual_exposure",
                                "auto_exposure",
                            )
                            if k in raw
                        },
                    }
                    if stamp == a:
                        gap["edge_metadata"]["before"] = info
                    elif stamp == b:
                        gap["edge_metadata"]["after"] = info
                    else:
                        gap["metadata_between_count"] += 1
                        if len(gap["metadata_between"]) < metadata_limit:
                            gap["metadata_between"].append(info)
    for gap in gaps:
        gap["interpretation"] = (
            "metadata_observed_inside_recorded_image_gap"
            if gap["metadata_between_count"]
            else "unresolved"
        )
        gap["metadata_samples_truncated"] = gap["metadata_between_count"] > len(
            gap["metadata_between"]
        )
    return {
        "schema_version": "camera-delivery-audit/1",
        "episode": str(episode),
        "gap_threshold_ms": gap_ms,
        "gaps": gaps,
        "omitted_gaps": omitted,
        "source_modified": False,
        "limits": "Metadata proves driver-observed frame evidence, not successful Image publication. Intentional recording sampling, writer loss and clock domains must be checked separately.",
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--episode", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--gap-ms", type=float, default=150)
    p.add_argument("--max-gaps", type=int, default=32)
    args = p.parse_args(argv)
    if not math.isfinite(args.gap_ms) or args.gap_ms <= 0 or not 1 <= args.max_gaps <= 128:
        p.error("invalid audit bounds")
    output = Path(args.output)
    if output.exists():
        p.error("output already exists")
    result = audit(args.episode, gap_ms=args.gap_ms, max_gaps=args.max_gaps)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf8")
    print(
        json.dumps(
            {
                "gaps": len(result["gaps"]),
                "omitted_gaps": result["omitted_gaps"],
                "output": str(output),
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
