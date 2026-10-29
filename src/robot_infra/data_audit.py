"""Offline diagnostics; source sidecars remain unchanged."""

import hashlib
import json
from collections import Counter
from pathlib import Path

from .episode_reader import episode_manifest, iter_events, read_index


def quantiles(values):
    values = sorted(x for x in values if type(x) in (int, float))
    return (
        {
            name: values[round((len(values) - 1) * q)]
            for name, q in (("min", 0), ("median", 0.5), ("p95", 0.95), ("max", 1))
        }
        if values
        else None
    )


def audit_episode(root, output, *, deep=False):
    root, manifest = episode_manifest(root)
    output = Path(output).resolve()
    if output == root or output.is_relative_to(root):
        raise ValueError("audit output must be outside source episode")
    from .episode_qc import quality_report

    qc = quality_report(root, manifest, deep=deep, output_root=output)
    start = manifest["start_monotonic_ns"]
    inputs = []
    iks = []
    guards = Counter()
    commands = Counter()
    stops = Counter()
    for item in iter_events(
        root, kinds={"raw_input", "motion_result", "guard", "command", "stop"}, feedback=False
    ):
        e = item["event"]
        p = e["payload"]
        kind = e["kind"]
        if kind == "raw_input":
            inputs.append(
                {
                    "received_ns": e["host_monotonic_ns"],
                    "t_s": (e["host_monotonic_ns"] - start) / 1e9,
                    "client_time_ms": p["client_time_ms"],
                    "seq": p["seq"],
                    "rtt_ms": p.get("transport", {}).get("previous_rtt_ms"),
                    "frame_work_ms": p.get("previous_frame_work_ms"),
                    "media": p.get("client_media", {}),
                }
            )
        elif kind == "motion_result" and "result" in p:
            iks.append(p["result"])
        elif kind == "guard":
            guards[p.get("decision", "unknown") + ":" + str(p.get("reason"))] += 1
        elif kind == "command":
            commands[p["side"] + ":" + p.get("operation", "arm")] += 1
        elif kind == "stop":
            stops[p.get("reason", "unknown")] += 1
    input_gaps = [
        {
            "end_t_s": b["t_s"],
            "host_gap_ms": (b["received_ns"] - a["received_ns"]) / 1e6,
            "client_gap_ms": b["client_time_ms"] - a["client_time_ms"],
        }
        for a, b in zip(inputs, inputs[1:])
    ]
    camera = {}
    for name, rows in read_index(root).items():
        if not name.startswith("camera/"):
            continue
        gaps = [
            {
                "end_t_s": (b["received_ns"] - start) / 1e9,
                "host_gap_ms": (b["received_ns"] - a["received_ns"]) / 1e6,
                "source_gap_ms": None
                if a["source_stamp_ns"] is None or b["source_stamp_ns"] is None
                else (b["source_stamp_ns"] - a["source_stamp_ns"]) / 1e6,
            }
            for a, b in zip(rows, rows[1:])
        ]
        camera[name] = {
            "count": len(rows),
            "largest_gaps": sorted(gaps, key=lambda x: x["host_gap_ms"], reverse=True)[:5],
        }
    warnings = []
    rtt = quantiles(x["rtt_ms"] for x in inputs)
    gaps = quantiles(x["host_gap_ms"] for x in input_gaps)
    if rtt and rtt["max"] > 200:
        warnings.append(
            "control RTT exceeds 200ms; current capture QC does not grade teleop responsiveness"
        )
    if manifest["status"] != "saved":
        warnings.append("episode has not been committed as saved")
    if not any("/depth" in name for name in manifest["expected_streams"]):
        warnings.append("no depth recorded; RGB-D training cannot synthesize missing depth")
    report = {
        "schema_version": "episode-audit/1",
        "source": str(root),
        "source_manifest_sha256": hashlib.sha256((root / "manifest.json").read_bytes()).hexdigest(),
        "episode": manifest["episode"],
        "status": manifest["status"],
        "duration_s": qc["duration_s"],
        "qc": qc,
        "inputs": {
            "count": len(inputs),
            "rtt_ms": rtt,
            "host_gap_ms": gaps,
            "client_gap_ms": quantiles(x["client_gap_ms"] for x in input_gaps),
            "frame_work_ms": quantiles(x["frame_work_ms"] for x in inputs),
            "largest_gaps": sorted(input_gaps, key=lambda x: x["host_gap_ms"], reverse=True)[:5],
            "rtt_peaks": sorted(inputs, key=lambda x: x["rtt_ms"] or 0, reverse=True)[:5],
        },
        "cameras": camera,
        "ik": {
            "count": len(iks),
            "elapsed_ms": quantiles(x["elapsed_ms"] for x in iks),
            "reasons": dict(Counter(x["reason"] for x in iks)),
        },
        "guards": dict(guards),
        "commands": dict(commands),
        "stops": dict(stops),
        "warnings": warnings,
        "causality": "RTT includes transport and application scheduling; these logs alone cannot isolate Wi-Fi, browser and host load",
    }
    from .episode_store import write_json

    write_json(output / "audit.json", report)
    return report


def audit_journal(paths, windows=()):
    """Stream segments once; include events outside episode windows."""
    inputs = []
    counts = Counter()
    client_events = []
    stops = []
    first = last = None
    bytes_read = 0
    for path in paths:
        with Path(path).open(encoding="utf-8") as source:
            for line in source:
                bytes_read += len(line.encode())
                e = json.loads(line)
                counts[e["kind"]] += 1
                stamp = e["host_monotonic_ns"]
                first = stamp if first is None else min(first, stamp)
                last = stamp if last is None else max(last, stamp)
                if e["kind"] == "raw_input":
                    p = e["payload"]
                    inputs.append(
                        {
                            "received_ns": stamp,
                            "rtt_ms": p.get("transport", {}).get("previous_rtt_ms"),
                            "client_time_ms": p["client_time_ms"],
                            "episodes": [
                                w["episode"] for w in windows if w["start"] <= stamp <= w["end"]
                            ],
                        }
                    )
                elif e["kind"] == "client_event":
                    client_events.append(e)
                elif e["kind"] == "stop":
                    stops.append(e)
    return {
        "schema_version": "session-audit/1",
        "paths": [str(p) for p in paths],
        "bytes": bytes_read,
        "duration_s": (last - first) / 1e9 if last is not None else 0,
        "counts": dict(counts),
        "rtt_ms": quantiles(x["rtt_ms"] for x in inputs),
        "rtt_peaks": sorted(inputs, key=lambda x: x["rtt_ms"] or 0, reverse=True)[:12],
        "max_input_gap_ms": max(
            ((b["received_ns"] - a["received_ns"]) / 1e6 for a, b in zip(inputs, inputs[1:])),
            default=0,
        ),
        "client_events": client_events,
        "stops": stops,
    }
