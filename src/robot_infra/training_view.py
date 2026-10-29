"""Auditable causal observations and command targets; not a LeRobot writer."""

import hashlib
import json
import math
from bisect import bisect_right
from collections import defaultdict
from copy import deepcopy
from pathlib import Path

from .episode_reader import episode_manifest, iter_events, read_index
from .episode_store import write_json


def validate_training_profile(c):
    fields = {"schema_version", "fps", "endpoints", "visuals", "sampling", "quality"}
    if not isinstance(c, dict) or set(c) != fields or c["schema_version"] != "training-view/1":
        raise ValueError("unsupported training profile")
    if type(c["fps"]) is not int or not 1 <= c["fps"] <= 240:
        raise ValueError("invalid training fps")
    ids = set()
    names = set()
    if not isinstance(c["endpoints"], list) or not c["endpoints"]:
        raise ValueError("named training endpoints required")
    for e in c["endpoints"]:
        if set(e) != {"id", "side", "names", "feedback_field", "command_field", "unit"}:
            raise ValueError("explicit endpoint semantics required")
        if any(not isinstance(e[k], str) or not e[k] for k in ("id", "side")) or e["id"] in ids:
            raise ValueError("duplicate/invalid endpoint")
        ids.add(e["id"])
        if (
            not isinstance(e["names"], list)
            or not e["names"]
            or any(not isinstance(n, str) or not n for n in e["names"])
        ):
            raise ValueError("explicit ordered names required")
        expanded = [e["id"] + "." + n for n in e["names"]]
        if len(set(expanded)) != len(expanded) or names.intersection(expanded):
            raise ValueError("duplicate coordinate names")
        names.update(expanded)
        if (e["feedback_field"], e["command_field"], e["unit"]) not in (
            ("q", "q", "rad"),
            ("gripper", "gripper_m", "m"),
        ):
            raise ValueError("explicit supported state/target units required")
        if e["feedback_field"] == "gripper" and len(e["names"]) != 1:
            raise ValueError("gripper is one opening coordinate")
    keys = set()
    streams = set()
    if not isinstance(c["visuals"], list):
        raise ValueError("visual sources required")
    for v in c["visuals"]:
        if (
            set(v) != {"feature", "stream", "required", "depth_m_per_unit"}
            or type(v["required"]) is not bool
        ):
            raise ValueError("invalid visual mapping")
        if (
            not isinstance(v["feature"], str)
            or not v["feature"]
            or not isinstance(v["stream"], str)
            or not v["stream"].startswith("camera/")
        ):
            raise ValueError("invalid camera feature/stream")
        if v["feature"] in keys or v["stream"] in streams:
            raise ValueError("duplicate visual mapping")
        keys.add(v["feature"])
        streams.add(v["stream"])
        if v["stream"].endswith("/depth") and (
            type(v["depth_m_per_unit"]) not in (int, float)
            or not math.isfinite(v["depth_m_per_unit"])
            or v["depth_m_per_unit"] <= 0
        ):
            raise ValueError("explicit verified depth scale required")
        if not v["stream"].endswith(("/rgb", "/depth")):
            raise ValueError("unsupported visual stream")
    sampling = c["sampling"]
    if set(sampling) != {"max_state_age_s", "max_image_age_s", "action_offset_steps"}:
        raise ValueError("explicit causal sampling required")
    for k in ("max_state_age_s", "max_image_age_s"):
        if (
            type(sampling[k]) not in (int, float)
            or not math.isfinite(sampling[k])
            or sampling[k] <= 0
        ):
            raise ValueError("invalid sample age")
    if (
        type(sampling["action_offset_steps"]) is not int
        or not 0 <= sampling["action_offset_steps"] <= 10
    ):
        raise ValueError("invalid action offset")
    if set(c["quality"]) != {"require_saved", "require_qc_pass"} or any(
        type(v) is not bool for v in c["quality"].values()
    ):
        raise ValueError("explicit quality admission required")
    return deepcopy(c)


def _vector(value, size):
    value = list(value) if isinstance(value, (list, tuple)) else [value]
    if len(value) != size or any(
        type(v) not in (int, float) or not math.isfinite(v) for v in value
    ):
        raise ValueError("invalid state/target dimension or value")
    return value


def published_targets(records, endpoints):
    """Confirm ordinary commands by receipt; holds are emitted after send."""
    pending = defaultdict(list)
    targets = defaultdict(list)
    by_side = defaultdict(list)
    for e in endpoints:
        by_side[e["side"]].append(e)

    def accept(item, available, confirmation):
        event = item["event"]
        p = event["payload"]
        for e in by_side.get(p.get("side"), []):
            field = e["command_field"]
            if field not in p or p[field] is None:
                continue
            if p.get("operation") == "gripper_only" and field != "gripper_m":
                continue
            targets[e["id"]].append(
                {
                    "time_ns": available,
                    "command_time_ns": event["host_monotonic_ns"],
                    "values": _vector(p[field], len(e["names"])),
                    "evidence": item["evidence"],
                    "confirmation": confirmation,
                    "operation": p.get("operation", "arm"),
                }
            )

    for item in records:
        event = item["event"]
        p = event["payload"]
        if event["kind"] == "command":
            if p.get("operation") == "measured_hold":
                accept(item, event["host_monotonic_ns"], "post_send_hold_event")
            elif "seq" in p:
                pending[(p.get("side"), p["seq"])].append(item)
        elif (
            event["kind"] == "receipt"
            and p.get("status") == "published_not_acknowledged"
            and "seq" in p
        ):
            for command in pending.pop((p.get("side"), p["seq"]), []):
                accept(command, event["host_monotonic_ns"], "published_not_acknowledged")
    for rows in targets.values():
        rows.sort(key=lambda x: x["time_ns"])
    return dict(targets)


def command_records(root):
    for item in iter_events(root, kinds={"command", "receipt"}, feedback=False):
        yield {
            "event": item["event"],
            "evidence": {"kind": "episode_mcap", "sequence": item["sequence"]},
        }


def journal_command_records(paths):
    for path in paths:
        with Path(path).open(encoding="utf-8") as source:
            for i, line in enumerate(source):
                e = json.loads(line)
                if e["kind"] in ("command", "receipt"):
                    yield {
                        "event": e,
                        "evidence": {"kind": "session_journal", "path": str(path), "line": i + 1},
                    }


def _latest(rows, stamp, key):
    i = bisect_right(rows, stamp, key=lambda x: x[key]) - 1
    return rows[i] if i >= 0 else None


def view_rows(root, config, *, journal_paths=()):
    root, m = episode_manifest(root)
    c = validate_training_profile(config)
    start, end = m["start_monotonic_ns"], m["end_monotonic_ns"]
    if c["quality"]["require_saved"] and m["status"] != "saved":
        raise ValueError("episode is not committed as saved")
    if c["quality"]["require_qc_pass"] and not m.get("qc_passed"):
        raise ValueError("episode QC did not pass")
    streams = read_index(root)
    feedback = defaultdict(list)
    for item in iter_events(root, kinds=set(), feedback=True):
        e = item["event"]
        p = e["payload"]
        feedback[p["side"]].append(
            {"time_ns": e["host_monotonic_ns"], "payload": p, "sequence": item["sequence"]}
        )
    for rows in feedback.values():
        rows.sort(key=lambda x: x["time_ns"])
    targets = published_targets(command_records(root), c["endpoints"])
    if journal_paths:
        seeds = published_targets(journal_command_records(journal_paths), c["endpoints"])
        for name, rows in seeds.items():
            seed = _latest(rows, start, "time_ns")
            if seed:
                targets.setdefault(name, []).insert(0, seed)
    # New recordings can carry confirmed in-memory targets from before recording.
    initial = m.get("context", {}).get("initial_targets", {})
    for e in c["endpoints"]:
        value = initial.get(e["side"], {}).get(e["command_field"])
        if (
            value
            and value["time_ns"] <= start
            and not any(x["time_ns"] <= start for x in targets.get(e["id"], []))
        ):
            targets.setdefault(e["id"], []).insert(
                0,
                {
                    "time_ns": value["time_ns"],
                    "command_time_ns": value["time_ns"],
                    "values": _vector(value["values"], len(e["names"])),
                    "evidence": {"kind": "manifest_initial_target"},
                    "confirmation": "snapshot_after_publish",
                    "operation": "prior_target",
                },
            )
    for e in c["endpoints"]:
        configured = m.get("context", {}).get("config", {}).get("model", {}).get("joint_names")
        if e["feedback_field"] == "q" and configured is not None and e["names"] != configured:
            raise ValueError("joint order does not match recorded model")
    missing = [v["stream"] for v in c["visuals"] if v["required"] and not streams.get(v["stream"])]
    if missing:
        raise ValueError("required visual streams missing: " + ",".join(missing))
    for rows in streams.values():
        rows.sort(key=lambda x: x["received_ns"])
    for index in range(math.ceil((end - start) / 1e9 * c["fps"])):
        stamp = start + round(index * 1e9 / c["fps"])
        if stamp >= end:
            break
        action_stamp = stamp + round(c["sampling"]["action_offset_steps"] * 1e9 / c["fps"])
        state = []
        action = []
        mask = []
        evidence = {}
        valid = True
        reasons = []
        for e in c["endpoints"]:
            f = _latest(feedback.get(e["side"], []), stamp, "time_ns")
            target = _latest(targets.get(e["id"], []), action_stamp, "time_ns")
            if f is None or stamp - f["time_ns"] > c["sampling"]["max_state_age_s"] * 1e9:
                values = [None] * len(e["names"])
                valid = False
                reasons.append(e["id"] + ": missing/stale state")
            else:
                values = _vector(f["payload"][e["feedback_field"]], len(e["names"]))
            state += values
            known = target is not None and action_stamp < end
            action += target["values"] if known else [None] * len(e["names"])
            mask += [known] * len(e["names"])
            if not known:
                valid = False
                reasons.append(e["id"] + ": unknown target or episode end")
            evidence[e["id"]] = {
                "feedback_sequence": f["sequence"] if f else None,
                "target": target,
            }
        visuals = {}
        for v in c["visuals"]:
            row = _latest(streams.get(v["stream"], []), stamp, "received_ns")
            ok = (
                row is not None
                and stamp - row["received_ns"] <= c["sampling"]["max_image_age_s"] * 1e9
            )
            visuals[v["feature"]] = {
                "stream": v["stream"],
                "row": row,
                "valid": ok,
                "depth_m_per_unit": v["depth_m_per_unit"],
            }
            if v["required"] and not ok:
                valid = False
                reasons.append(v["stream"] + ": missing/stale image")
        yield {
            "frame_index": index,
            "timestamp": (stamp - start) / 1e9,
            "observation_time_ns": stamp,
            "action_time_ns": action_stamp,
            "observation.state": state,
            "action": action,
            "action_valid": mask,
            "visuals": visuals,
            "valid": valid,
            "reasons": reasons,
            "evidence": evidence,
        }


def build_training_view(root, config, output, *, journal_paths=()):
    root, m = episode_manifest(root)
    c = validate_training_profile(config)
    output = Path(output).resolve()
    if output == root or output.is_relative_to(root):
        raise ValueError("view output must be outside source episode")
    output.mkdir(parents=True, exist_ok=False)
    count = valid = 0
    try:
        with (output / "rows.jsonl").open("x", encoding="utf-8") as f:
            for row in view_rows(root, c, journal_paths=journal_paths):
                f.write(json.dumps(row, allow_nan=False) + "\n")
                count += 1
                valid += row["valid"]
        report = {
            "schema_version": "training-view-result/1",
            "source": str(root),
            "source_manifest_sha256": hashlib.sha256(
                (root / "manifest.json").read_bytes()
            ).hexdigest(),
            "source_mcap_sha256": m["mcap_sha256"],
            "task": m["task"],
            "episode": m["episode"],
            "config": c,
            "rows": count,
            "valid_rows": valid,
            "coordinate_names": [e["id"] + "." + n for e in c["endpoints"] for n in e["names"]],
            "units": [e["unit"] for e in c["endpoints"] for _ in e["names"]],
            "journal_sources": [{"path": str(p), "sha256": _file_hash(p)} for p in journal_paths],
            "format": "JSONL audit view with image references; not LeRobot v3; not physical playback",
            "action_semantics": "confirmed published absolute targets, persistent until replaced; unknown targets are null/masked, never measured-width labels",
        }
        write_json(output / "view.json", report)
        return report
    except Exception as exc:
        write_json(output / "failure.json", {"error": str(exc), "incomplete": True})
        raise


def _file_hash(path):
    with Path(path).open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()
