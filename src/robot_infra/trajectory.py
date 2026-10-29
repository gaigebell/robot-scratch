"""Offline native-target timeline. Does not create a backend or reuse leases."""

from .episode_reader import episode_manifest
from .training_view import command_records, published_targets, validate_training_profile


def trajectory_plan(root, profile):
    root, m = episode_manifest(root)
    c = validate_training_profile(profile)
    if m["status"] != "saved" or not m.get("qc_passed"):
        raise ValueError("replay planning requires a saved QC-passing episode")
    targets = published_targets(command_records(root), c["endpoints"])
    start = m["start_monotonic_ns"]
    events = []
    for endpoint, rows in targets.items():
        spec = next(e for e in c["endpoints"] if e["id"] == endpoint)
        for row in rows:
            events.append(
                {
                    "offset_ns": row["time_ns"] - start,
                    "endpoint": endpoint,
                    "names": spec["names"],
                    "unit": spec["unit"],
                    "positions": row["values"],
                    "source": row["evidence"],
                    "confirmation": row["confirmation"],
                }
            )
    events.sort(key=lambda e: (e["offset_ns"], e["endpoint"]))
    return {
        "schema_version": "trajectory-plan/1",
        "source": str(root),
        "source_mcap_sha256": m["mcap_sha256"],
        "physical_commands": False,
        "timing": "original host-monotonic publish confirmation offsets; no silent gap compression",
        "duration_ns": m["end_monotonic_ns"] - start,
        "events": events,
        "unknown_initial_targets": "must resolve from context/journal or explicit preflight; never fabricate gripper goal from measured opening",
        "execution_status": "offline plan only; physical runner/start-state alignment/new ownership still required",
    }


class TimelineSource:
    def __init__(self, plan):
        if plan.get("schema_version") != "trajectory-plan/1":
            raise ValueError("unsupported trajectory plan")
        self.events = plan["events"]
        self.index = 0
        self.last = -1
        if any(e["offset_ns"] < 0 for e in self.events) or any(
            b["offset_ns"] < a["offset_ns"] for a, b in zip(self.events, self.events[1:])
        ):
            raise ValueError("invalid trajectory timing")

    def poll(self, elapsed_ns):
        if type(elapsed_ns) is not int or elapsed_ns < self.last:
            raise ValueError("monotonic playback time required")
        self.last = elapsed_ns
        due = []
        while self.index < len(self.events) and self.events[self.index]["offset_ns"] <= elapsed_ns:
            due.append(self.events[self.index])
            self.index += 1
        return due
