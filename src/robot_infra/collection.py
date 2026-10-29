"""Versioned multi-endpoint FAKE collection plan. No ROS/SDK or real factory."""

import math
from copy import deepcopy
from dataclasses import dataclass

from .contracts import digest

PLAN_VERSION = "collection-plan/1"
EVENT_VERSION = "collection-event/1"


def _keys(value, fields, label):
    if not isinstance(value, dict) or set(value) != set(fields.split()):
        raise ValueError(f"{label}: exact fields required: {fields}")


def _name(value):
    return (
        isinstance(value, str)
        and 0 < len(value) <= 100
        and all(c.isalnum() or c in "._-" for c in value)
    )


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_collection(config):
    """Compile this small fake execution plan, not an arbitrary node graph."""
    c = deepcopy(config)
    _keys(
        c,
        "schema_version mode asset_id clock_domain endpoints backends group sources cameras timing",
        "plan",
    )
    if (
        c["schema_version"] != PLAN_VERSION
        or c["mode"] != "fake"
        or c["clock_domain"] != "fake_monotonic"
    ):
        raise ValueError("only collection-plan/1 fake/fake_monotonic is implemented")
    if not _name(c["asset_id"]):
        raise ValueError("explicit synthetic asset_id required")
    for key in ("endpoints", "backends", "sources", "cameras"):
        if not isinstance(c[key], list) or not 1 <= len(c[key]) <= 32:
            raise ValueError(f"{key}: 1..32 entries required")
    if any(not _name(x) for x in c["backends"] + c["cameras"]):
        raise ValueError("invalid backend/camera name")
    if len(set(c["backends"])) != len(c["backends"]) or len(set(c["cameras"])) != len(c["cameras"]):
        raise ValueError("duplicate backend/camera")
    endpoints = {}
    for e in c["endpoints"]:
        _keys(e, "id backend kind names unit frame_id limits initial max_step", "endpoint")
        if (
            not _name(e["id"])
            or e["id"] in endpoints
            or e["backend"] not in c["backends"]
            or not _name(e["frame_id"])
        ):
            raise ValueError("duplicate/invalid endpoint or unknown backend/frame")
        if (e["kind"], e["unit"]) not in [("joint_position", "rad"), ("gripper_position", "m")]:
            raise ValueError("explicit supported kind/unit required")
        names = e["names"]
        if (
            not isinstance(names, list)
            or not 1 <= len(names) <= 32
            or any(not _name(x) for x in names)
            or len(set(names)) != len(names)
        ):
            raise ValueError("unique ordered coordinate names required")
        if e["kind"] == "gripper_position" and len(names) != 1:
            raise ValueError("gripper_position is one explicit opening coordinate")
        if (
            not isinstance(e["limits"], list)
            or len(e["limits"]) != len(names)
            or not isinstance(e["initial"], list)
            or len(e["initial"]) != len(names)
        ):
            raise ValueError("endpoint dimension mismatch")
        for bounds, initial in zip(e["limits"], e["initial"]):
            if (
                not isinstance(bounds, list)
                or len(bounds) != 2
                or not all(_finite(x) for x in bounds)
                or not _finite(initial)
                or not bounds[0] <= initial <= bounds[1]
                or bounds[0] >= bounds[1]
            ):
                raise ValueError("finite ordered limits and in-range initial required")
        if not _finite(e["max_step"]) or e["max_step"] <= 0:
            raise ValueError("explicit positive per-update max_step required")
        endpoints[e["id"]] = e
    if set(e["backend"] for e in endpoints.values()) != set(c["backends"]):
        raise ValueError("every backend must own endpoints")
    for backend in c["backends"]:
        names = [n for e in endpoints.values() if e["backend"] == backend for n in e["names"]]
        if len(names) != len(set(names)):
            raise ValueError("coordinate names conflict inside a backend batch")
    _keys(c["group"], "id members", "group")
    members = c["group"]["members"]
    if (
        not _name(c["group"]["id"])
        or not isinstance(members, list)
        or any(not _name(x) for x in members)
        or len(set(members)) != len(members)
        or set(members) != set(endpoints)
    ):
        raise ValueError("one complete execution group required")
    sources = {}
    for source in c["sources"]:
        _keys(source, "id priority", "source")
        if (
            not _name(source["id"])
            or source["id"] in sources
            or type(source["priority"]) is not int
            or source["priority"] < 0
        ):
            raise ValueError("unique source and nonnegative integer priority required")
        sources[source["id"]] = source["priority"]
    _keys(c["timing"], "period_ns max_state_age_ns max_action_age_ns lease_ttl_ns", "timing")
    if any(type(x) is not int or x <= 0 for x in c["timing"].values()):
        raise ValueError("positive integer nanosecond timing required")
    return {
        "config": c,
        "config_hash": digest(c),
        "endpoints": endpoints,
        "sources": sources,
        "batches": {
            b: [e["id"] for e in c["endpoints"] if e["backend"] == b] for b in c["backends"]
        },
    }


@dataclass(frozen=True)
class EndpointTarget:
    endpoint_id: str
    names: tuple[str, ...]
    positions: tuple[float, ...]
    unit: str
    frame_id: str


@dataclass(frozen=True)
class CommandSet:
    session_id: str
    source: str
    seq: int
    epoch: int
    lease_token: str | None
    snapshot_id: str
    created_ns: int
    target_ns: int
    expires_ns: int
    targets: tuple[EndpointTarget, ...]
    schema_version: str = EVENT_VERSION
