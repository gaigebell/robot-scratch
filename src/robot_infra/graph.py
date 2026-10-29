"""Strict fake-only graph compiler. No plugin import or device construction."""

import json
import math
from collections import Counter
from copy import deepcopy
from importlib.resources import files
from pathlib import Path

from jsonschema import Draft202012Validator

from .contracts import digest


class ConfigError(ValueError):
    pass


PORTS = {
    "fake_teleop": {"intent": ("out", "joint_position")},
    "stub_policy": {"intent": ("out", "joint_position")},
    "identity_mapper": {"intent": ("in", "joint_position"), "proposal": ("out", "joint_position")},
    "guard": {
        "proposal": ("in", "joint_position"),
        "command": ("out", "joint_position"),
        "state": ("in", "joint_state"),
    },
    "fake_robot": {"command": ("in", "joint_position"), "state": ("out", "joint_state")},
    "fake_camera": {"rgb": ("out", "rgb8")},
    "recorder": {"state": ("in", "joint_state"), "rgb": ("in", "rgb8")},
}
ROUTES = {
    ("fake_teleop.intent", "identity_mapper.intent"),
    ("stub_policy.intent", "identity_mapper.intent"),
    ("identity_mapper.proposal", "guard.proposal"),
    ("guard.command", "fake_robot.command"),
    ("fake_robot.state", "guard.state"),
    ("fake_robot.state", "recorder.state"),
    ("fake_camera.rgb", "recorder.rgb"),
}


def strict_json(data: str):
    def object_hook(pairs):
        result = {}
        for k, v in pairs:
            if k in result:
                raise ValueError(f"duplicate key: {k}")
            result[k] = v
        return result

    def bad_number(value):
        raise ValueError(f"non-finite JSON number: {value}")

    return json.loads(data, object_pairs_hook=object_hook, parse_constant=bad_number)


def validate_graph(config: dict) -> dict:
    schema = json.loads(files("robot_infra").joinpath("schemas/graph.json").read_text())
    errors = sorted(
        Draft202012Validator(schema).iter_errors(config), key=lambda e: str(list(e.path))
    )
    if errors:
        raise ConfigError(
            "; ".join(f"{'.'.join(map(str, e.path)) or '$'}: {e.message}" for e in errors)
        )
    cfg = deepcopy(config)
    asset = cfg["asset"]
    timing = cfg["timing"]
    if 2 * timing["period_ns"] >= min(timing["max_action_age_ns"], timing["lease_ttl_ns"]):
        raise ConfigError(
            "timing: two-sample stub horizon plus one-period latency must fit action age and lease TTL"
        )
    for i, (low, high) in enumerate(asset["limits"]):
        p = asset["initial_positions"][i]
        if (
            not all(math.isfinite(x) for x in (low, high, p))
            or not low < high
            or not low <= p <= high
        ):
            raise ConfigError(
                f"asset.limits[{i}]: finite ordered limits and initial position required"
            )
    nodes = {n["id"]: n for n in cfg["nodes"]}
    kinds = {n["kind"]: n for n in cfg["nodes"]}
    if len(nodes) != 7 or Counter(n["kind"] for n in cfg["nodes"]) != Counter(PORTS.keys()):
        raise ConfigError("nodes: unique IDs and exactly one of each M1 node kind required")
    for node in nodes.values():
        expected = PORTS[node["kind"]]
        if set(node["ports"]) != set(expected):
            raise ConfigError(f"node {node['id']}: required ports {sorted(expected)}")
        for name, port in node["ports"].items():
            if (port["direction"], port["type"]) != expected[name]:
                raise ConfigError(f"port {node['id']}.{name}: incompatible direction/type")
            if port["type"] != "rgb8":
                for key in ("unit", "frame_id", "joint_names"):
                    if port.get(key) != asset[key]:
                        raise ConfigError(f"port {node['id']}.{name}: {key} differs from asset")
            if port.get("clock_domain") != cfg["timing"]["clock_domain"]:
                raise ConfigError(f"port {node['id']}.{name}: clock_domain mismatch")
    routes = []
    for edge in cfg["edges"]:
        endpoints = []
        for field, direction in (("from", "out"), ("to", "in")):
            node_id, sep, port_id = edge[field].partition(".")
            if not sep or node_id not in nodes or port_id not in nodes[node_id]["ports"]:
                raise ConfigError(f"edge {edge}: missing node/port {edge[field]}")
            port = nodes[node_id]["ports"][port_id]
            if port["direction"] != direction:
                raise ConfigError(f"edge {edge}: {field} direction mismatch")
            endpoints.append((nodes[node_id]["kind"] + "." + port_id, port))
        if endpoints[0][1]["type"] != endpoints[1][1]["type"]:
            raise ConfigError(f"edge {edge}: type mismatch")
        routes.append((endpoints[0][0], endpoints[1][0]))
    if len(set(routes)) != len(routes) or set(routes) != ROUTES:
        raise ConfigError(
            "edges: M1 routes require mapper -> unique guard -> robot; no bypass, extra writer or missing feedback"
        )
    # The only feedback cycle is robot's sampled state -> guard, not an algebraic loop.
    return {
        "config": cfg,
        "config_hash": digest(cfg),
        "nodes_by_kind": {k: n["id"] for k, n in kinds.items()},
    }


def load_graph(path: str | Path) -> dict:
    try:
        return validate_graph(strict_json(Path(path).read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError, OverflowError) as exc:
        raise ConfigError(str(exc)) from exc
