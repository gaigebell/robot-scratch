"""Versioned plans for three workflows. This compiler never launches devices."""

from collections import Counter, defaultdict, deque
from copy import deepcopy

# Each catalog entry specifies input and output port types. Runtime plugins
# remain separate; listing a node here does not claim its hardware support.
CATALOG = {
    "driver": ({"commands": "driver-command/1"}, {"observations": "observation-stream/1"}),
    "cameras": ({}, {"observations": "observation-stream/1"}),
    "snapshot": ({"observations": "observation-stream/1"}, {"snapshot": "observation-batch/1"}),
    "webxr": ({}, {"intent": "teleop-intent/1", "operator": "session-control/1"}),
    "operator": ({}, {"operator": "session-control/1"}),
    "teleop_mapping": (
        {"intent": "teleop-intent/1", "snapshot": "observation-batch/1"},
        {"action": "candidate-action/1"},
    ),
    "trajectory_source": ({}, {"action": "candidate-action/1"}),
    "task_context": ({}, {"context": "policy-context/1"}),
    "history": (
        {"observations": "observation-stream/1", "executed": "execution-events/1"},
        {"history": "policy-history/1"},
    ),
    "policy_client": (
        {
            "snapshot": "observation-batch/1",
            "context": "policy-context/1",
            "history": "policy-history/1",
        },
        {"action": "candidate-action/1"},
    ),
    "scheduler": ({"action": "candidate-action/1"}, {"action": "candidate-action/1"}),
    "motion_adapter": (
        {"action": "candidate-action/1", "snapshot": "observation-batch/1"},
        {"targets": "native-targets/1"},
    ),
    "gateway": (
        {
            "targets": "native-targets/1",
            "snapshot": "observation-batch/1",
            "operator": "session-control/1",
        },
        {"commands": "driver-command/1", "events": "execution-events/1"},
    ),
    "recorder": (
        {
            "observations": "observation-stream/1",
            "events": "execution-events/1",
            "actions": "candidate-action/1",
            "targets": "native-targets/1",
        },
        {},
    ),
}


def compile_experiment(config):
    c = deepcopy(config)
    if (
        set(c) != {"schema_version", "mode", "deployment", "nodes", "edges"}
        or c["schema_version"] != "experiment-graph/1"
    ):
        raise ValueError("unsupported experiment graph")
    if c["mode"] not in ("collection", "replay", "inference"):
        raise ValueError("unknown workflow mode")
    if not isinstance(c["deployment"], str) or not c["deployment"]:
        raise ValueError("explicit deployment reference required")
    if not isinstance(c["nodes"], list) or not isinstance(c["edges"], list):
        raise ValueError("node and edge lists required")
    nodes = {}
    kinds = Counter()
    for node in c["nodes"]:
        if (
            set(node) != {"id", "kind", "config"}
            or not isinstance(node["id"], str)
            or not node["id"]
            or "." in node["id"]
            or node["id"] in nodes
        ):
            raise ValueError("invalid/duplicate graph node")
        if node["kind"] not in CATALOG or not isinstance(node["config"], dict):
            raise ValueError("unsupported node kind/config")
        nodes[node["id"]] = node
        kinds[node["kind"]] += 1
    for kind in (
        "driver",
        "cameras",
        "snapshot",
        "scheduler",
        "motion_adapter",
        "gateway",
        "recorder",
    ):
        if kinds[kind] != 1:
            raise ValueError("one shared " + kind + " node required")
    required = {
        "collection": ("webxr", "teleop_mapping"),
        "replay": ("operator", "trajectory_source"),
        "inference": ("operator", "policy_client", "task_context", "history"),
    }[c["mode"]]
    for kind in required:
        if kinds[kind] != 1:
            raise ValueError("missing workflow node: " + kind)
    active_sources = sum(kinds[k] for k in ("teleop_mapping", "trajectory_source", "policy_client"))
    if active_sources != 1:
        raise ValueError("one action producer required per workflow graph")
    seen = set()
    incoming = defaultdict(list)
    adj = defaultdict(list)
    indegree = {name: 0 for name in nodes}
    typed = []
    for edge in c["edges"]:
        if set(edge) != {"from", "to", "delayed"} or type(edge["delayed"]) is not bool:
            raise ValueError("explicit graph edge/timing required")
        endpoints = []
        for key in ("from", "to"):
            if not isinstance(edge[key], str) or edge[key].count(".") != 1:
                raise ValueError("node.port edge required")
            node_id, port = edge[key].split(".")
            if node_id not in nodes:
                raise ValueError("edge node missing")
            ports = CATALOG[nodes[node_id]["kind"]][1 if key == "from" else 0]
            if port not in ports:
                raise ValueError("edge direction/port mismatch")
            endpoints.append((node_id, port, ports[port]))
        a, b = endpoints
        if a[2] != b[2]:
            raise ValueError("edge type mismatch")
        key = (edge["from"], edge["to"])
        if key in seen:
            raise ValueError("duplicate edge")
        seen.add(key)
        incoming[b[:2]].append(a)
        if nodes[b[0]]["kind"] == "driver" and nodes[a[0]]["kind"] != "gateway":
            raise ValueError("driver writes must pass the shared gateway")
        if edge["delayed"] and nodes[a[0]]["kind"] not in ("driver", "history"):
            raise ValueError("only feedback/history edges may cross ticks")
        if not edge["delayed"]:
            adj[a[0]].append(b[0])
            indegree[b[0]] += 1
        typed.append({**edge, "type": a[2]})
    for name, node in nodes.items():
        for port in CATALOG[node["kind"]][0]:
            feeds = incoming[(name, port)]
            if not feeds:
                raise ValueError("unbound required input: " + name + "." + port)
            if len(feeds) > 1 and port != "observations":
                raise ValueError("ambiguous input producer: " + name + "." + port)
    queue = deque(name for name, degree in indegree.items() if degree == 0)
    order = []
    while queue:
        name = queue.popleft()
        order.append(name)
        for target in adj[name]:
            indegree[target] -= 1
            if not indegree[target]:
                queue.append(target)
    if len(order) != len(nodes):
        raise ValueError("instantaneous graph cycle; feedback/history must be delayed")
    source = next(
        n
        for n in nodes.values()
        if n["kind"] in ("teleop_mapping", "trajectory_source", "policy_client")
    )
    motion = next(n for n in nodes.values() if n["kind"] == "motion_adapter")
    encoding = source["config"].get("action_encoding")
    if encoding not in (
        "joint_absolute",
        "eef_absolute_matrix",
        "eef_initial_relative_xyzw",
    ) or encoding != motion["config"].get("action_encoding"):
        raise ValueError("explicit matching action encoding required")
    # No coordinate retargeting is selected by the compiler itself.
    for key in ("frame_mapping", "gripper_mapping"):
        if not isinstance(motion["config"].get(key), str) or not motion["config"][key]:
            raise ValueError("explicit " + key + " declaration required")
    return {
        "schema_version": "experiment-plan/1",
        "mode": c["mode"],
        "plan_only": True,
        "physical_commands": False,
        "runtime_status": "typed plan only; physical graph runner not implemented",
        "deployment": c["deployment"],
        "instantaneous_order": order,
        "nodes": list(nodes.values()),
        "edges": typed,
        "action_encoding": encoding,
    }
