"""Synthetic native-joint sources share one multi-endpoint collection path."""

import math

from .collection import validate_collection
from .collection_episode import CollectionRecorder, read_collection
from .collection_runtime import CollectionSession


def run_collection_demo(config, output, *, steps=20):
    if type(steps) is not int or not 2 <= steps <= 10000:
        raise ValueError("demo steps must be 2..10000")
    plan = validate_collection(config)
    if len(plan["config"]["sources"]) < 2:
        raise ValueError("demo requires two explicit sources to exercise source replacement")
    sources = [s["id"] for s in plan["config"]["sources"][:2]]
    with CollectionRecorder(output) as recorder:
        session = CollectionSession(config, sink=recorder.write)
        session.start()
        source = sources[0]
        lease = session.acquire(source)
        for step in range(steps):
            if step == steps // 2:
                session.release(lease)
                source = sources[1]
                lease = session.acquire(source)
            lease = session.renew(lease)
            snapshot = session.observe()
            targets = {}
            for endpoint, spec in plan["endpoints"].items():
                targets[endpoint] = [
                    initial
                    + min(spec["max_step"] / 4, (high - initial) / 2, (initial - low) / 2)
                    * math.sin(step * 0.1)
                    for initial, (low, high) in zip(spec["initial"], spec["limits"])
                ]
            intent = session.intent(source, targets, lease, snapshot)
            session.emit(
                "raw_source",
                {
                    "source": source,
                    "seq": intent.seq,
                    "raw_time_ns": session.clock.now_ns,
                    "kind": "synthetic_teleop_joint_targets"
                    if source == sources[0]
                    else "stub_policy_joint_targets",
                    "targets": targets,
                    "snapshot_id": snapshot["snapshot_id"],
                },
            )
            result = session.submit(intent)
            if result["decision"] != "executed_fake":
                raise ValueError(f"fake collection rejected: {result}")
            session.clock.advance(plan["config"]["timing"]["period_ns"])
        recorder.finish(session.finish())
    return read_collection(output)
