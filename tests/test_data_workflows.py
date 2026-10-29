import hashlib
import json
from copy import deepcopy
from pathlib import Path

import pytest
from test_real_episode import cameras, config, image

from robot_infra.data_audit import audit_episode, audit_journal
from robot_infra.diagnostics import EventJournal, diagnostic_config, journal_segments
from robot_infra.experiment_graph import compile_experiment
from robot_infra.real_episode import EpisodeWriter
from robot_infra.training_view import (
    build_training_view,
    published_targets,
    validate_training_profile,
    view_rows,
)
from robot_infra.trajectory import TimelineSource, trajectory_plan


def profile():
    return {
        "schema_version": "training-view/1",
        "fps": 30,
        "endpoints": [
            {
                "id": "custom.arm",
                "side": "custom",
                "names": ["j1", "j2"],
                "feedback_field": "q",
                "command_field": "q",
                "unit": "rad",
            },
            {
                "id": "custom.jaw",
                "side": "custom",
                "names": ["width"],
                "feedback_field": "gripper",
                "command_field": "gripper_m",
                "unit": "m",
            },
        ],
        "visuals": [
            {
                "feature": "observation.images.front",
                "stream": "camera/custom_front/rgb",
                "required": True,
                "depth_m_per_unit": None,
            }
        ],
        "sampling": {"max_state_age_s": 0.1, "max_image_age_s": 0.1, "action_offset_steps": 1},
        "quality": {"require_saved": True, "require_qc_pass": True},
    }


def episode(root):
    root.mkdir()
    cfg = config()
    cfg["required_events"] = []
    p = cameras()
    expected = {"camera/" + r + "/rgb": d["record"]["rgb"] for r, d in p.items()}
    expected["feedback/custom"] = {}
    m = {
        "schema_version": "real-episode/1",
        "task": "test",
        "episode": 0,
        "status": "saved",
        "start_monotonic_ns": 1_000_000_000,
        "recording": cfg,
        "camera_pipeline": p,
        "expected_streams": expected,
        "context": {"config": {"model": {"joint_names": ["j1", "j2"]}}},
    }
    writer = EpisodeWriter(root, m)
    for i in range(31):
        stamp = 1_000_000_000 + round(i * 1e9 / 30)
        for role in p:
            writer.frame(role, "rgb", image(stamp), bytes(range(12)))
        writer.write_event(
            "feedback",
            {
                "side": "custom",
                "q": [0.1, 0.2] if i < 2 else [0.9, 0.8],
                "gripper": 0.03,
                "device_stamp_ns": stamp,
            },
            stamp,
            stamp + 100,
        )
        if i == 1:
            writer.write_event(
                "command",
                {"side": "custom", "q": [0.4, 0.5], "gripper_m": 0.0, "seq": 7},
                stamp + 1,
                stamp + 101,
            )
            writer.write_event(
                "receipt",
                {"side": "custom", "seq": 7, "status": "published_not_acknowledged"},
                stamp + 2,
                stamp + 102,
            )
    writer.write_event(
        "raw_input",
        {
            "seq": 1,
            "client_time_ms": 5.0,
            "transport": {"previous_rtt_ms": 250},
            "previous_frame_work_ms": 0.5,
        },
        1_100_000_000,
        2_100_000_000,
    )
    qc = writer.finish(
        {"end_monotonic_ns": 2_000_000_000, "interrupted": None, "dropped": 0, "writer_error": None}
    )
    assert qc["passed"]
    m = json.loads((root / "manifest.json").read_text())
    m["status"] = "saved"
    (root / "manifest.json").write_text(json.dumps(m))
    return root


def test_readonly_deep_audit_and_command_training_labels(tmp_path):
    source = episode(tmp_path / "episode")
    before = {p.name: p.read_bytes() for p in source.glob("*.json*")}
    audit = audit_episode(source, tmp_path / "audit", deep=True)
    assert (
        audit["qc"]["passed"]
        and audit["qc"]["deep_integrity_checked"]
        and audit["inputs"]["rtt_ms"]["max"] == 250
    )
    assert before == {p.name: p.read_bytes() for p in source.glob("*.json*")}
    rows = list(view_rows(source, profile()))
    assert rows[0]["action_valid"] == [False, False, False]  # No fabricated starting targets.
    assert rows[1]["observation.state"] == [0.1, 0.2, 0.03]  # Future state must not leak backward.
    assert rows[1]["action"] == [0.4, 0.5, 0.0]  # Closing goal, not .03m contact-limited feedback.
    assert (
        rows[1]["evidence"]["custom.jaw"]["target"]["confirmation"] == "published_not_acknowledged"
    )
    assert (
        rows[1]["visuals"]["observation.images.front"]["row"]["received_ns"]
        <= rows[1]["observation_time_ns"]
    )
    report = build_training_view(source, profile(), tmp_path / "view")
    assert (
        report["valid_rows"] > 0
        and report["units"] == ["rad", "rad", "m"]
        and "not LeRobot" in report["format"]
    )
    with pytest.raises(ValueError):
        audit_episode(source, source / "bad")
    with pytest.raises(ValueError):
        build_training_view(source, profile(), source / "bad")


def test_unknown_targets_receipt_failure_and_journal_seed(tmp_path):
    source = episode(tmp_path / "episode")
    p = profile()
    p["endpoints"][1]["id"] = "other.jaw"
    p["endpoints"][1]["side"] = "other"
    unknown = list(view_rows(source, p))
    assert all(
        r["observation.state"][-1] is None and r["action_valid"][-1] is False and not r["valid"]
        for r in unknown
    )  # No implicit cross-arm mapping.

    def record(kind, payload, stamp):
        return {
            "event": {"kind": kind, "payload": payload, "host_monotonic_ns": stamp},
            "evidence": {"kind": "test"},
        }

    assert (
        published_targets(
            [record("command", {"side": "custom", "q": [0.4, 0.5], "seq": 1}, 100)],
            profile()["endpoints"],
        )
        == {}
    )
    assert (
        published_targets(
            [
                record("command", {"side": "custom", "q": [0.4, 0.5], "seq": 1}, 100),
                record("receipt", {"side": "custom", "seq": 1, "status": "failed"}, 101),
            ],
            profile()["endpoints"],
        )
        == {}
    )
    journal = tmp_path / "events.jsonl"
    entries = [
        record("command", {"side": "custom", "q": [0.2, 0.3], "gripper_m": 0.0, "seq": 3}, 100),
        record(
            "receipt", {"side": "custom", "seq": 3, "status": "published_not_acknowledged"}, 101
        ),
    ]
    journal.write_text("".join(json.dumps(e["event"]) + "\n" for e in entries))
    row = next(view_rows(source, profile(), journal_paths=[journal]))
    assert row["action"] == [0.2, 0.3, 0.0] and row["action_valid"] == [True] * 3
    assert row["evidence"]["custom.jaw"]["target"]["evidence"]["kind"] == "session_journal"


def test_depth_absence_joint_order_and_quality_do_not_fabricate_data(tmp_path):
    source = episode(tmp_path / "episode")
    c = profile()
    c["visuals"].append(
        {
            "feature": "observation.depth.front",
            "stream": "camera/custom_front/depth",
            "required": True,
            "depth_m_per_unit": None,
        }
    )
    with pytest.raises(ValueError, match="scale"):
        validate_training_profile(c)
    c["visuals"][-1]["depth_m_per_unit"] = 0.001
    with pytest.raises(ValueError, match="missing"):
        list(view_rows(source, c))
    c = profile()
    c["endpoints"][0]["names"].reverse()
    with pytest.raises(ValueError, match="joint order"):
        list(view_rows(source, c))
    m = json.loads((source / "manifest.json").read_text())
    m["status"] = "review"
    (source / "manifest.json").write_text(json.dumps(m))
    with pytest.raises(ValueError, match="saved"):
        list(view_rows(source, profile()))


def test_offline_timeline_keeps_original_times_and_never_replays_lease(tmp_path):
    source = episode(tmp_path / "episode")
    plan = trajectory_plan(source, profile())
    assert plan["physical_commands"] is False
    assert len(plan["events"]) == 2 and all(
        "lease" not in e and "command" not in e for e in plan["events"]
    )
    clock = TimelineSource(plan)
    first = plan["events"][0]["offset_ns"]
    assert clock.poll(first - 1) == []
    assert len(clock.poll(first)) == 2 and clock.poll(first) == []
    with pytest.raises(ValueError):
        clock.poll(first - 1)


@pytest.mark.parametrize("mode", ["collection", "replay", "inference"])
def test_typed_workflow_graphs_compile_without_hardware(mode):
    path = Path(__file__).parents[1] / "configs/graphs" / ("a4s-" + mode + ".json")
    c = json.loads(path.read_text())
    plan = compile_experiment(c)
    assert plan["mode"] == mode and plan["physical_commands"] is False and plan["plan_only"]
    bad = deepcopy(c)
    bad["edges"][0]["from"] = "cameras.missing"
    with pytest.raises(ValueError):
        compile_experiment(bad)
    bad = deepcopy(c)
    next(n for n in bad["nodes"] if n["kind"] == "motion_adapter")["config"]["action_encoding"] = (
        "unknown"
    )
    with pytest.raises(ValueError, match="encoding"):
        compile_experiment(bad)
    bad = deepcopy(c)
    bad["edges"].append(deepcopy(bad["edges"][0]))
    with pytest.raises(ValueError, match="duplicate"):
        compile_experiment(bad)


def test_segmented_diagnostics_preserve_every_event_and_checksums(tmp_path):
    journal = EventJournal(tmp_path / "journal", segment_bytes=300, min_free_bytes=1)
    for i in range(20):
        journal.emit("test", {"i": i})
    journal.close()
    summary = json.loads((journal.root / "journal-summary.json").read_text())
    assert (
        summary["error"] is None
        and len(summary["segments"]) > 1
        and summary["counts"]["test"] == 20
    )
    events = []
    for item, path in zip(summary["segments"], journal_segments(journal.root)):
        raw = path.read_bytes()
        assert item["sha256"] == hashlib.sha256(raw).hexdigest() and item["bytes"] == len(raw)
        events.extend(json.loads(line) for line in raw.splitlines())
    assert [e["payload"]["i"] for e in events] == list(range(20))
    assert audit_journal(journal_segments(journal.root))["counts"]["test"] == 20
    assert diagnostic_config()["max_bytes"] is None


def test_diagnostic_explicit_capacity_retains_error_evidence(tmp_path):
    journal = EventJournal(tmp_path / "journal", max_bytes=1, min_free_bytes=1)
    journal.emit("test", {})
    journal.close()
    summary = json.loads((journal.root / "journal-summary.json").read_text())
    assert summary["error"] == "journal capacity reached" and summary["writer_stopped"]


def test_native_rate_recording_keeps_frames_delivered_in_bursts(tmp_path):
    root = tmp_path / "burst"
    root.mkdir()
    cfg = config()
    cfg["required_events"] = []
    p = cameras()
    m = {
        "schema_version": "real-episode/1",
        "task": "test",
        "episode": 0,
        "start_monotonic_ns": 1_000_000_000,
        "recording": cfg,
        "camera_pipeline": p,
        "expected_streams": {
            "camera/" + role + "/rgb": d["record"]["rgb"] for role, d in p.items()
        },
    }
    writer = EpisodeWriter(root, m)
    for i in range(31):
        # Three frames delivered close together; preserve all at capture FPS.
        stamp = 1_000_000_000 + (i // 3) * 100_000_000 + (i % 3) * 5_000_000
        for role in p:
            writer.frame(
                role,
                "rgb",
                {**image(stamp), "stamp_ns": 1_000_000_000 + round(i * 1e9 / 30)},
                bytes(range(12)),
            )
    writer.finish(
        {"end_monotonic_ns": 2_100_000_000, "interrupted": None, "dropped": 0, "writer_error": None}
    )
    manifest = json.loads((root / "manifest.json").read_text())
    assert all(
        v == {"received_by_writer": 31, "rate_filtered": 0, "written": 31}
        for v in manifest["frame_capture_stats"].values()
    )


def test_future_episode_initial_target_snapshot_keeps_closing_goal(tmp_path):
    source = episode(tmp_path / "episode")
    m = json.loads((source / "manifest.json").read_text())
    m["context"]["initial_targets"] = {
        "custom": {
            "q": {"values": [0.3, 0.4], "time_ns": 900_000_000},
            "gripper_m": {"values": [0.0], "time_ns": 900_000_000},
        }
    }
    (source / "manifest.json").write_text(json.dumps(m))
    row = next(view_rows(source, profile()))
    assert row["action"] == [0.3, 0.4, 0.0] and row["observation.state"][-1] == 0.03
    assert row["evidence"]["custom.jaw"]["target"]["confirmation"] == "snapshot_after_publish"
