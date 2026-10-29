import json
import threading
import time
from pathlib import Path

import pytest

from robot_infra.adapters.realsense_ros import driver_arguments, validate_config
from robot_infra.camera_config import pipeline
from robot_infra.episode_qc import quality_report
from robot_infra.episode_store import EpisodeStore
from robot_infra.image_codec import decode_image, encode_image
from robot_infra.real_episode import EpisodeRecorder, RateGate, RawEpisodeWriter


def recording_config():
    return json.loads(
        (Path(__file__).parents[1] / "configs/teleop-session.example.json").read_text()
    )["recording"]


def cameras():
    p = {"width": 2, "height": 2, "fps": 30}
    return {
        r: {
            "capture": {"rgb": p.copy(), "depth": None},
            "preview": {"width": 1, "height": 1, "fps": 15, "jpeg_quality": 70},
            "record": {"rgb": p.copy(), "depth": None},
        }
        for r in ("custom_front", "custom_wrist")
    }


def context():
    return {"camera_pipeline": cameras(), "arms": ["custom_arm"]}


def config():
    c = recording_config()
    c["alignment"]["reference_stream"] = "camera/custom_front/rgb"
    c["min_free_bytes"] = 1
    return c


def wait_for(recorder, state, timeout=4):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        snap = recorder.snapshot()
        if snap["state"] == state:
            return snap
        if snap["state"] == "ERROR":
            raise AssertionError(snap)
        time.sleep(0.005)
    raise AssertionError(recorder.snapshot())


def image(stamp=1_000_000_000):
    return {
        "width": 2,
        "height": 2,
        "step": 6,
        "encoding": "rgb8",
        "stamp_ns": stamp + 100,
        "host_monotonic_ns": stamp,
        "host_unix_ns": stamp + 1_000_000_000,
        "is_bigendian": 0,
        "frame_id": "optical",
    }


def test_native_and_resized_RGB_depth_envelopes():
    meta = image()
    raw = bytes(range(12))
    m, b = decode_image(encode_image(meta, raw))
    assert m == meta and b == raw
    depth = {**meta, "width": 1, "height": 1, "step": 2, "encoding": "16UC1", "is_bigendian": 1}
    assert decode_image(encode_image(depth, b"\x01\x02")) == (depth, b"\x01\x02")
    with pytest.raises(ValueError):
        decode_image(b"\xff\xff\xff\xff")


def test_capture_preview_record_are_independent_and_parameters_explicit():
    c = json.loads(
        (Path(__file__).parents[1] / "configs/realsense-streams.example.json").read_text()
    )
    for i, item in enumerate(c["cameras"]):
        item["serial"] = str(12345 + i)
    validate_config(c)
    item = c["cameras"][0]
    args = driver_arguments("/driver", item)
    assert "rgb_camera.color_profile:=640x480x30" in args and "enable_depth:=false" in args
    assert (
        pipeline(item)["preview"]["width"] == 480
        and pipeline(item)["record"]["rgb"]["width"] == 640
    )
    depth = {"width": 640, "height": 480, "fps": 30}
    item["capture"]["depth"] = depth
    item["record"]["depth"] = depth.copy()
    item["driver_parameters"] = {"rgb_camera.enable_auto_exposure": False}
    args = driver_arguments("/driver", item)
    assert "enable_depth:=true" in args and "depth_module.depth_profile:=640x480x30" in args
    assert "rgb_camera.enable_auto_exposure:=false" in args
    item["preview"]["width"] = 1280
    with pytest.raises(ValueError, match="exceed"):
        validate_config(c)
    item["preview"]["width"] = 480
    item["driver_parameters"] = {"enable_color": False}
    with pytest.raises(ValueError, match="reserved"):
        validate_config(c)


def test_task_lock_numbering_skip_and_preserved_overwrite(tmp_path):
    a = EpisodeStore(tmp_path, "test task", 3, "error")
    p = a.begin()
    (p / "data").write_text("old")
    a.commit(p, True)
    with pytest.raises(FileExistsError):
        EpisodeStore(tmp_path, "test task", 0, "skip")
    a.close()
    with pytest.raises(FileExistsError):
        EpisodeStore(tmp_path, "test task", 3, "error")
    b = EpisodeStore(tmp_path, "test task", 3, "skip")
    assert b.index == 4
    b.close()
    c = EpisodeStore(tmp_path, "test task", 3, "overwrite")
    p = c.begin()
    (p / "data").write_text("new")
    assert (c.root / "episodes/episode_000003/data").read_text() == "old"
    c.commit(p, True)
    assert (c.root / "episodes/episode_000003/data").read_text() == "new"
    assert [p.read_text() for p in (c.root / "backups").glob("*/data")] == ["old"]
    c.close()


def test_discard_preserves_evidence_and_reuses_number(tmp_path):
    a = EpisodeStore(tmp_path, "task", 0, "skip")
    p = a.begin()
    (p / "manifest.json").write_text("{}")
    target = a.commit(p, False)
    assert target.parent.name == "discarded" and a.index == 0
    p = a.begin()
    assert p.name.startswith("episode_000000_")
    a.close()
    b = EpisodeStore(tmp_path, "task", 0, "skip")
    assert b.index == 1
    b.close()


def test_MCAP_raw_roundtrip_QC_alignment_and_corruption(tmp_path):
    ctx = context()
    cfg = config()
    expected = {
        "camera/" + r + "/rgb": p["record"]["rgb"] for r, p in ctx["camera_pipeline"].items()
    }
    expected["feedback/custom_arm"] = {}
    manifest = {
        "schema_version": "real-episode/1",
        "task": "task",
        "episode": 0,
        "start_monotonic_ns": 1_000_000_000,
        "recording": cfg,
        "camera_pipeline": ctx["camera_pipeline"],
        "expected_streams": expected,
    }
    writer = RawEpisodeWriter(tmp_path, manifest)
    for i in range(31):
        stamp = 1_000_000_000 + round(i * 1e9 / 30)
        for role in ctx["camera_pipeline"]:
            writer.frame(role, "rgb", image(stamp), bytes(range(12)))
        writer.write_event(
            "feedback",
            {"side": "custom_arm", "q": [0.2], "device_stamp_ns": stamp},
            stamp,
            stamp + 100,
        )
    for kind in cfg["required_events"]:
        writer.write_event(kind, {"test": True}, 1_500_000_000, 1_500_000_100)
    qc = writer.finish(
        {"end_monotonic_ns": 2_000_000_000, "interrupted": None, "dropped": 0, "writer_error": None}
    )
    assert qc["passed"], qc
    m = json.loads((tmp_path / "manifest.json").read_text())
    assert quality_report(tmp_path, m, deep=True)["passed"]
    assert (
        json.loads((tmp_path / "alignment.jsonl").read_text().splitlines()[0])["matches"][
            "feedback/custom_arm"
        ]["delta_ns"]
        == 0
    )
    from mcap.reader import NonSeekingReader

    with (tmp_path / "streams.mcap").open("rb") as source:
        messages = [
            (c, m)
            for s, c, m in NonSeekingReader(source).iter_messages(log_time_order=False)
            if c.message_encoding == "robot-infra-raw-image/1"
        ]
    assert len(messages) == 62 and decode_image(messages[0][1].data)[1] == bytes(range(12))
    with (tmp_path / "streams.mcap").open("r+b") as source:
        source.seek(100)
        source.write(b"BAD")
    with pytest.raises(ValueError, match="hash"):
        quality_report(tmp_path, m, deep=True)


def test_async_start_finish_discard_close_and_interrupted_QC(tmp_path):
    r = EpisodeRecorder(config(), tmp_path, "task", 0, "skip")
    try:
        r.start(context())
        wait_for(r, "RECORDING")
        now = time.monotonic_ns()
        r.frame("custom_front", "rgb", image(now), bytes(range(12)))
        r.event("raw_input", {"timestamp": 17})
        r.finish("VR_hidden")
        s = wait_for(r, "REVIEW")
        assert not s["qc"]["passed"] and any("interrupted" in x for x in s["qc"]["issues"])
        r.decide(False)
        s = wait_for(r, "IDLE")
        assert s["discarded_episodes"] == 1 and s["episode"] == 0
    finally:
        r.close()
    assert not (r.store.root / ".collector.lock").exists() and not r.thread.is_alive()


def test_overflow_never_blocks_or_silently_publishes_complete(tmp_path):
    entered, release = threading.Event(), threading.Event()

    class SlowWriter(RawEpisodeWriter):
        def frame(self, *args):
            entered.set()
            release.wait(3)
            return super().frame(*args)

    cfg = config()
    cfg["queue_bytes"] = 1100
    r = EpisodeRecorder(cfg, tmp_path, "task", 0, "error", writer_factory=SlowWriter)
    try:
        r.start(context())
        wait_for(r, "RECORDING")
        meta = image(time.monotonic_ns())
        r.frame("custom_front", "rgb", meta, b"\x01" * 12)
        assert entered.wait(1)
        assert r.frame("custom_front", "rgb", meta, b"\x01" * 12)
        start = time.monotonic()
        assert not r.frame("custom_front", "rgb", meta, b"\x01" * 12)
        assert time.monotonic() - start < 0.1
        assert r.snapshot()["dropped"] == 1
        release.set()
        s = wait_for(r, "REVIEW")
        assert not s["qc"]["passed"] and s["error"] == "recording_queue_overflow"
    finally:
        release.set()
        r.close()


def test_disk_failure_retains_incomplete_and_releases_task_on_close(tmp_path):
    class BrokenWriter(RawEpisodeWriter):
        def frame(self, *args):
            raise OSError("simulated disk failure")

    r = EpisodeRecorder(config(), tmp_path, "task", 0, "error", writer_factory=BrokenWriter)
    r.start(context())
    wait_for(r, "RECORDING")
    r.frame("custom_front", "rgb", image(), b"\x00" * 12)
    until = time.monotonic() + 2
    while r.snapshot()["state"] != "ERROR" and time.monotonic() < until:
        time.sleep(0.005)
    assert r.snapshot()["state"] == "ERROR"
    r.close()
    assert list((r.store.root / ".incomplete").glob("*/failure.json")) and not list(
        (r.store.root / "episodes").iterdir()
    )


def test_rate_gate_handles_small_native_jitter_without_halving_fps():
    gate = RateGate(30)
    stamps = [round(i * 1e9 / 30) + (-100000 if i % 2 else 100000) for i in range(120)]
    assert sum(gate.accept(t) for t in stamps) == 120


def test_overwrite_failure_restores_old_episode(tmp_path, monkeypatch):
    import robot_infra.episode_store as storage

    store = EpisodeStore(tmp_path, "task", 0, "overwrite")
    old = store.root / "episodes/episode_000000"
    old.mkdir()
    (old / "data").write_text("old")
    path = store.begin()
    (path / "data").write_text("new")
    replace = storage.os.replace

    def fail_new(source, target):
        if Path(source) == path:
            raise OSError("simulated commit failure")
        return replace(source, target)

    monkeypatch.setattr(storage.os, "replace", fail_new)
    with pytest.raises(OSError):
        store.commit(path, True)
    assert (old / "data").read_text() == "old" and (path / "data").read_text() == "new"
    store.close()


def test_depth_and_resized_RGB_MCAP_preserve_native_metadata(tmp_path):
    ctx = context()
    p = ctx["camera_pipeline"]["custom_front"]
    p["record"]["rgb"] = {"width": 1, "height": 1, "fps": 30}
    p["capture"]["depth"] = {"width": 1, "height": 1, "fps": 30}
    p["record"]["depth"] = p["capture"]["depth"].copy()
    manifest = {
        "schema_version": "real-episode/1",
        "start_monotonic_ns": 1,
        "recording": config(),
        "camera_pipeline": ctx["camera_pipeline"],
        "expected_streams": {},
    }
    writer = RawEpisodeWriter(tmp_path, manifest)
    writer.frame("custom_front", "rgb", image(), bytes(range(12)))
    depth = {**image(), "width": 1, "height": 1, "encoding": "16UC1", "step": 2, "is_bigendian": 1}
    writer.frame("custom_front", "depth", depth, b"\x01\x02")
    writer.finish({"end_monotonic_ns": 2_000_000_000, "interrupted": None})
    from mcap.reader import NonSeekingReader

    with (tmp_path / "streams.mcap").open("rb") as source:
        values = {
            c.topic: decode_image(m.data)
            for s, c, m in NonSeekingReader(source).iter_messages(log_time_order=False)
            if c.message_encoding == "robot-infra-raw-image/1"
        }
    rgb = values["/camera/custom_front/rgb"]
    assert len(rgb[1]) == 3 and rgb[0]["source_geometry"]["width"] == 2
    assert values["/camera/custom_front/depth"] == (depth, b"\x01\x02")


def test_disk_reserve_failure_does_not_create_a_saved_episode(tmp_path):
    cfg = config()
    cfg["min_free_bytes"] = 10**30
    r = EpisodeRecorder(cfg, tmp_path, "task", 0, "skip")
    r.start(context())
    until = time.monotonic() + 2
    while r.snapshot()["state"] != "ERROR" and time.monotonic() < until:
        time.sleep(0.005)
    assert "disk space" in r.snapshot()["error"]
    r.close()
    assert not list((r.store.root / "episodes").iterdir())
