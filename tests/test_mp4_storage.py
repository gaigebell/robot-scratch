import hashlib
import json
import time
from pathlib import Path

import pytest

av = pytest.importorskip("av")
from robot_infra.episode_qc import quality_report
from robot_infra.image_codec import decode_image
from robot_infra.image_storage import MP4Storage, check_image_storage, storage_config
from robot_infra.real_episode import EpisodeRecorder, EpisodeWriter


def config():
    c = json.loads((Path(__file__).parents[1] / "configs/teleop-session.example.json").read_text())[
        "recording"
    ]
    c["storage"] = {"image_backend": "mp4"}
    c["min_free_bytes"] = 1
    c["alignment"]["reference_stream"] = "camera/workcell/front view/rgb"
    return c


def context():
    profile = {"width": 128, "height": 96, "fps": 30}
    pipeline = {
        r: {
            "capture": {"rgb": profile.copy(), "depth": None},
            "record": {"rgb": profile.copy(), "depth": None},
        }
        for r in ["workcell/front view", "another_sensor"]
    }
    depth = {"width": 16, "height": 12, "fps": 30}
    pipeline["another_sensor"]["capture"]["depth"] = depth.copy()
    pipeline["another_sensor"]["record"]["depth"] = depth.copy()
    return {"camera_pipeline": pipeline, "arms": ["generic_one_joint_arm"]}


def meta(stamp, *, bgr=False, depth=False):
    width, height, encoding, step = (
        (16, 12, "16UC1", 32) if depth else (128, 96, "bgr8" if bgr else "rgb8", 388)
    )
    return {
        "width": width,
        "height": height,
        "step": step,
        "encoding": encoding,
        "is_bigendian": int(depth),
        "stamp_ns": stamp + 10,
        "host_monotonic_ns": stamp,
        "host_unix_ns": stamp + 10**9,
        "frame_id": "custom optical",
    }


def pixels(*, bgr=False):
    color = bytes([30, 90, 210] if bgr else [210, 90, 30])
    return (color * 128 + b"\xff" * 4) * 96


def wait(recorder, state):
    end = time.monotonic() + 10
    while time.monotonic() < end:
        s = recorder.snapshot()
        if s["state"] == state:
            return s
        if s["state"] == "ERROR":
            raise AssertionError(s)
        time.sleep(0.005)
    raise AssertionError(recorder.snapshot())


@pytest.fixture
def episode(tmp_path):
    cfg = config()
    ctx = context()
    expected = {}
    for role, p in ctx["camera_pipeline"].items():
        for kind, definition in p["record"].items():
            if definition:
                expected["camera/" + role + "/" + kind] = definition
    expected["feedback/generic_one_joint_arm"] = {}
    manifest = {
        "schema_version": "real-episode/2",
        "recording": cfg,
        "camera_pipeline": ctx["camera_pipeline"],
        "start_monotonic_ns": 10**9,
        "expected_streams": expected,
    }
    writer = EpisodeWriter(tmp_path, manifest)
    for i in range(31):
        stamp = 10**9 + round(i * 10**9 / 30) + (100_000 if i % 2 else 0)
        writer.frame("workcell/front view", "rgb", meta(stamp), pixels())
        writer.frame("another_sensor", "rgb", meta(stamp, bgr=True), pixels(bgr=True))
        writer.frame("another_sensor", "depth", meta(stamp, depth=True), b"\x01\x02" * 16 * 12)
        writer.write_event(
            "feedback",
            {"side": "generic_one_joint_arm", "q": [0.3], "device_stamp_ns": stamp},
            stamp,
            stamp + 100,
        )
    for kind in cfg["required_events"]:
        writer.write_event(kind, {"example": True}, 1_500_000_000, 1_500_000_100)
    qc = writer.finish(
        {"end_monotonic_ns": 2_000_000_000, "interrupted": None, "dropped": 0, "writer_error": None}
    )
    assert qc["passed"], qc
    return tmp_path, json.loads((tmp_path / "manifest.json").read_text())


def test_real_mp4_color_stride_depth_timing_and_deep_QC(episode):
    root, manifest = episode
    report = quality_report(root, manifest, deep=True)
    assert report["passed"] and report["deep_integrity_checked"]
    assert len(manifest["media"]) == 2 and all(
        v["decoded_frames"] == 31 for v in report["media"].values()
    )
    assert (
        sum(p.stat().st_size for p in root.rglob("*.mp4")) + (root / "streams.mcap").stat().st_size
        < 2 * 31 * 128 * 96 * 3 / 2
    )
    for name, asset in manifest["media"].items():
        with av.open(str(root / asset["path"])) as source:
            frames = list(source.decode(video=0))
            assert len(frames) == 31
            image = frames[0].reformat(format="rgb24")
            plane = image.planes[0]
            position = 10 * plane.line_size + 10 * 3
            actual = bytes(plane)[position : position + 3]
            assert all(abs(a - b) < 8 for a, b in zip(actual, [210, 90, 30])), actual
    from mcap.reader import NonSeekingReader

    references = []
    depth = []
    with (root / "streams.mcap").open("rb") as source:
        for schema, channel, message in NonSeekingReader(source).iter_messages(
            log_time_order=False
        ):
            if channel.message_encoding == "robot-infra-raw-image/1":
                depth.append(decode_image(message.data))
            elif channel.topic == "/camera/workcell/front view/rgb":
                references.append(json.loads(message.data))
    assert (
        len(depth) == 31
        and depth[0][0]["is_bigendian"] == 1
        and depth[0][1] == b"\x01\x02" * 16 * 12
    )
    assert references[1]["image"]["stamp_ns"] == 10**9 + round(10**9 / 30) + 100_000 + 10
    assert references[1]["media"]["pts"] == 1 and references[1]["media"]["time_base"] == [1, 30]
    assert references[0]["image"]["step"] == 388  # original row stride preserved


def test_corrupt_video_is_rejected(episode):
    root, m = episode
    p = root / next(iter(m["media"].values()))["path"]
    with p.open("r+b") as f:
        data = f.read()
        index = data.index(b"mdat") + 16
        f.seek(index)
        f.write(bytes([data[index] ^ 1]))
    with pytest.raises(ValueError, match="video.*hash"):
        quality_report(root, m, deep=True)


def test_missing_or_traversing_video_is_rejected(episode):
    root, m = episode
    asset = next(iter(m["media"].values()))
    original = asset["path"]
    asset["path"] = "../outside.mp4"
    with pytest.raises(ValueError, match="inside episode"):
        quality_report(root, m, deep=True)
    asset["path"] = original
    (root / original).unlink()
    with pytest.raises(ValueError, match="file missing"):
        quality_report(root, m, deep=True)


def test_frame_index_mismatch_is_rejected_even_with_new_index_hash(episode):
    root, m = episode
    p = root / "index.jsonl"
    rows = [json.loads(line) for line in p.read_text().splitlines()]
    next(row for row in rows if row.get("media"))["media"]["frame_index"] = 99
    p.write_text("".join(json.dumps(row) + "\n" for row in rows))
    m["index_sha256"] = hashlib.sha256(p.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match="index/PTS"):
        quality_report(root, m, deep=True)


@pytest.mark.parametrize(
    "selection",
    [
        {"image_backend": "invalid"},
        {"image_backend": "mp4", "video": {"threads": 0}},
        {"image_backend": "mp4", "video": {"options": {"crf": 20}}},
    ],
)
def test_bad_storage_configuration(selection):
    with pytest.raises(ValueError):
        storage_config(selection)


def test_encoder_preflight_rejects_dimensions_codec_and_options():
    c = context()["camera_pipeline"]
    c["another_sensor"]["record"]["rgb"]["width"] = 127
    with pytest.raises(ValueError, match="even"):
        check_image_storage({"image_backend": "mp4"}, c)
    with pytest.raises(ValueError, match="preflight"):
        check_image_storage(
            {"image_backend": "mp4", "video": {"codec": "does_not_exist"}},
            context()["camera_pipeline"],
        )
    with pytest.raises(ValueError, match="options"):
        check_image_storage(
            {"image_backend": "mp4", "video": {"options": {"unknown_setting": "1"}}},
            context()["camera_pipeline"],
        )
    with pytest.raises(ValueError, match="preflight"):
        check_image_storage(
            {"image_backend": "mp4", "video": {"codec": "rawvideo", "options": {}}},
            context()["camera_pipeline"],
        )


def test_optional_dependency_missing_fails_before_creating_task(tmp_path, monkeypatch):
    import builtins

    original = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name == "av":
            raise ImportError("simulated missing optional library")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    with pytest.raises(ValueError, match="optional mp4"):
        EpisodeRecorder(config(), tmp_path / "absent", "task", 0, "skip")
    assert not (tmp_path / "absent").exists()
    raw_config = config()
    raw_config["storage"] = {"image_backend": "raw_mcap"}
    recorder = EpisodeRecorder(raw_config, tmp_path / "raw", "task", 0, "skip")
    try:
        recorder.start(context())
        wait(recorder, "RECORDING")
        recorder.frame("workcell/front view", "rgb", meta(time.monotonic_ns()), pixels())
        recorder.finish("short_raw_test")
        wait(recorder, "REVIEW")
    finally:
        recorder.close()


@pytest.mark.parametrize("save", [True, False])
def test_async_commit_includes_all_video_assets(tmp_path, save):
    recorder = EpisodeRecorder(config(), tmp_path, "task", 0, "skip")
    try:
        recorder.start(context())
        wait(recorder, "RECORDING")
        base = time.monotonic_ns()
        for i in range(4):
            recorder.frame("workcell/front view", "rgb", meta(base + i * 34_000_000), pixels())
        recorder.finish("short_test")
        wait(recorder, "REVIEW")
        recorder.decide(save)
        s = wait(recorder, "IDLE")
        target = Path(s["last_commit"])
        assert target.parent.name == ("episodes" if save else "discarded")
        manifest = json.loads((target / "manifest.json").read_text())
        assert manifest["schema_version"] == "real-episode/2"
        assert len(list((target / "videos").glob("*.mp4"))) == 1
        assert quality_report(target, manifest, deep=True)["deep_integrity_checked"]
        assert s["episode"] == (1 if save else 0)
    finally:
        recorder.close()
    assert not recorder.thread.is_alive() and not (recorder.store.root / ".collector.lock").exists()


@pytest.mark.parametrize("failure", ["encode", "flush"])
def test_encoder_failures_leave_incomplete_never_saved(tmp_path, monkeypatch, failure):
    observed = []
    if failure == "encode":
        original = MP4Storage.frame

        def broken(self, *args):
            original(self, *args)
            observed.append(self)
            raise OSError("simulated encode failure")

        monkeypatch.setattr(MP4Storage, "frame", broken)
    else:

        def broken(self):
            observed.append(self)
            raise OSError("simulated flush failure")

        monkeypatch.setattr(MP4Storage, "finish", broken)
    recorder = EpisodeRecorder(config(), tmp_path, "task", 0, "skip")
    recorder.start(context())
    wait(recorder, "RECORDING")
    recorder.frame("workcell/front view", "rgb", meta(time.monotonic_ns()), pixels())
    if failure == "flush":
        recorder.finish()
    end = time.monotonic() + 10
    while recorder.snapshot()["state"] != "ERROR" and time.monotonic() < end:
        time.sleep(0.005)
    assert recorder.snapshot()["state"] == "ERROR"
    recorder.close()
    assert list((recorder.store.root / ".incomplete").glob("*/failure.json")) and not list(
        (recorder.store.root / "episodes").iterdir()
    )
    assert observed and all(v["container"] is None for v in observed[0].videos.values())


def test_blocked_encoder_keeps_callback_nonblocking_and_reports_overflow(tmp_path, monkeypatch):
    import threading

    entered, release = threading.Event(), threading.Event()
    original = MP4Storage.frame

    def blocked(self, *args):
        entered.set()
        release.wait(5)
        return original(self, *args)

    monkeypatch.setattr(MP4Storage, "frame", blocked)
    c = config()
    c["queue_bytes"] = 40000
    r = EpisodeRecorder(c, tmp_path, "task", 0, "skip")
    try:
        r.start(context())
        wait(r, "RECORDING")
        base = time.monotonic_ns()
        assert r.frame("workcell/front view", "rgb", meta(base), pixels())
        assert entered.wait(2)
        assert r.frame("workcell/front view", "rgb", meta(base + 34_000_000), pixels())
        began = time.monotonic()
        assert not r.frame("workcell/front view", "rgb", meta(base + 68_000_000), pixels())
        assert time.monotonic() - began < 0.1
        release.set()
        s = wait(r, "REVIEW")
        assert s["dropped"] == 1 and not s["qc"]["passed"]
    finally:
        release.set()
        r.close()


def test_close_active_mp4_flushes_incomplete_and_releases_lock(tmp_path):
    r = EpisodeRecorder(config(), tmp_path, "task", 0, "skip")
    r.start(context())
    wait(r, "RECORDING")
    r.frame("workcell/front view", "rgb", meta(time.monotonic_ns()), pixels())
    r.close()
    assert not r.thread.is_alive() and not (r.store.root / ".collector.lock").exists()
    assert not list((r.store.root / "episodes").iterdir())
    target = next((r.store.root / ".incomplete").iterdir())
    m = json.loads((target / "manifest.json").read_text())
    assert m["interrupted"] == "process_exit" and m["status"] == "review"
    assert quality_report(target, m, deep=True)["deep_integrity_checked"]


def test_another_encoder_and_odd_native_depth_do_not_assume_hardware(tmp_path):
    from robot_infra.media_qc import inspect_media

    settings = {"image_backend": "mp4", "video": {"codec": "mpeg4", "options": {}}}
    c = check_image_storage(settings, context()["camera_pipeline"])
    backend = MP4Storage(tmp_path, c)
    streams = {}
    for name, fps in [("any/stream", 30), ("different_rate", 15)]:
        rows = []
        for i in range(3):
            m = meta(10**9 + round(i * 10**9 / fps))
            stored = backend.frame(name, m, pixels(), {"width": 128, "height": 96, "fps": fps})
            rows.append({"sequence": i, "media": stored.index["media"]})
        streams[name] = rows
    depth = {**meta(10**9, depth=True), "width": 1, "height": 1, "step": 2}
    raw = backend.frame("native/depth", depth, b"\x01\x02", {"width": 1, "height": 1, "fps": 30})
    assert decode_image(raw.body) == (depth, b"\x01\x02")
    assets = backend.finish()
    assert all(
        v["decoded_frames"] == 3
        for v in inspect_media(tmp_path, assets, streams, deep=True).values()
    )
    assert assets["different_rate"]["fps"] == 15


def test_three_RGB_MP4_and_head_only_depth_remains_byte_exact(tmp_path):
    from mcap.reader import NonSeekingReader

    cfg = config()
    cfg["alignment"]["reference_stream"] = "camera/scene_fixed/rgb"
    rgb = {"width": 128, "height": 96, "fps": 30}
    d = {"width": 16, "height": 12, "fps": 30}
    pipeline = {
        r: {
            "capture": {"rgb": rgb.copy(), "depth": d.copy() if r == "scene_fixed" else None},
            "record": {"rgb": rgb.copy(), "depth": d.copy() if r == "scene_fixed" else None},
        }
        for r in ("left_wrist", "right_wrist", "scene_fixed")
    }
    expected = {
        "camera/" + r + "/" + s: p for r, v in pipeline.items() for s, p in v["record"].items() if p
    }
    expected["feedback/arm"] = {}
    m = {
        "schema_version": "real-episode/2",
        "recording": cfg,
        "camera_pipeline": pipeline,
        "start_monotonic_ns": 10**9,
        "expected_streams": expected,
    }
    writer = EpisodeWriter(tmp_path, m)
    depth_raw = b"\x34\x12" * 16 * 12
    for i in range(31):
        stamp = 10**9 + round(i * 10**9 / 30)
        for role in pipeline:
            writer.frame(role, "rgb", meta(stamp), pixels())
        writer.frame("scene_fixed", "depth", meta(stamp, depth=True), depth_raw)
        writer.write_event(
            "feedback", {"side": "arm", "q": [0.0], "device_stamp_ns": stamp}, stamp, stamp
        )
    for kind in cfg["required_events"]:
        writer.write_event(kind, {}, 1_500_000_000, 1_500_000_000)
    assert writer.finish(
        {"end_monotonic_ns": 2 * 10**9, "dropped": 0, "interrupted": None, "writer_error": None}
    )["passed"]
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert len(manifest["media"]) == 3
    assert all(name.endswith("/rgb") for name in manifest["media"])
    assert quality_report(tmp_path, manifest, deep=True)["passed"]
    depths = []
    with (tmp_path / "streams.mcap").open("rb") as source:
        for _, channel, message in NonSeekingReader(source).iter_messages(log_time_order=False):
            if channel.message_encoding == "robot-infra-raw-image/1":
                assert channel.topic == "/camera/scene_fixed/depth"
                depths.append(decode_image(message.data))
    assert len(depths) == 31 and all(
        raw == depth_raw and m["encoding"] == "16UC1" for m, raw in depths
    )
