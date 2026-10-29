import json
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from robot_infra.camera_benchmark import ProbeApp
from robot_infra.camera_ingress import CameraIngress, camera_transport_config
from robot_infra.diagnostics import EventJournal
from robot_infra.live_teleop import make_server
from robot_infra.performance import Performance, performance_config
from robot_infra.preview import PreviewWorker, preview_config
from robot_infra.real_episode import EpisodeRecorder


def wait(predicate):
    end = time.monotonic() + 3
    while not predicate():
        if time.monotonic() > end:
            raise TimeoutError("worker did not progress")
        time.sleep(0.005)


def pipeline():
    shape = {"width": 2, "height": 2, "fps": 30}
    return {
        "camera": {
            "capture": {"rgb": shape, "depth": None},
            "record": {"rgb": shape, "depth": None},
            "preview": {**shape, "jpeg_quality": 60},
        }
    }


def message(stamp=1):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0), frame_id="optical"),
        width=2,
        height=2,
        encoding="rgb8",
        step=6,
        is_bigendian=0,
        data=b"\x02" * 12,
    )


def test_performance_memory_bounds_quantiles_and_disabled():
    metrics = Performance({"window_samples": 4, "max_series": 1})
    for n in range(100):
        metrics.observe("callback", n)
    metrics.observe("other", 3)
    result = metrics.snapshot()
    s = result["series"]["callback"]
    assert (s["count"], s["window_count"], s["max_ms"], s["window_p95_ms"]) == (100, 4, 99, 99)
    assert s["window_p50_ms"] == 97 and result["rejected_series_samples"] == 1
    off = Performance({"enabled": False})
    off.observe("x", 1)
    assert off.snapshot()["series"] == {}
    for settings in ({"window_samples": 0}, {"max_series": True}, {"enabled": 1}, {"unknown": 1}):
        with pytest.raises(ValueError):
            performance_config(settings)


def test_slow_preview_cannot_block_raw_capture_and_no_demand_no_encoding(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    published = []
    frames = []

    def encode(*args):
        entered.set()
        release.wait(3)
        return b"jpeg"

    metrics = Performance()
    preview = PreviewWorker(
        ["camera"], encode, lambda *args: published.append(args), metrics, archive_root=tmp_path
    )
    recorder = SimpleNamespace(state="RECORDING", frame=lambda *args: frames.append(args))
    ingress = CameraIngress(pipeline(), metrics, recorder=recorder, preview=preview)
    try:
        ingress.image(message(), "camera", "rgb")
        assert len(frames) == 1 and not entered.is_set() and not (tmp_path / "preview").exists()
        preview.request("camera")
        ingress.gates["camera"].next = None
        ingress.image(message(2), "camera", "rgb")
        assert entered.wait(2)
        for n in range(3, 10):
            ingress.gates["camera"].next = None
            ingress.image(message(n), "camera", "rgb")
        assert len(frames) == 9 and preview.snapshot()["pending_roles"] == ["camera"]
        assert preview.snapshot()["counts"]["replaced"] >= 5
        release.set()
        wait(lambda: len(published) == 2)
        assert published[-1][1]["capture_sequence"] == 9
        assert frames[-1][2]["stamp_ns"] == 9_000_000_000 and frames[-1][3] == b"\x02" * 12
        assert not (tmp_path / "preview").exists()
    finally:
        release.set()
        preview.close()
    assert not preview.thread.is_alive()


def test_preview_archive_opt_in_error_is_local_and_lease_expires(tmp_path):
    metrics = Performance()
    events = []
    preview = PreviewWorker(
        ["camera"],
        lambda *a: b"jpeg",
        lambda *a: None,
        metrics,
        config={"archive_jpeg": True, "lease_s": 0.1},
        archive_root=tmp_path,
        emit=lambda *a: events.append(a),
    )
    ingress = CameraIngress(pipeline(), metrics, preview=preview)
    try:
        assert not preview.request("unknown")
        preview.request("camera")
        ingress.image(message(), "camera", "rgb")
        wait(lambda: preview.snapshot()["counts"].get("encoded") == 1)
        assert (tmp_path / "preview/camera/1.jpg").read_bytes() == b"jpeg"
        assert events[0][0] == "camera_preview" and "sha256" in events[0][1]
        time.sleep(0.12)
        assert not preview.wanted("camera")
    finally:
        preview.close()
    failed = PreviewWorker(
        ["camera"],
        lambda *a: (_ for _ in ()).throw(OSError("encoder failed")),
        lambda *a: None,
        metrics,
        config={"on_demand": False},
    )
    try:
        failed.offer("camera", {"host_monotonic_ns": time.monotonic_ns()}, b"", {})
        wait(lambda: failed.error is not None)
        assert failed.error == "encoder failed"
    finally:
        failed.close()
    for c in ({"archive_jpeg": 1}, {"lease_s": float("nan")}, {"unknown": True}):
        with pytest.raises(ValueError):
            preview_config(c)


def test_camera_probe_http_has_frame_identity_and_never_actuates(tmp_path):
    journal = EventJournal(tmp_path / "journal")
    metrics = Performance()
    c = {"schema_version": "live-teleop/2", "camera_pipeline": pipeline()}
    app = ProbeApp(c, journal, metrics)
    ingress = CameraIngress(pipeline(), metrics)
    app.ingress = ingress
    preview = PreviewWorker(
        ["camera"],
        lambda *a: b"jpeg",
        lambda r, m, j, s: app.images.update(
            {
                r: {
                    "jpeg": j,
                    "seq": s,
                    "received_s": m["host_monotonic_ns"] / 1e9,
                    "capture_sequence": m["capture_sequence"],
                }
            }
        ),
        metrics,
    )
    ingress.preview = preview
    app.preview = preview
    server = make_server(("127.0.0.1", 0), app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with pytest.raises(HTTPError) as exc:
            urlopen(base + "/camera/camera.jpg")
        assert exc.value.code == 503
        ingress.image(message(), "camera", "rgb")
        wait(lambda: "camera" in app.images)
        with urlopen(base + "/camera/camera.jpg") as r:
            assert r.read() == b"jpeg" and r.headers["X-Capture-Sequence"] == "1"
            assert float(r.headers["X-Frame-Age-Ms"]) >= 0
        with urlopen(base + "/api/performance") as r:
            assert json.load(r)["metrics"]["enabled"]
        assert (
            app.input({"session_id": "test", "transport": {"previous_rtt_ms": 100}})[
                "robot_commands"
            ]
            is False
        )
        app.pause("operator")
        assert app.status()["diagnostic_only"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)
        preview.close()
        journal.close()


def test_source_rewinds_are_distinct_from_host_gaps():
    metrics = Performance()
    ingress = CameraIngress(pipeline(), metrics)
    ingress.image(message(2), "camera", "rgb")
    ingress.image(message(1), "camera", "rgb")
    assert ingress.source_rewinds["camera.rgb"] == 1
    s = metrics.snapshot()["series"]
    assert "camera.receive_gap.camera.rgb" in s
    assert "camera.source_gap.camera.rgb" not in s
    assert camera_transport_config()["image_reliability"] == "best_effort"
    assert camera_transport_config({"image_reliability": "reliable"})["image_queue_depth"] == 5
    for settings in (
        {"image_reliability": "unknown"},
        {"image_queue_depth": 0},
        {"image_queue_depth": True},
    ):
        with pytest.raises(ValueError):
            camera_transport_config(settings)


def test_recorder_timing_preserves_received_frames_and_bytes(tmp_path):
    from pathlib import Path

    from mcap.reader import make_reader

    from robot_infra.image_codec import decode_image

    c = json.loads((Path(__file__).parents[1] / "configs/teleop-session.example.json").read_text())[
        "recording"
    ]
    c["alignment"]["reference_stream"] = "camera/camera/rgb"
    c["min_free_bytes"] = 1
    c["required_events"] = []
    metrics = Performance()
    recorder = EpisodeRecorder(c, tmp_path, "timing", 0, "error", metrics=metrics)
    ingress = CameraIngress(pipeline(), metrics, recorder=recorder)
    try:
        recorder.start({"camera_pipeline": pipeline(), "arms": []})
        wait(lambda: recorder.state == "RECORDING")
        for n in range(4):
            ingress.image(message(n + 1), "camera", "rgb")
        recorder.finish()
        wait(lambda: recorder.state == "REVIEW")
        snap = recorder.snapshot()
        assert snap["dropped"] == 0 and snap["error"] is None
        manifest = json.loads((Path(snap["pending_path"]) / "manifest.json").read_text())
        assert manifest["frame_capture_stats"]["camera/camera/rgb"]["written"] == 4
        frames = []
        with (Path(snap["pending_path"]) / "streams.mcap").open("rb") as f:
            for schema, channel, message_ in make_reader(f).iter_messages():
                if channel.topic == "/camera/camera/rgb":
                    frames.append(decode_image(message_.data))
        assert len(frames) == 4 and all(raw == b"\x02" * 12 for meta, raw in frames)
        stats = metrics.snapshot()["series"]
        assert stats["record.queue.camera.rgb"]["count"] == 4
        assert stats["record.write.camera.rgb"]["count"] == 4
    finally:
        recorder.close()
