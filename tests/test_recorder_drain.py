"""Stopping capture must drain accepted frames before video/MCAP finalization."""

import json
import threading
import time

import pytest
from test_real_episode import config, context, image, wait_for

from robot_infra.real_episode import EpisodeRecorder, EpisodeWriter


@pytest.mark.parametrize("backend", ["raw_mcap", "mp4"])
def test_stop_while_writer_is_blocked_drains_all_accepted_frames(tmp_path, backend):
    if backend == "mp4":
        pytest.importorskip("av")
    entered, release, finalized = threading.Event(), threading.Event(), threading.Event()

    class BlockedWriter(EpisodeWriter):
        def frame(self, *args):
            entered.set()
            assert release.wait(3), "test did not release writer"
            return super().frame(*args)

        def finish(self, updates):
            finalized.set()
            return super().finish(updates)

    settings = config()
    settings["storage"] = {"image_backend": backend}
    settings["required_events"] = []
    setup = context()
    setup["arms"] = []
    setup["camera_pipeline"].pop("custom_wrist")
    recorder = EpisodeRecorder(
        settings, tmp_path, "drain", 0, "error", writer_factory=BlockedWriter
    )
    try:
        recorder.start(setup)
        wait_for(recorder, "RECORDING")
        accepted = []
        for i in range(3):
            stamp = time.monotonic_ns()
            accepted.append(stamp)
            assert recorder.frame("custom_front", "rgb", image(stamp), bytes([i]) * 12)
            if i == 0:
                assert entered.wait(1)
        recorder.finish()
        snap = recorder.snapshot()
        assert snap["state"] == "FINALIZING" and snap["queue_items"] == 2
        assert not finalized.is_set()
        assert not recorder.frame("custom_front", "rgb", image(time.monotonic_ns()), bytes(12))
        with pytest.raises(RuntimeError, match="not ready"):
            recorder.decide(True)
        release.set()
        snap = wait_for(recorder, "REVIEW")
        assert finalized.is_set() and snap["queue_items"] == 0 and snap["dropped"] == 0
        path = recorder.path
        manifest = json.loads((path / "manifest.json").read_text())
        assert manifest["frame_capture_stats"]["camera/custom_front/rgb"] == {
            "received_by_writer": 3,
            "rate_filtered": 0,
            "written": 3,
        }
        rows = [json.loads(line) for line in (path / "index.jsonl").read_text().splitlines()]
        assert [row["received_ns"] for row in rows] == accepted
        if backend == "mp4":
            import av

            asset = manifest["media"]["camera/custom_front/rgb"]
            with av.open(str(path / asset["path"])) as video:
                assert len(list(video.decode(video=0))) == 3
        else:
            from mcap.reader import make_reader

            with (path / "streams.mcap").open("rb") as stream:
                assert (
                    len(
                        list(make_reader(stream).iter_messages(topics=["/camera/custom_front/rgb"]))
                    )
                    == 3
                )
    finally:
        release.set()
        recorder.close()
