import json
import struct
import zlib

import pytest

from robot_infra.adapters.realsense_ros import driver_arguments, validate_config
from robot_infra.observations import ImageSampleRecorder, rgb_png


def test_rgb_png_removes_padding_and_converts_bgr():
    png = rgb_png(bytes([30, 20, 10, 0]), 1, 1, 4, "bgr8")
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    offset, compressed = 8, b""
    while offset < len(png):
        length = struct.unpack(">I", png[offset : offset + 4])[0]
        kind = png[offset + 4 : offset + 8]
        if kind == b"IDAT":
            compressed += png[offset + 8 : offset + 8 + length]
        offset += 12 + length
    assert zlib.decompress(compressed) == bytes([0, 10, 20, 30])
    with pytest.raises(ValueError):
        rgb_png(b"", 1, 1, 3, "rgb8")


def test_sample_recording_is_bounded_and_preserves_raw_timestamps(tmp_path):
    root = tmp_path / "sample"
    sink = ImageSampleRecorder(root, ["wrist"], max_frames=1)
    raw = bytes([10, 20, 30])
    for _ in range(3):
        sink.frame("wrist", raw, 1, 1, 3, "rgb8", {"stamp_ns": 123, "frame_id": "optical"})
    sink.camera_info("wrist", {"k": [1, 0, 0]})
    sink.metadata("wrist", {"stamp_ns": 123}, '{"frame_number":1}')
    result = sink.close()
    assert result["roles"]["wrist"]["frames_seen"] == 3
    assert result["roles"]["wrist"]["frames_saved"] == 1
    assert (root / "wrist/0000.raw").read_bytes() == raw
    records = [json.loads(x) for x in (root / "observations.jsonl").read_text().splitlines()]
    assert records[0]["value"]["header"]["stamp_ns"] == 123
    assert records[-1]["value"]["raw_json"] == '{"frame_number":1}'
    with pytest.raises(FileExistsError):
        ImageSampleRecorder(root, ["wrist"])


def test_camera_roles_and_serials_configure_driver_without_robot_topics():
    item = {
        "role": "custom_camera",
        "serial": "00001234",
        "serial_namespace": "realsense_sdk",
        "width": 640,
        "height": 480,
        "fps": 15,
    }
    config = {"schema_version": "realsense-rgb-sample/1", "cameras": [item]}
    validate_config(config)
    args = driver_arguments("/device/driver", item)
    assert "serial_no:='00001234'" in args
    assert "enable_depth:=false" in args
    assert all("cmd_vel" not in x and "enable_flag" not in x for x in args)
    with pytest.raises(ValueError, match="duplicate"):
        validate_config({**config, "cameras": [item, item]})
    with pytest.raises(ValueError, match="realsense_sdk"):
        validate_config({**config, "cameras": [{**item, "serial_namespace": "usb_descriptor"}]})


@pytest.mark.parametrize("role", ["../escape", "", "/camera", "a/b"])
def test_sample_role_cannot_escape_recording_root(tmp_path, role):
    with pytest.raises(ValueError):
        ImageSampleRecorder(tmp_path / "out", [role])
