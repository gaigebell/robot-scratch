import importlib.util
import json
from pathlib import Path

from mcap.writer import CompressionType, Writer


def test_metadata_can_exist_for_image_missing_before_writer(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "delivery_audit", Path(__file__).parents[1] / "tools/audit_camera_delivery.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "camera_pipeline": {
                    "test": {"capture": {"rgb": {"fps": 30}}, "record": {"rgb": {"fps": 30}}}
                }
            }
        )
    )
    images = [
        {"stream": "camera/test/rgb", "received_ns": stamp, "source_stamp_ns": stamp}
        for stamp in (1_000_000_000, 1_200_000_000)
    ]
    (tmp_path / "index.jsonl").write_text("".join(json.dumps(r) + "\n" for r in images))
    with (tmp_path / "streams.mcap").open("wb") as f:
        writer = Writer(f, compression=CompressionType.NONE)
        writer.start()
        channel = writer.register_channel("/event/device_metadata", "json", 0)
        for n, stamp in enumerate((1_000_000_000, 1_033_333_333, 1_200_000_000)):
            event = {
                "host_monotonic_ns": stamp + 1000,
                "payload": {
                    "role": "test",
                    "stream": "rgb",
                    "stamp_ns": stamp,
                    "raw_json": json.dumps({"frame_counter": n, "clock_domain": "fixture"}),
                },
            }
            writer.add_message(channel, stamp, json.dumps(event).encode(), publish_time=stamp)
        writer.finish()
    result = module.audit(tmp_path)
    gap = result["gaps"][0]
    assert gap["metadata_between_count"] == 1 and gap["same_capture_record_fps"] is True
    assert gap["edge_metadata"]["after"]["metadata"]["frame_counter"] == 2
    assert gap["interpretation"] == "metadata_observed_inside_recorded_image_gap"
    assert result["source_modified"] is False
