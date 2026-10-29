import json
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from test_real_episode import config, context, image, wait_for

from robot_infra.adapters.piper_pose import native_end_pose
from robot_infra.adapters.ros_pose import subscribe_pose_feedback
from robot_infra.episode_qc import quality_report
from robot_infra.pose_feedback import native_pose_sample, pose_sample, validate_pose_sources
from robot_infra.real_episode import EpisodeRecorder, EpisodeWriter


def sources():
    return {
        "tool_custom": {
            "topic": "/generic_sensor/pose",
            "format": "pose_stamped",
            "orientation_provenance": "converted",
            "conversion": "upstream Euler xyz to xyzw",
            "provenance": "vendor_ros; quaternion converted upstream",
            "source_clock_domain": "upstream_header_unverified",
            "reference_frame": None,
            "tracked_frame": None,
            "frame_status": "unverified",
        }
    }


def sample(quaternion=(0.0, 0.0, 0.0, 1.0)):
    return pose_sample(
        "tool_custom", sources()["tool_custom"], [0.25, -0.1, 0.3], quaternion, 1_000_000_100, ""
    )


def message(quaternion=(0.0, 0.0, 0.0, 1.0)):
    return NS(
        header=NS(stamp=NS(sec=1, nanosec=100), frame_id=""),
        pose=NS(
            position=NS(x=0.25, y=-0.1, z=0.3), orientation=NS(**dict(zip("xyzw", quaternion)))
        ),
    )


def test_subscriber_preserves_source_and_never_writes_or_normalizes():
    class ReadOnlyNode:
        def create_subscription(self, typ, topic, callback, qos):
            self.callback = callback
            assert typ is object and topic == "/generic_sensor/pose" and qos == 1
            return "subscription"

    node = ReadOnlyNode()
    events = []
    assert subscribe_pose_feedback(
        node, sources(), lambda *args: events.append(args), message_type=object
    ) == ["subscription"]
    node.callback(message((0.1, 0.2, 0.3, 0.9)))
    kind, payload = events[-1]
    assert kind == "pose_feedback" and payload["orientation_xyzw"] == [0.1, 0.2, 0.3, 0.9]
    assert (
        payload["position_m"] == [0.25, -0.1, 0.3] and payload["source_stamp_ns"] == 1_000_000_100
    )
    assert (
        payload["header_frame_id"] == ""
        and payload["reference_frame"] is None
        and payload["tracked_frame"] is None
    )
    assert not payload["quaternion_unit"]
    node.callback(message((float("nan"), 0, 0, 1)))
    assert events[-1][0] == "observation_error"
    json.dumps(events, allow_nan=False)


def test_pose_sources_are_optional_and_explicit():
    assert validate_pose_sources() == {}
    assert subscribe_pose_feedback(object(), None, lambda *args: None) == []
    c = sources()
    checked = validate_pose_sources(c)
    checked["tool_custom"]["topic"] = "/new"
    assert c != checked
    for changed in ({"topic": "relative"}, {"reference_frame": ""}, {"unknown": True}):
        bad = deepcopy(c)
        bad["tool_custom"].update(changed)
        with pytest.raises(ValueError):
            validate_pose_sources(bad)
    bad = deepcopy(c)
    bad["another"] = bad["tool_custom"].copy()
    with pytest.raises(ValueError):
        validate_pose_sources(bad)
    with pytest.raises(ValueError):
        validate_pose_sources({"../escape": c["tool_custom"]})
    with pytest.raises(ValueError):
        pose_sample("id", c["tool_custom"], [0, 0, 0], [0, 0, 0, float("inf")], 1, "frame")


def native():
    return native_end_pose(
        NS(
            time_stamp=1.25,
            end_pose=NS(
                X_axis=123456,
                Y_axis=-789,
                Z_axis=200000,
                RX_axis=90000,
                RY_axis=-45000,
                RZ_axis=123,
            ),
        )
    )


def native_definition():
    return {
        **sources()["tool_custom"],
        "format": "native_pose_json",
        "orientation_provenance": "native",
        "conversion": None,
    }


def test_native_euler_stays_integer_and_units_are_not_converted():
    value = native()
    definition = native_definition()
    assert validate_pose_sources({"any_tool": definition})
    result = native_pose_sample("any_tool", definition, value)
    assert result["native"]["position"]["values"] == [123456, -789, 200000]
    assert result["native"]["orientation"]["values"] == [90000, -45000, 123]
    assert (
        result["native"]["position"]["unit"] == "0.001mm"
        and result["native"]["orientation"]["unit"] == "0.001degree"
    )
    assert result["orientation_provenance"] == "native" and result["conversion"] is None
    assert "orientation_xyzw" not in result and "position_m" not in result
    assert (
        result["source_stamp_ns"] == 1_250_000_000 and result["native"]["sdk_time_stamp_s"] == 1.25
    )

    class Node:
        def create_subscription(self, typ, topic, callback, qos):
            self.callback = callback

    node = Node()
    events = []
    subscribe_pose_feedback(
        node, {"any_tool": definition}, lambda *v: events.append(v), message_type=object
    )
    node.callback(NS(data=json.dumps(value)))
    assert events == [("pose_feedback", result)]
    node.callback(NS(data="{}"))
    assert events[-1][0] == "observation_error"
    for change in (
        {"orientation_provenance": "converted", "conversion": None},
        {"orientation_provenance": "native", "conversion": "Euler to quat"},
    ):
        with pytest.raises(ValueError):
            validate_pose_sources({"any_tool": {**definition, **change}})


def test_driver_wrapper_adds_only_cache_observation_and_keeps_existing_publisher():
    from robot_infra.adapters.piper_driver import driver_class

    class Vendor:
        class PiperRosNode:
            def PublishArmEndPose(self):
                self.parent_called = True

    cls = driver_class(Vendor, None)
    node = cls.__new__(cls)
    cache = NS(
        time_stamp=1.25,
        end_pose=NS(
            **dict(
                zip(
                    ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"),
                    (123456, -789, 200000, 90000, -45000, 123),
                )
            )
        ),
    )
    node.piper = NS(GetArmEndPoseMsgs=lambda: cache)  # No enable/control/connect APIs.
    node._native_pose_type = NS
    captured = []
    node._native_pose_publisher = NS(publish=lambda m: captured.append(json.loads(m.data)))
    node.PublishArmEndPose()
    assert node.parent_called and captured == [native()]


@pytest.mark.parametrize("backend", ["raw_mcap", "mp4"])
@pytest.mark.parametrize("representation", ["quaternion", "native_euler"])
def test_pose_observation_roundtrip_index_alignment_and_QC(tmp_path, backend, representation):
    if backend == "mp4":
        pytest.importorskip("av")
    cfg = config()
    cfg["storage"] = {"image_backend": backend}
    ctx = context()
    ctx["pose_feedback"] = sources()
    first_sample = sample()
    if representation == "native_euler":
        ctx["pose_feedback"] = {"tool_custom": native_definition()}
        first_sample = native_pose_sample("tool_custom", native_definition(), native())
        first_sample["source_stamp_ns"] = 1_000_000_100
    expected = {
        "camera/" + r + "/rgb": p["record"]["rgb"] for r, p in ctx["camera_pipeline"].items()
    }
    expected.update({"feedback/custom_arm": {}, "pose_feedback/tool_custom": {}})
    manifest = {
        "schema_version": "real-episode/2" if backend == "mp4" else "real-episode/1",
        "task": "task",
        "episode": 0,
        "start_monotonic_ns": 1_000_000_000,
        "recording": cfg,
        "camera_pipeline": ctx["camera_pipeline"],
        "expected_streams": expected,
        "context": ctx,
    }
    writer = EpisodeWriter(tmp_path, manifest)
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
        value = {**first_sample, "source_stamp_ns": stamp + 100}
        if representation == "native_euler":
            value = deepcopy(value)
            value["native"]["source_stamp_ns"] = stamp + 100
        writer.write_event("pose_feedback", value, stamp, stamp + 100)
    for kind in cfg["required_events"]:
        writer.write_event(kind, {"test": True}, 1_500_000_000, 1_500_000_100)
    qc = writer.finish(
        {"end_monotonic_ns": 2_000_000_000, "interrupted": None, "dropped": 0, "writer_error": None}
    )
    assert qc["passed"], qc
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert quality_report(tmp_path, manifest, deep=True)["passed"]
    assert qc["streams"]["pose_feedback/tool_custom"]["unique_source_stamps"] == 31
    row = json.loads((tmp_path / "alignment.jsonl").read_text().splitlines()[0])
    assert row["matches"]["pose_feedback/tool_custom"]["delta_ns"] == 0
    from mcap.reader import NonSeekingReader

    with (tmp_path / "streams.mcap").open("rb") as source:
        poses = [
            json.loads(m.data)
            for _, c, m in NonSeekingReader(source).iter_messages(log_time_order=False)
            if c.topic == "/pose_feedback/tool_custom"
        ]
    if representation == "native_euler":
        first_sample["native"]["source_stamp_ns"] = 1_000_000_100
    assert len(poses) == 31 and poses[0]["payload"] == first_sample


@pytest.mark.parametrize(
    "kind", ["missing", "non_unit", "observation_error", "frozen", "zero_stamp"]
)
def test_QC_reports_pose_failures(tmp_path, kind):
    cfg = config()
    cfg["required_events"] = []
    cfg["qc"]["min_duration_s"] = 0.001
    ctx = context()
    ctx["pose_feedback"] = sources()
    recorder = EpisodeRecorder(cfg, tmp_path, "task", 0, "skip")
    try:
        recorder.start(ctx)
        wait_for(recorder, "RECORDING")
        if kind == "non_unit":
            recorder.event("pose_feedback", sample((0, 0, 0, 0)))
        elif kind == "observation_error":
            recorder.event(
                "observation_error", {"source_id": "tool_custom", "error": "invalid sample"}
            )
        elif kind == "frozen":
            recorder.event("pose_feedback", sample())
            recorder.event("pose_feedback", sample())
        elif kind == "zero_stamp":
            recorder.event("pose_feedback", {**sample(), "source_stamp_ns": 0})
        recorder.finish()
        snapshot = wait_for(recorder, "REVIEW")
        assert not snapshot["qc"]["passed"]
        issues = snapshot["qc"]["issues"]
        expected = {
            "missing": "pose_feedback/tool_custom: missing",
            "non_unit": "non-unit quaternion",
            "observation_error": "observation error",
            "frozen": "frozen source stamp",
            "zero_stamp": "zero source stamp",
        }[kind]
        assert any(expected in x for x in issues)
        manifest = json.loads((recorder.path / "manifest.json").read_text())
        assert "pose_feedback/tool_custom" in manifest["expected_streams"]
    finally:
        recorder.close()


def test_profile_rejects_ambiguous_conversion_and_native_command_topic_collision():
    from pathlib import Path

    from robot_infra.live_teleop import validate_profile

    profile = json.loads(
        (Path(__file__).parents[1] / "configs/teleop-session.example.json").read_text()
    )
    assert validate_profile(profile)
    bad = deepcopy(profile)
    bad["pose_feedback"]["left_quaternion"]["conversion"] = None
    with pytest.raises(ValueError, match="conversion"):
        validate_profile(bad)
    bad = deepcopy(profile)
    bad["arms"]["left"]["native_pose_topic"] = bad["arms"]["left"]["command_topic"]
    with pytest.raises(ValueError, match="observation topics"):
        validate_profile(bad)


def test_real_ROS_message_serialization_without_devices(monkeypatch):
    import time

    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import PoseStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    ctx = Context()
    rclpy.init(context=ctx, domain_id=191)
    node = rclpy.create_node("robot_infra_pose_loopback_test", context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    events = []
    try:
        definitions = sources()
        definitions["native"] = {**native_definition(), "topic": "/generic_sensor/native"}
        subscribe_pose_feedback(node, definitions, lambda *v: events.append(v))
        publisher = node.create_publisher(PoseStamped, "/generic_sensor/pose", 1)
        native_publisher = node.create_publisher(String, "/generic_sensor/native", 1)
        msg = PoseStamped()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 100
        msg.pose.position.x = 0.25
        msg.pose.position.y = -0.1
        msg.pose.position.z = 0.3
        msg.pose.orientation.w = 1.0
        raw = String()
        raw.data = json.dumps(native())
        deadline = time.monotonic() + 5
        while (
            time.monotonic() < deadline
            and len({p["source_id"] for k, p in events if k == "pose_feedback"}) < 2
        ):
            publisher.publish(msg)
            native_publisher.publish(raw)
            executor.spin_once(timeout_sec=0.05)
        received = {p["source_id"]: p for k, p in events if k == "pose_feedback"}
        assert set(received) == {"tool_custom", "native"}
        assert received["tool_custom"]["orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
        assert received["native"]["native"] == native()
        assert (
            received["native"]["orientation_provenance"] == "native"
            and received["tool_custom"]["orientation_provenance"] == "converted"
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)
