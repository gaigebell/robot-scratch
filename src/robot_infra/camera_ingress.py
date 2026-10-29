"""Shared camera fan-out for live service and camera-only isolation probes."""

import threading
import time
from collections import Counter, deque

from .real_episode import RateGate


def camera_transport_config(value=None):
    defaults = {"image_reliability": "best_effort", "image_queue_depth": 5}
    if value is None:
        return defaults
    if not isinstance(value, dict) or set(value) - set(defaults):
        raise ValueError("invalid camera_transport config")
    c = {**defaults, **value}
    if c["image_reliability"] not in ("best_effort", "reliable"):
        raise ValueError("invalid image reliability")
    if type(c["image_queue_depth"]) is not int or not 1 <= c["image_queue_depth"] <= 128:
        raise ValueError("invalid image queue depth")
    return c


class CameraIngress:
    """Fan out observations to independent recording and latest-frame consumers.

    Preview rate gating must never become a capture filter. Source timestamps
    and local receipt times remain separate even when the camera clock rewinds.
    """

    def __init__(self, pipeline, metrics, *, recorder=None, preview=None):
        self.pipeline, self.metrics, self.recorder, self.preview = (
            pipeline,
            metrics,
            recorder,
            preview,
        )
        self.lock = threading.Lock()
        self.stream_last = {}
        self.observed_profiles = {}
        self.capture_sequences = Counter()
        self.camera_ticks = {}
        self.camera_info = {}
        self.source_last = {}
        self.source_rewinds = Counter()
        self.gates = {r: RateGate(p["preview"]["fps"]) for r, p in pipeline.items()}

    def image(self, m, role, stream):
        mono, wall = time.monotonic_ns(), time.time_ns()
        key = (role, stream)
        name = role + "." + stream
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        try:
            with self.lock:
                old = self.stream_last.get(key)
                source_old = self.source_last.get(key)
                self.stream_last[key] = mono / 1e9
                self.source_last[key] = stamp
                self.capture_sequences[key] += 1
                seq = self.capture_sequences[key]
                self.observed_profiles[key] = {
                    "width": m.width,
                    "height": m.height,
                    "encoding": m.encoding,
                    "step": m.step,
                }
                if stream == "rgb":
                    self.camera_ticks.setdefault(role, deque(maxlen=90)).append(mono / 1e9)
                if source_old is not None and stamp < source_old:
                    self.source_rewinds[name] += 1
            if old is not None:
                self.metrics.observe("camera.receive_gap." + name, (mono / 1e9 - old) * 1000)
            if source_old is not None and stamp >= source_old:
                self.metrics.observe("camera.source_gap." + name, (stamp - source_old) / 1e6)
            p = self.pipeline[role]
            preview = bool(
                stream == "rgb"
                and self.preview
                and self.preview.wanted(role)
                and self.gates[role].accept(mono)
            )
            recording = bool(
                self.recorder and self.recorder.state == "RECORDING" and p["record"].get(stream)
            )
            if not preview and not recording:
                return
            meta = {
                "role": role,
                "stream": stream,
                "stamp_ns": stamp,
                "capture_sequence": seq,
                "frame_id": m.header.frame_id,
                "width": m.width,
                "height": m.height,
                "step": m.step,
                "encoding": m.encoding,
                "is_bigendian": m.is_bigendian,
                "host_monotonic_ns": mono,
                "host_unix_ns": wall,
                "source_clock_domain": "ros_header; sensor metadata recorded separately",
                "depth_unit": "native_values; consult device metadata/calibration"
                if stream == "depth"
                else None,
            }
            raw = bytes(m.data)
            if recording:
                self.recorder.frame(role, stream, meta, raw)
            if preview:
                self.preview.offer(role, meta, raw, p["preview"])
        finally:
            self.metrics.elapsed("camera.callback." + name, mono)


def subscribe_cameras(node, config, ingress, emit, errors):
    """Import ROS only at the device boundary; never create robot publishers."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    transport = camera_transport_config(config.get("camera_transport"))
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    image_qos = QoSProfile(
        depth=transport["image_queue_depth"],
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE
        if transport["image_reliability"] == "reliable"
        else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    for role, p in ingress.pipeline.items():
        for stream in ("rgb", "depth"):
            if not p["capture"].get(stream):
                continue
            topic = (
                config["camera_topics"][role]
                if stream == "rgb"
                else "/" + role + "/depth/image_rect_raw"
            )
            node.create_subscription(
                Image, topic, lambda m, r=role, s=stream: ingress.image(m, r, s), image_qos
            )

            def info(m, r=role, s=stream):
                data = {
                    "role": r,
                    "stream": s,
                    "stamp_ns": m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                    "frame_id": m.header.frame_id,
                    "width": m.width,
                    "height": m.height,
                    "k": list(m.k),
                    "d": list(m.d),
                    "r": list(m.r),
                    "p": list(m.p),
                    "distortion_model": m.distortion_model,
                }
                key = r + "." + s
                with ingress.lock:
                    first = key not in ingress.camera_info
                    ingress.camera_info[key] = data
                if first:
                    emit("camera_info", data)
                elif ingress.recorder:
                    ingress.recorder.event("camera_info", data)

            node.create_subscription(
                CameraInfo, topic.rsplit("/", 1)[0] + "/camera_info", info, qos_profile_sensor_data
            )
            try:
                from realsense2_camera_msgs.msg import Metadata

                def metadata(m, r=role, s=stream):
                    if ingress.recorder:
                        ingress.recorder.event(
                            "device_metadata",
                            {
                                "role": r,
                                "stream": s,
                                "stamp_ns": m.header.stamp.sec * 1_000_000_000
                                + m.header.stamp.nanosec,
                                "frame_id": m.header.frame_id,
                                "raw_json": m.json_data,
                            },
                        )

                node.create_subscription(
                    Metadata,
                    "/" + role + ("/color/metadata" if stream == "rgb" else "/depth/metadata"),
                    metadata,
                    qos_profile_sensor_data,
                )
            except ImportError:
                errors.append("RealSense metadata message unavailable: " + role + "." + stream)
