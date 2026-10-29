"""ROS-domain RealSense RGB observation adapter; no robot command topics."""

import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path

from robot_infra.camera_config import pipeline
from robot_infra.observations import ImageSampleRecorder


def validate_config(config):
    if config.get("schema_version") not in ("realsense-rgb-sample/1", "realsense-streams/2"):
        raise ValueError("unsupported camera configuration")
    cameras = config.get("cameras")
    if not isinstance(cameras, list) or not 1 <= len(cameras) <= 8:
        raise ValueError("1..8 cameras required")
    for item in cameras:
        if item.get("serial_namespace") != "realsense_sdk":
            raise ValueError("camera serial must be explicitly identified as realsense_sdk")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", item.get("role", "")) or not re.fullmatch(
            r"\d+", item.get("serial", "")
        ):
            raise ValueError("explicit camera role and serial required")
        if config["schema_version"] == "realsense-streams/2" and "capture" not in item:
            raise ValueError("v2 requires capture/preview/record branches")
        pipeline(item)
    if len({x["role"] for x in cameras}) != len(cameras) or len(
        {x["serial"] for x in cameras}
    ) != len(cameras):
        raise ValueError("duplicate role or camera serial")


def driver_arguments(executable, item):
    settings = pipeline(item)
    rgb = settings["capture"]["rgb"]
    depth = settings["capture"]["depth"]
    profile = f"{rgb['width']}x{rgb['height']}x{rgb['fps']}"
    params = {
        "serial_no": "'" + item["serial"] + "'",
        "camera_name": item["role"],
        "camera_namespace": "''",
        "rgb_camera.color_profile": profile,
        "enable_color": "true",
        "enable_depth": "true" if depth else "false",
        "enable_infra1": "false",
        "enable_infra2": "false",
        "enable_gyro": "false",
        "enable_accel": "false",
    }
    if depth:
        params["depth_module.depth_profile"] = f"{depth['width']}x{depth['height']}x{depth['fps']}"
    params.update(
        {
            k: str(v).lower() if type(v) is bool else (json.dumps(v) if type(v) is str else str(v))
            for k, v in settings["driver_parameters"].items()
        }
    )
    args = [executable, "--ros-args", "-r", "__node:=" + item["role"], "-r", "__ns:=/"]
    for key, value in params.items():
        args += ["-p", key + ":=" + value]
    return args


def header_dict(header):
    return {
        "stamp_ns": header.stamp.sec * 1_000_000_000 + header.stamp.nanosec,
        "frame_id": header.frame_id,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--driver", required=True)
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--max-frames", type=int, default=3)
    args = parser.parse_args(argv)
    config = json.loads(Path(args.config).read_text(encoding="utf-8-sig"))
    validate_config(config)
    if not 1 <= args.duration <= 120:
        parser.error("diagnostic duration must be 1..120 seconds")
    # Dependencies remain entirely in the ROS device domain.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from realsense2_camera_msgs.msg import Metadata
    from sensor_msgs.msg import CameraInfo, Image

    sink = ImageSampleRecorder(args.output, [c["role"] for c in config["cameras"]], args.max_frames)
    (sink.root / "camera-config.json").write_text(
        json.dumps(config, indent=2) + "\n", encoding="utf-8"
    )
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node("robot_infra_rgb_observer")
    try:
        for item in config["cameras"]:
            role = item["role"]

            def image(message, role=role):
                try:
                    sink.frame(
                        role,
                        bytes(message.data),
                        message.width,
                        message.height,
                        message.step,
                        message.encoding,
                        header_dict(message.header),
                    )
                except (ValueError, OSError) as exc:
                    if len(sink.state[role]["errors"]) < 10:
                        sink.state[role]["errors"].append(str(exc))

            def info(message, role=role):
                sink.camera_info(
                    role,
                    {
                        "header": header_dict(message.header),
                        "width": message.width,
                        "height": message.height,
                        "distortion_model": message.distortion_model,
                        "d": list(message.d),
                        "k": list(message.k),
                        "r": list(message.r),
                        "p": list(message.p),
                    },
                )

            node.create_subscription(
                Image, f"/{role}/color/image_raw", image, qos_profile_sensor_data
            )
            node.create_subscription(
                CameraInfo, f"/{role}/color/camera_info", info, qos_profile_sensor_data
            )
            node.create_subscription(
                Metadata,
                f"/{role}/color/metadata",
                lambda msg, role=role: sink.metadata(role, header_dict(msg.header), msg.json_data),
                qos_profile_sensor_data,
            )
            log = (sink.root / (role + "-driver.log")).open("x")
            logs.append(log)
            processes.append(
                subprocess.Popen(
                    driver_arguments(args.driver, item),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Only children created by this invocation; never pkill unrelated nodes.
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()
        result = sink.close()
    print(json.dumps(result))
    return 0 if result["all_have_frames"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
