"""Run the audited dual-Piper arm-only live slice in an explicit ROS domain.

Source the installed ROS environments first. All processes are children of
this invocation; Ctrl+C stops only these children. Never auto-enable on launch.
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

from robot_infra.adapters.realsense_ros import driver_arguments, validate_config
from robot_infra.camera_config import bind_cameras
from robot_infra.live_teleop import create_model, validate_profile


def arm_arguments(executable, side, arm, *, gripper_enabled=False):
    channel = arm["can_interface"]
    if not re.fullmatch(r"[a-zA-Z0-9_-]{1,15}", channel):
        raise ValueError("explicit CAN interface required")
    remaps = {
        "__ns": "/" + side + "_arm",
        "__node": "robot_infra_" + side + "_driver",
        "joint_ctrl_single": arm["command_topic"],
        "enable_flag": arm["enable_topic"],
        "pos_cmd": "/robot_infra/" + side + "/unused_pose",
        "enable_srv": "/robot_infra/" + side + "/unused_enable_srv",
    }
    args = [executable, "--ros-args"]
    for key, value in remaps.items():
        args += ["-r", key + ":=" + value]
    for key, value in {
        "can_port": channel,
        "auto_enable": "false",
        "gripper_exist": "true" if gripper_enabled else "false",
    }.items():
        args += ["-p", key + ":=" + value]
    return args


def add_arguments(p):
    p.add_argument("--config", required=True)
    p.add_argument("--camera-config", required=True)
    p.add_argument(
        "--arm-driver", help="v1 legacy executable; v2 uses checked Python driver wrapper"
    )
    p.add_argument("--camera-driver", required=True)
    p.add_argument("--ros-domain-id", required=True, type=int)
    p.add_argument("--cert", required=True)
    p.add_argument("--key", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8445)
    p.add_argument("--duration", type=int, default=1800)
    p.add_argument("--advertise-host", help="device address shown to the headset operator")
    p.add_argument(
        "--check-only",
        action="store_true",
        help="validate environment/profile without constructing devices",
    )


def run(args):
    def invalid(message):
        raise ValueError(message)

    if not 0 <= args.ros_domain_id <= 232 or not 0 <= args.duration:
        invalid("invalid domain/duration")
    config = validate_profile(json.loads(Path(args.config).read_text(encoding="utf-8-sig")))
    cameras = json.loads(Path(args.camera_config).read_text(encoding="utf-8-sig"))
    validate_config(cameras)
    config = bind_cameras(config, cameras)
    request = getattr(args, "collection_request", None)
    if getattr(args, "force_free_teleop", False):
        config["recording_enabled"] = False
    if request:
        from .episode_store import task_id
        from .image_storage import check_image_storage, storage_config
        from .real_episode import validate_recording

        if getattr(args, "image_storage", None):
            selected = storage_config(config.get("recording", {}).get("storage"))
            selected["image_backend"] = args.image_storage
            config.setdefault("recording", {})["storage"] = selected
        validate_recording(config.get("recording", {}))
        task_id(request["task"])
        check_image_storage(config["recording"].get("storage"), config["camera_pipeline"])
        if request["start_episode"] < 0:
            invalid("start episode must be nonnegative")
        task_root = Path(request["root"]).expanduser().resolve() / task_id(request["task"])
        if (task_root / ".collector.lock").exists() and not request.get("recover_lock"):
            invalid("dataset task is locked; inspect owner or explicitly recover a dead lock")
        name = f"episode_{request['start_episode']:06d}"
        if request["existing"] == "error" and (
            (task_root / "episodes" / name).exists()
            or any((task_root / ".incomplete").glob(name + "_*"))
        ):
            invalid("requested episode number already exists")
        config["recording_enabled"] = True
        config["collection_request"] = request
    if config.get("recording_enabled") and not request:
        invalid("recording requires explicit collect task/path/numbering arguments")
    if config.get("webxr_transport", {}).get("input") == "websocket":
        from websockets.sync.server import serve  # noqa: F401 - dependency preflight before devices

        if config["webxr_transport"]["port"] == args.port:
            invalid("control and media ports must differ")
    elif config.get("webxr_transport", {}).get("input") == "webrtc":
        from aiortc import RTCPeerConnection  # noqa: F401 - fail before constructing any device
    model = create_model(config)
    if len(model.names) != 6:
        invalid("audited Piper dialect requires six arm joints")
    if set(config["camera_topics"]) != {item["role"] for item in cameras["cameras"]}:
        invalid("camera roles must match session topic roles")
    if config["schema_version"] == "live-teleop/2":
        for side, arm in config["arms"].items():
            if any(not lo <= q <= hi for q, (lo, hi) in zip(arm["home_rad"], model.limits)):
                invalid("home outside model joint limits: " + side)
            interface = Path("/sys/class/net") / arm["can_interface"]
            if not interface.is_dir() or not int((interface / "flags").read_text().strip(), 16) & 1:
                invalid("CAN interface missing or DOWN: " + arm["can_interface"])
    if len({arm["can_interface"] for arm in config["arms"].values()}) != len(config["arms"]):
        invalid("arms must use distinct CAN interfaces")
    for path in (args.camera_driver, args.cert, args.key):
        if not Path(path).is_file():
            invalid("missing configured file: " + path)
    if config["schema_version"] == "live-teleop/1" and (
        not args.arm_driver or not Path(args.arm_driver).is_file()
    ):
        invalid("v1 requires existing --arm-driver")
    if args.check_only:
        print(
            json.dumps(
                {
                    "preflight": "passed_without_device_construction",
                    "model_sha256": model.sha256,
                    "joint_names": model.names,
                    "cameras": [c["role"] for c in cameras["cameras"]],
                    "home": {s: a.get("home_rad") for s, a in config["arms"].items()},
                    "collection_request": request,
                    "camera_pipeline": config["camera_pipeline"],
                    "pose_feedback": config.get("pose_feedback", {}),
                    "image_storage": storage_config(config["recording"].get("storage"))[
                        "image_backend"
                    ]
                    if request
                    else None,
                }
            )
        )
        return
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    from .provenance import runtime_identity

    config["runtime_identity"] = runtime_identity()
    effective = root / "effective-config.json"
    effective.write_text(json.dumps(config, indent=2), encoding="utf-8")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    processes, logs, manifest = [], [], []

    def start(role, command):
        log = (root / (role + ".log")).open("x")
        logs.append(log)
        child = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True
        )
        processes.append(child)
        manifest.append({"role": role, "pid": child.pid, "args": command})
        (root / "processes.json").write_text(
            json.dumps({"ros_domain_id": args.ros_domain_id, "children": manifest}, indent=2),
            encoding="utf-8",
        )

    try:
        for side, arm in config["arms"].items():
            arguments = arm_arguments(
                args.arm_driver or "driver",
                side,
                arm,
                gripper_enabled=config.get("gripper_control", False),
            )
            arguments += ["--log-level", "warn"]
            if config["schema_version"] == "live-teleop/2":
                arguments = [
                    sys.executable,
                    "-m",
                    "robot_infra.adapters.piper_driver",
                    "--config",
                    str(effective),
                    *arguments[1:],
                ]
            start(side + "_driver", arguments)
        for camera in cameras["cameras"]:
            start(camera["role"], driver_arguments(args.camera_driver, camera))
        start(
            "gateway",
            [
                sys.executable,
                "-m",
                "robot_infra.live_teleop",
                "--config",
                str(effective),
                "--output",
                str(root / "episode"),
                "--cert",
                str(Path(args.cert).resolve()),
                "--key",
                str(Path(args.key).resolve()),
                "--host",
                args.host,
                "--port",
                str(args.port),
                "--duration",
                str(args.duration),
            ],
        )
        deadline = time.monotonic() + args.duration + 5 if args.duration else float("inf")
        print(
            json.dumps(
                {
                    "event": "stack_started",
                    "output": str(root),
                    "url": f"https://{args.advertise_host or args.host}:{args.port}/",
                    "hardware_auto_enable": False,
                    "workflow": "device self-check then A/home/A/teleop",
                }
            ),
            flush=True,
        )
        while time.monotonic() < deadline:
            if processes[-1].poll() == 0:
                break
            if any(child.poll() is not None for child in processes):
                raise RuntimeError("stack child exited; inspect project-owned logs")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Gateway first, to revoke ownership / attempt measured hold before ROS
        # drivers disappear. No global process-name killing or vendor changes.
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                timeout = (
                    config["recording"]["flush_timeout_s"] + 5
                    if request and child is processes[-1]
                    else 8
                )
                try:
                    child.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
        for log in logs:
            log.close()
        (root / "stack-ended.json").write_text(
            json.dumps(
                {
                    "exit_codes": [c.poll() for c in processes],
                    "stop_semantics": "gateway hold attempted; no automatic home, disable or gripper-close",
                }
            ),
            encoding="utf-8",
        )


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    add_arguments(p)
    return run(p.parse_args(argv))


if __name__ == "__main__":
    main()
