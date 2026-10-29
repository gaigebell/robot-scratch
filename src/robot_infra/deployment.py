"""Portable device-local launch profile; every machine-specific path is explicit."""

import argparse
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path


def prepare_collection(
    deployment, output, *, trigger_pressed, teleop_start, image_storage, depth_roles, depth_profile
):
    """Write an independent profile set, preserving machine identities; no devices."""
    from copy import deepcopy

    from .adapters.realsense_ros import validate_config
    from .camera_config import bind_cameras
    from .episode_store import write_json
    from .image_storage import storage_config
    from .live_teleop import validate_profile

    device = load_deployment(deployment)
    session = json.loads(Path(device["config"]).read_text(encoding="utf-8-sig"))
    cameras = json.loads(Path(device["camera_config"]).read_text(encoding="utf-8-sig"))
    if session.get("schema_version") != "live-teleop/2":
        raise ValueError("configurable grippers require live-teleop/2")
    if trigger_pressed not in ("open", "close") or teleop_start not in (
        "preserve",
        "follow_trigger",
    ):
        raise ValueError("explicit gripper behavior required")
    if image_storage not in ("raw_mcap", "mp4"):
        raise ValueError("unsupported image storage")
    if len(set(depth_roles)) != len(depth_roles) or set(depth_roles) - {
        c["role"] for c in cameras["cameras"]
    }:
        raise ValueError("unknown or duplicate depth role")
    if len(depth_profile) != 3 or any(type(x) is not int or x < 1 for x in depth_profile):
        raise ValueError("invalid depth profile")
    depth = dict(zip(("width", "height", "fps"), depth_profile))
    for arm in session["arms"].values():
        arm["gripper"].update(trigger_pressed=trigger_pressed, teleop_start=teleop_start)
    selected = storage_config(session["recording"].get("storage"))
    selected["image_backend"] = image_storage
    session["recording"]["storage"] = selected
    for item in cameras["cameras"]:
        from .camera_config import pipeline

        if "capture" not in item:
            normalized = pipeline(item)
            item.update({k: normalized[k] for k in ("capture", "preview", "record")})
        for branch in ("capture", "record"):
            item[branch]["depth"] = deepcopy(depth) if item["role"] in depth_roles else None
    cameras["schema_version"] = "realsense-streams/2"
    validate_profile(session)
    validate_config(cameras)
    bound = bind_cameras(session, cameras)
    root = Path(output).expanduser().resolve()
    # Never write into the currently selected configuration directory itself.
    root.mkdir(parents=True, exist_ok=False)
    device["config"] = str(root / "session.json")
    device["camera_config"] = str(root / "cameras.json")
    write_json(root / "session.json", session)
    write_json(root / "cameras.json", cameras)
    write_json(root / "deployment.json", device)
    return {
        "deployment": str(root / "deployment.json"),
        "image_storage": image_storage,
        "grippers": {s: a["gripper"] for s, a in session["arms"].items()},
        "camera_pipeline": bound["camera_pipeline"],
        "physical_commands": False,
        "device_construction": False,
    }


def load_deployment(path):
    path = Path(path).expanduser().resolve()
    c = json.loads(path.read_text(encoding="utf-8-sig"))
    if c.get("schema_version") != "robot-deployment/1":
        raise ValueError("unsupported deployment config")
    for key in ("config", "camera_config", "camera_driver", "cert", "key", "output_root"):
        if not isinstance(c.get(key), str) or not c[key]:
            raise ValueError("deployment path missing: " + key)
        p = Path(c[key]).expanduser()
        c[key] = str(p if p.is_absolute() else path.parent / p)
    for key in ("setup_scripts", "python_paths"):
        if not isinstance(c.get(key), list) or any(not isinstance(x, str) or not x for x in c[key]):
            raise ValueError("deployment list missing: " + key)
        c[key] = [
            str(Path(x).expanduser() if Path(x).expanduser().is_absolute() else path.parent / x)
            for x in c[key]
        ]
    if type(c.get("ros_domain_id")) is not int or not 0 <= c["ros_domain_id"] <= 232:
        raise ValueError("explicit ROS domain required")
    return c


def add_arguments(parser, recording):
    parser.add_argument("--deployment", required=True)
    parser.add_argument("--duration", type=int, help="seconds, 0 means until Ctrl+C")
    parser.add_argument("--advertise-host")
    parser.add_argument("--check-only", action="store_true")
    if recording:
        parser.add_argument(
            "--image-storage",
            choices=["raw_mcap", "mp4"],
            help="override configured image storage backend",
        )
        parser.add_argument("--task", required=True)
        parser.add_argument("--dataset-root", required=True)
        parser.add_argument("--start-episode", type=int, required=True)
        parser.add_argument("--existing", choices=["error", "skip", "overwrite"], required=True)
        parser.add_argument(
            "--recover-lock",
            action="store_true",
            help="recover only a dead collector lock on this host",
        )


def launch(args):
    c = load_deployment(args.deployment)
    output = Path(c["output_root"]) / (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    )
    values = {
        k: c.get(k)
        for k in ("config", "camera_config", "camera_driver", "ros_domain_id", "cert", "key")
    }
    values.update(
        arm_driver=c.get("arm_driver"),
        host=c.get("host", "127.0.0.1"),
        port=c.get("port", 8445),
        duration=c.get("duration", 0) if args.duration is None else args.duration,
        advertise_host=args.advertise_host or c.get("advertise_host"),
        check_only=args.check_only,
        output=str(output),
    )
    values["collection_request"] = None
    values["image_storage"] = getattr(args, "image_storage", None)
    if args.command == "collect":
        values["collection_request"] = {
            "root": args.dataset_root,
            "task": args.task,
            "start_episode": args.start_episode,
            "existing": args.existing,
            "recover_lock": args.recover_lock,
        }
    values["force_free_teleop"] = args.command == "device-teleop"
    from .session_launcher import run

    return run(argparse.Namespace(**values))
