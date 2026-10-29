"""Camera fan-out configuration. No ROS imports or device construction."""

import math
from copy import deepcopy


def profile(value):
    if not isinstance(value, dict) or any(
        type(value.get(k)) is not int or not 1 <= value[k] <= (240 if k == "fps" else 4096)
        for k in ("width", "height", "fps")
    ):
        raise ValueError("camera profile requires positive width/height/fps")
    return {k: value[k] for k in ("width", "height", "fps")}


def pipeline(item):
    """Normalize legacy RGB samples or explicit capture/preview/record branches."""
    if "capture" not in item:
        capture = {"rgb": profile(item), "depth": None}
        preview = {**capture["rgb"], "fps": min(8, item["fps"]), "jpeg_quality": 75}
        record = {"rgb": capture["rgb"].copy(), "depth": None}
    else:
        capture = {
            s: profile(item["capture"][s]) if item["capture"].get(s) is not None else None
            for s in ("rgb", "depth")
        }
        if capture["rgb"] is None:
            raise ValueError("RGB capture is required for this VR adapter")
        preview = {
            **profile(item["preview"]),
            "jpeg_quality": item["preview"].get("jpeg_quality", 75),
        }
        if type(preview["jpeg_quality"]) is not int or not 1 <= preview["jpeg_quality"] <= 95:
            raise ValueError("invalid JPEG quality")
        record = {
            s: profile(item["record"][s]) if item["record"].get(s) is not None else None
            for s in ("rgb", "depth")
        }
        for branch in (preview, record["rgb"]):
            if branch and any(branch[k] > capture["rgb"][k] for k in ("width", "height", "fps")):
                raise ValueError("RGB consumers cannot exceed capture profile")
        if record["depth"]:
            if capture["depth"] is None or record["depth"] != capture["depth"]:
                raise ValueError("depth recording preserves the native depth capture profile")
    parameters = item.get("driver_parameters", {})
    if not isinstance(parameters, dict):
        raise ValueError("driver parameters must be an object")
    reserved = {
        "serial_no",
        "camera_name",
        "camera_namespace",
        "rgb_camera.color_profile",
        "depth_module.depth_profile",
        "enable_color",
        "enable_depth",
        "enable_infra1",
        "enable_infra2",
        "enable_gyro",
        "enable_accel",
    }
    for key, value in parameters.items():
        if (
            key in reserved
            or not isinstance(key, str)
            or not key
            or type(value) not in (bool, int, float, str)
            or (type(value) is float and not math.isfinite(value))
        ):
            raise ValueError("invalid or reserved driver parameter: " + str(key))
    return {
        "role": item["role"],
        "capture": capture,
        "preview": preview,
        "record": record,
        "driver_parameters": deepcopy(parameters),
    }


def bind_cameras(config, cameras):
    """Derive runtime topics once from the camera role, preserving custom RGB topics."""
    c = deepcopy(config)
    roles = {x["role"] for x in cameras["cameras"]}
    if set(c["camera_topics"]) != roles:
        raise ValueError("camera roles must match session topic roles")
    c["camera_pipeline"] = {x["role"]: pipeline(x) for x in cameras["cameras"]}
    c["camera_profiles"] = {r: p["capture"]["rgb"] for r, p in c["camera_pipeline"].items()}
    c["camera_configuration"] = deepcopy(cameras)
    return c
