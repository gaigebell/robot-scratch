"""Read-only pose observations; no robot, ROS, FK or storage dependency."""

import json
import math
import re
from copy import deepcopy


def validate_pose_sources(sources=None):
    sources = {} if sources is None else sources
    if not isinstance(sources, dict):
        raise ValueError("pose_feedback must be a source mapping")
    topics = set()
    fields = {
        "topic",
        "format",
        "provenance",
        "source_clock_domain",
        "reference_frame",
        "tracked_frame",
        "frame_status",
        "orientation_provenance",
        "conversion",
    }
    for name, definition in sources.items():
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", name):
            raise ValueError("invalid pose source ID")
        if not isinstance(definition, dict) or set(definition) != fields:
            raise ValueError("explicit pose source fields required: " + name)
        for key in ("topic", "provenance", "source_clock_domain", "frame_status"):
            if not isinstance(definition[key], str) or not definition[key].strip():
                raise ValueError("invalid pose " + key)
        topic = definition["topic"]
        if not topic.startswith("/") or topic in topics:
            raise ValueError("distinct absolute pose topics required")
        topics.add(topic)
        if definition["format"] not in ("pose_stamped", "native_pose_json"):
            raise ValueError("unsupported pose observation format")
        if definition["orientation_provenance"] not in ("native", "converted", "unknown"):
            raise ValueError("explicit orientation provenance required")
        conversion = definition["conversion"]
        if conversion is not None and (not isinstance(conversion, str) or not conversion.strip()):
            raise ValueError("pose conversion must be described or null")
        if definition["orientation_provenance"] == "converted" and conversion is None:
            raise ValueError("converted orientation requires conversion description")
        if definition["orientation_provenance"] == "native" and conversion is not None:
            raise ValueError("native orientation cannot claim a conversion")
        for key in ("reference_frame", "tracked_frame"):
            if definition[key] is not None and (
                not isinstance(definition[key], str) or not definition[key].strip()
            ):
                raise ValueError("pose frame must be explicit or null")
    return deepcopy(sources)


def pose_sample(
    source_id, definition, position_m, orientation_xyzw, source_stamp_ns, header_frame_id
):
    for values, size in ((position_m, 3), (orientation_xyzw, 4)):
        if len(values) != size or any(
            type(x) not in (int, float) or not math.isfinite(x) for x in values
        ):
            raise ValueError("nonfinite or invalid pose components")
    if type(source_stamp_ns) is not int or source_stamp_ns < 0:
        raise ValueError("invalid pose source timestamp")
    if not isinstance(header_frame_id, str):
        raise ValueError("invalid pose header frame")
    norm = math.hypot(*orientation_xyzw)
    if not math.isfinite(norm):
        raise ValueError("invalid quaternion norm")
    return {
        **pose_identity(source_id, definition),
        "representation": "quaternion_xyzw",
        "position_m": list(position_m),
        "orientation_xyzw": list(orientation_xyzw),
        "source_stamp_ns": source_stamp_ns,
        "header_frame_id": header_frame_id,
        "quaternion_norm": norm,
        "quaternion_unit": abs(norm - 1.0) <= 1e-3,
    }


def pose_identity(source_id, definition):
    return {
        "schema_version": "pose-feedback/1",
        "source_id": source_id,
        **{
            k: definition[k]
            for k in (
                "topic",
                "reference_frame",
                "tracked_frame",
                "frame_status",
                "provenance",
                "source_clock_domain",
                "orientation_provenance",
                "conversion",
            )
        },
    }


def native_pose_sample(source_id, definition, value):
    if not isinstance(value, dict) or value.get("schema_version") != "native-pose/1":
        raise ValueError("unsupported native pose payload")
    json.dumps(value, allow_nan=False)
    if (
        type(value.get("source_stamp_ns")) is not int
        or value["source_stamp_ns"] < 0
        or not isinstance(value.get("header_frame_id"), str)
    ):
        raise ValueError("invalid native pose header")
    for key in ("position", "orientation"):
        vector = value.get(key, {})
        if (
            not isinstance(vector, dict)
            or not isinstance(vector.get("values"), list)
            or len(vector["values"]) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in vector["values"])
        ):
            raise ValueError("invalid native pose vector")
        if (
            not isinstance(vector.get("axes"), list)
            or len(vector["axes"]) != 3
            or any(not isinstance(x, str) or not x for x in vector["axes"])
        ):
            raise ValueError("explicit native axes required")
        if not isinstance(vector.get("unit"), str) or not vector["unit"]:
            raise ValueError("explicit native unit required")
    orientation = value["orientation"]
    if (
        orientation.get("representation") != "euler"
        or not isinstance(orientation.get("convention"), str)
        or not orientation["convention"]
    ):
        raise ValueError("explicit native Euler convention required")
    # Preserve vendor values/units verbatim; no Euler conversion or normalization.
    return {
        **pose_identity(source_id, definition),
        "representation": "native_euler",
        "source_stamp_ns": value["source_stamp_ns"],
        "header_frame_id": value["header_frame_id"],
        "native": deepcopy(value),
    }
