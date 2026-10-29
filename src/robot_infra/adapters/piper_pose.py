"""Piper SDK cache -> native observation. Does not command or connect hardware."""

import math


def native_end_pose(snapshot):
    axes = snapshot.end_pose
    position = [getattr(axes, k) for k in ("X_axis", "Y_axis", "Z_axis")]
    rotation = [getattr(axes, k) for k in ("RX_axis", "RY_axis", "RZ_axis")]
    if any(type(x) is not int for x in (*position, *rotation)):
        raise ValueError("SDK native pose must contain integer axes")
    stamp = snapshot.time_stamp
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0:
        raise ValueError("invalid SDK pose timestamp")
    seconds = int(stamp)
    return {
        "schema_version": "native-pose/1",
        "position": {"values": position, "axes": ["X", "Y", "Z"], "unit": "0.001mm"},
        "orientation": {
            "values": rotation,
            "axes": ["RX", "RY", "RZ"],
            "unit": "0.001degree",
            "representation": "euler",
            "convention": "vendor_RX_RY_RZ; unmodified, not a verified cross-robot rotation convention",
        },
        "sdk_time_stamp_s": stamp,
        "source_stamp_ns": seconds * 1_000_000_000 + int((stamp - seconds) * 1e9),
        "header_frame_id": "",
        "timestamp_conversion": "source_stamp_ns from SDK float seconds; original sdk_time_stamp_s preserved",
        "component_atomicity": "SDK cache assembled from multiple CAN frames; simultaneous acquisition unverified",
    }
