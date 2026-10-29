"""Checked wrapper around an installed Piper ROS driver, not a forked SDK.

EnableArm is kept separate from GripperCtrl. No automatic home, disable,
zeroing or gripper-close is performed. Only the gateway supplies targets.
"""

import argparse
import hashlib
import importlib
import inspect
import json
import math
import signal
import threading
from copy import deepcopy
from pathlib import Path


def driver_class(vendor, rclpy, config=None):
    class SessionDriver(vendor.PiperRosNode):
        def declare_parameter(self, name, value=None, *args, **kwargs):
            forced = {"auto_enable": False, "gripper_exist": True, "gripper_val_mutiple": 1}
            if name in forced:
                value = forced[name]
                kwargs["ignore_override"] = True
            return super().declare_parameter(name, value, *args, **kwargs)

        def __init__(self):
            self._session_stop = threading.Event()
            self._native_pose_publisher = None
            super().__init__()
            if self.GetEnableFlag():
                raise RuntimeError("vendor initialized enabled unexpectedly")
            if config:
                channel = self.get_parameter("can_port").value
                arm = next(a for a in config["arms"].values() if a["can_interface"] == channel)
                if arm.get("native_pose_topic"):
                    from std_msgs.msg import String

                    self._native_pose_type = String
                    self._native_pose_publisher = self.create_publisher(
                        String, arm["native_pose_topic"], 1
                    )
            if config and config.get("gripper_control_mode") == "independent_trigger":
                from sensor_msgs.msg import JointState

                channel = self.get_parameter("can_port").value
                arm = next(a for a in config["arms"].values() if a["can_interface"] == channel)
                self.create_subscription(
                    JointState, arm["gripper_command_topic"], self.gripper_callback, 1
                )

        def enable_callback(self, message):
            # This name is private to the explicitly hashed vendor dialect.
            # Do not inherit its zero-target gripper side effects.
            if message.data:
                self.piper.EnableArm(7)
            self._PiperRosNode__enable_flag = bool(message.data)

        def handle_enable_service(self, request, response):
            response.enable_response = False
            return response

        def gripper_callback(self, message):
            if (
                list(message.name) != ["gripper"]
                or len(message.position) != 1
                or len(message.effort) != 1
            ):
                raise ValueError("explicit jaw-only target required")
            width, effort = message.position[0], message.effort[0]
            lo, hi = self.piper.GetSDKGripperRangeParam()
            if (
                not math.isfinite(width)
                or not lo <= width <= hi
                or not math.isfinite(effort)
                or not 0.5 <= effort <= 3
            ):
                raise ValueError("invalid gripper-only target")
            self.piper.GripperCtrl(round(width * 1_000_000), round(effort * 1000), 0x01, 0)

        def pos_callback(self, message):
            self.get_logger().warning(
                "Cartesian driver bypass is disabled; use the session gateway"
            )

        def joint_callback(self, message):
            names = ["joint" + str(i) for i in range(1, 7)]
            if list(message.name) not in (names, names + ["gripper"]) or len(
                message.position
            ) != len(message.name):
                raise ValueError("full named arm target required")
            if not all(
                math.isfinite(v) for v in (*message.position, *message.velocity, *message.effort)
            ):
                raise ValueError("nonfinite driver message")
            if len(message.velocity) != 7 or not 1 <= message.velocity[6] <= 100:
                raise ValueError("explicit legacy driver speed required")
            if len(message.position) == 7:
                lo, hi = self.piper.GetSDKGripperRangeParam()
                if not lo <= message.position[6] <= hi:
                    raise ValueError("jaw outside SDK range")
            return super().joint_callback(message)

        def publish_thread(self):
            while not self._session_stop.is_set() and rclpy.ok():
                if self.piper.isOk():
                    self.PublishArmState()
                    self.PublishArmJointAndGripper()
                    self.PublishArmCtrlAndGripper()
                    self.PublishArmEndPose()
                self._session_stop.wait(0.005)

        def PublishArmEndPose(self):
            # Existing vendor quaternion topic remains explicitly upstream-derived.
            # The additional topic copies the already-connected SDK cache only.
            if self._native_pose_publisher is not None:
                from .piper_pose import native_end_pose

                value = native_end_pose(deepcopy(self.piper.GetArmEndPoseMsgs()))
                message = self._native_pose_type()
                message.data = json.dumps(value, allow_nan=False)
                self._native_pose_publisher.publish(message)
            super().PublishArmEndPose()

        def close(self):
            self._session_stop.set()
            self.publisher_thread.join(timeout=1)
            self.piper.DisconnectPort(thread_timeout=0.2)

    return SessionDriver


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", required=True)
    args, ros_args = p.parse_known_args(argv)
    config = json.loads(Path(args.config).read_text(encoding="utf-8-sig"))
    vendor = importlib.import_module(config["driver"]["module"])
    path = Path(inspect.getsourcefile(vendor.PiperRosNode))
    if hashlib.sha256(path.read_bytes()).hexdigest() != config["driver"]["sha256"]:
        raise ValueError("vendor driver source changed; audit before constructing hardware")
    if vendor.PiperRosNode.__name__ != "PiperRosNode":
        raise ValueError("unsupported vendor dialect")
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)

    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    node = None
    try:
        node = driver_class(vendor, rclpy, config)()
        print(
            json.dumps(
                {
                    "driver_source": str(path),
                    "auto_enable": False,
                    "gripper_enable_side_effect": False,
                    "sdk_gripper_range_m": node.piper.GetSDKGripperRangeParam(),
                }
            ),
            flush=True,
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
