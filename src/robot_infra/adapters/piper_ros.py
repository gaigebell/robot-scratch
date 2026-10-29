"""Explicit adapter for the audited Piper ROS JointState driver dialect.

The gateway alone holds the write capability. ROS stays outside the core.
v1 uses the audited arm-only driver; v2 requires the checked driver wrapper.
"""


class PiperROSBackend:
    def __init__(self, node, config, joint_names):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        self.node, self.config, self.names = node, config, list(joint_names)
        self.JointState, self.Bool = JointState, Bool
        self.capability = None
        self.publishers, self.enablers = {}, {}
        self.grippers = {}
        for side, arm in config["arms"].items():
            self.publishers[side] = node.create_publisher(JointState, arm["command_topic"], 1)
            self.enablers[side] = node.create_publisher(Bool, arm["enable_topic"], 1)
            if config.get("gripper_control_mode") == "independent_trigger":
                self.grippers[side] = node.create_publisher(
                    JointState, arm["gripper_command_topic"], 1
                )

    def bind(self, capability):
        if self.capability is not None:
            raise RuntimeError("backend already bound")
        self.capability = capability

    def _check(self, side, capability):
        if capability is not self.capability or capability is None:
            raise PermissionError("gateway capability required")
        # This is process/domain coordination, not a DDS security boundary.
        if self.node.count_publishers(self.config["arms"][side]["command_topic"]) != 1:
            raise RuntimeError("command topic must have exactly one publisher")
        if self.node.count_publishers(self.config["arms"][side]["enable_topic"]) != 1:
            raise RuntimeError("enable topic must have exactly one publisher")
        if self.node.count_subscribers(self.config["arms"][side]["command_topic"]) != 1:
            raise RuntimeError("expected exactly one isolated driver subscriber")
        if side in self.grippers:
            topic = self.config["arms"][side]["gripper_command_topic"]
            if self.node.count_publishers(topic) != 1 or self.node.count_subscribers(topic) != 1:
                raise RuntimeError("gripper topic requires one gateway and one driver")

    def enable(self, side, capability):
        self._check(side, capability)
        msg = self.Bool()
        msg.data = True
        self.enablers[side].publish(msg)

    def ready(self):
        missing = []
        for side in self.config["arms"]:
            try:
                self._check(side, self.capability)
            except (RuntimeError, PermissionError) as exc:
                missing.append(side + ": " + str(exc))
        return not missing, missing

    def send(self, side, q, gripper, capability):
        self._check(side, capability)
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.names + (["gripper"] if gripper is not None else [])
        msg.position = list(q) + ([float(gripper)] if gripper is not None else [])
        # Audited legacy dialect: seventh velocity is global speed percentage.
        # A six-position message intentionally omits any gripper target; the
        # legacy driver still consumes seventh velocity as speed percentage.
        msg.velocity = [0.0] * 6 + [float(self.config["driver_speed_percent"])]
        effort = self.config["arms"][side].get("gripper", {}).get("legacy_effort", 1.0)
        msg.effort = [0.0] * 6 + [float(effort)]
        self.publishers[side].publish(msg)

    def send_gripper(self, side, width, capability):
        self._check(side, capability)
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(width)]
        msg.effort = [float(self.config["arms"][side]["gripper"]["legacy_effort"])]
        self.grippers[side].publish(msg)
