"""Read-only PoseStamped/native JSON subscriptions; never writes to a device."""

import json

from ..pose_feedback import native_pose_sample, pose_sample, validate_pose_sources


def subscribe_pose_feedback(node, sources, emit, *, message_type=None):
    sources = validate_pose_sources(sources)
    if not sources:
        return []
    subscriptions = []
    for source_id, definition in sources.items():
        typ = message_type
        if typ is None:
            if definition["format"] == "pose_stamped":
                from geometry_msgs.msg import PoseStamped

                typ = PoseStamped
            else:
                from std_msgs.msg import String

                typ = String

        def receive(message, name=source_id, spec=definition):
            try:
                if spec["format"] == "pose_stamped":
                    p, q = message.pose.position, message.pose.orientation
                    sample = pose_sample(
                        name,
                        spec,
                        [p.x, p.y, p.z],
                        [q.x, q.y, q.z, q.w],
                        message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                        message.header.frame_id,
                    )
                else:
                    sample = native_pose_sample(name, spec, json.loads(message.data))
            except (ValueError, TypeError, KeyError) as exc:
                emit(
                    "observation_error",
                    {
                        "source_id": name,
                        "topic": spec["topic"],
                        "observation_kind": "pose_feedback",
                        "error": str(exc),
                    },
                )
                return
            emit("pose_feedback", sample)

        subscriptions.append(node.create_subscription(typ, definition["topic"], receive, 1))
    return subscriptions
