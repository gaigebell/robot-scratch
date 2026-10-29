"""Streaming readers for real episodes. No ROS, hardware or training imports."""

import json
from pathlib import Path


def episode_manifest(root):
    root = Path(root).expanduser().resolve()
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema_version") not in ("real-episode/1", "real-episode/2"):
        raise ValueError("unsupported real episode")
    return root, manifest


def iter_events(root, *, kinds=None, feedback=True):
    from mcap.reader import NonSeekingReader

    root = Path(root)
    with (root / "streams.mcap").open("rb") as source:
        for _, channel, message in NonSeekingReader(source).iter_messages(log_time_order=False):
            if channel.message_encoding != "json" or not channel.topic.startswith(
                ("/event/", "/feedback/")
            ):
                continue
            if not feedback and channel.topic.startswith("/feedback/"):
                continue
            if (
                kinds is not None
                and channel.topic.rsplit("/", 1)[-1] not in kinds
                and not (feedback and channel.topic.startswith("/feedback/"))
            ):
                continue
            value = json.loads(message.data)
            if value.get("schema_version") != "real-episode-event/1":
                raise ValueError("unsupported event envelope")
            yield {"topic": channel.topic, "sequence": message.sequence, "event": value}


def read_index(root):
    from collections import defaultdict

    rows = defaultdict(list)
    with (Path(root) / "index.jsonl").open(encoding="utf-8") as source:
        for line in source:
            row = json.loads(line)
            rows[row["stream"]].append(row)
    return dict(rows)
