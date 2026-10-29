"""Synthetic devices only. Deliberately no discovery, sockets or SDK imports."""

import base64


class ManualClock:
    domain = "fake_monotonic"

    def __init__(self):
        self.now_ns = 0

    def advance(self, ns: int):
        if type(ns) is not int or ns < 0:
            raise ValueError("fake clock cannot go backwards")
        self.now_ns += ns


class FakeRobot:
    def __init__(self, initial_positions, writer_capability):
        self.positions = tuple(initial_positions)
        self._writer_capability = writer_capability
        self.calls = 0
        self.stops = 0

    def _execute(self, positions, capability):
        if capability is not self._writer_capability:
            raise PermissionError("only final guard may write fake robot")
        self.positions = tuple(positions)
        self.calls += 1

    def stop(self):
        # No physics: revoke software writes. This is not a physical stop model.
        self.stops += 1


class FakeCamera:
    def __init__(self):
        self.seq = 0

    def sample(self, now_ns: int, calibration_hash: str):
        self.seq += 1
        pixels = bytes((self.seq + i) % 256 for i in range(12))
        return {
            "source": "fake_camera",
            "role": "front",
            "seq": self.seq,
            "width": 2,
            "height": 2,
            "encoding": "rgb8",
            "data_base64": base64.b64encode(pixels).decode(),
            "device_time_ns": now_ns,
            "host_monotonic_ns": now_ns,
            "receive_ns": now_ns,
            "clock_domain": "fake_monotonic",
            "calibration_hash": calibration_hash,
        }


class FakeTeleoperator:
    def sample(self, step: int):
        return (0.2 if step % 2 == 0 else -0.2, 0.1)


class StubPolicy:
    """Returns a known native joint-position chunk; no model weights."""

    artifact = {
        "id": "stub-joint-position-v1",
        "action_schema": "joint_position",
        "unit": "rad",
        "frame_id": "fake_base",
        "joint_names": ["joint_a", "joint_b"],
        "weights": None,
        "preprocessing": "synthetic-identity-v1",
    }

    def predict(self):
        return ((0.3, -0.1), (0.35, -0.1))
