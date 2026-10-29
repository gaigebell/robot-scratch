import math
import struct

import pytest

from robot_infra.adapters.socketcan_rx import PiperV2Joints, receive_sample


def test_piper_feedback_assembly_keeps_raw_units_and_pair_times():
    decoder = PiperV2Joints()
    assert decoder.update(0x2A5, struct.pack(">ii", 90000, -90000), 100) is None
    assert decoder.update(0x2A6, struct.pack(">ii", 0, 180000), 110) is None
    result = decoder.update(0x2A7, struct.pack(">ii", -180000, 45000), 120)
    assert result["position_rad"] == pytest.approx(
        [math.pi / 2, -math.pi / 2, 0, math.pi, -math.pi, math.pi / 4]
    )
    assert result["assembly_skew_ns"] == 20
    assert decoder.update(0x800002A5, b"12345678", 130) is None
    assert decoder.update(0x2A5, b"bad", 130) is None


def test_socketcan_receiver_uses_only_receive_calls(tmp_path, monkeypatch):
    from robot_infra.adapters import socketcan_rx as module

    frames = [
        struct.pack("=IB3x8s", key, 8, struct.pack(">ii", 1, 2)) for key in (0x2A5, 0x2A6, 0x2A7)
    ]

    class Socket:
        def bind(self, address):
            assert address == ("fixture0",)

        def setblocking(self, value):
            assert value is False

        def recv(self, size):
            if frames:
                return frames.pop(0)
            raise BlockingIOError()

        def close(self):
            pass

        def send(self, *args):
            pytest.fail("receive adapter must never send")

    for name, value in [("AF_CAN", 29), ("CAN_RAW", 1)]:
        monkeypatch.setattr(module.socket, name, value, raising=False)
    monkeypatch.setattr(module.socket, "socket", lambda *a: Socket())
    monkeypatch.setattr(module.select, "select", lambda sockets, *a: (sockets, [], []))
    report = receive_sample(
        {"custom.arm": "fixture0"},
        tmp_path / "rx",
        duration=0.01,
        max_frames=2,
        decoders={"custom.arm": PiperV2Joints()},
    )
    assert report["all_have_joint_feedback"] and report["socket_send_calls"] == 0
    assert report["channels"]["custom.arm"]["frames_saved"] == 2
    assert len((tmp_path / "rx/raw-can.jsonl").read_text().splitlines()) == 2


def test_can_sample_rejects_two_roles_on_same_channel(tmp_path):
    with pytest.raises(ValueError):
        receive_sample({"a": "can0", "b": "can0"}, tmp_path / "invalid")
