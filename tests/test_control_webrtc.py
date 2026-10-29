"""Real local UDP/DTLS/SCTP round trip; never imports a hardware backend."""

import asyncio
import json
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("aiortc")
from aiortc import RTCConfiguration, RTCPeerConnection

from robot_infra.control_webrtc import WebRTCInputServer


def test_ordered_existing_packets_error_disconnect_and_cleanup():
    class App:
        session = SimpleNamespace(viewer=None)

        def __init__(self):
            self.calls = []
            self.pauses = []
            self.order = []

        def input(self, packet):
            if packet["seq"] < 0:
                raise ValueError("reordered input")
            time.sleep(0.01)  # Simulated work must not block the network loop.
            self.calls.append(packet)
            self.session.viewer = packet["session_id"]
            self.order.append("input")
            return {"accepted": True}

        def pause(self, reason):
            self.pauses.append(reason)
            self.order.append("pause")

    app = App()
    server = WebRTCInputServer(app)

    async def scenario():
        pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        dc = pc.createDataChannel("robot-input", ordered=True)
        replies = asyncio.Queue()
        dc.on("message", lambda raw: replies.put_nowait(json.loads(raw)))
        try:
            await pc.setLocalDescription(await pc.createOffer())
            answer = await asyncio.to_thread(
                server.offer, {"type": pc.localDescription.type, "sdp": pc.localDescription.sdp}
            )
            from aiortc import RTCSessionDescription

            await pc.setRemoteDescription(RTCSessionDescription(**answer))
            assert (await asyncio.wait_for(replies.get(), 5))["type"] == "ready"
            packets = [
                {"schema_version": "webxr-input/1", "session_id": "test", "seq": i, "A": i % 2}
                for i in range(8)
            ]
            for packet in packets:
                dc.send(json.dumps(packet))  # No per-packet ACK gate.
            for i in range(8):
                reply = await asyncio.wait_for(replies.get(), 5)
                assert reply["type"] == "ack" and reply["seq"] == i
            assert app.calls == packets
            dc.send(json.dumps({"seq": -1}))
            assert (await asyncio.wait_for(replies.get(), 5))["type"] == "error"
            dc.send("[]")
            assert (await asyncio.wait_for(replies.get(), 5))["type"] == "error"
            with pytest.raises(ValueError, match="already connected"):
                await asyncio.to_thread(server.offer, answer | {"type": "offer"})
            # Closing during application work must put pause AFTER that work,
            # rather than letting an already running input overwrite the stop.
            dc.send(json.dumps({"session_id": "test", "seq": 8}))
            await asyncio.sleep(0.002)
        finally:
            await pc.close()
        for _ in range(100):
            if app.pauses:
                break
            await asyncio.sleep(0.01)
        assert len(app.pauses) == 1
        await asyncio.sleep(0.03)
        assert app.order[-1] == "pause"

    try:
        asyncio.run(scenario())
    finally:
        server.close()
    assert not server.thread.is_alive()


def test_invalid_offer_does_not_reserve_a_peer():
    server = WebRTCInputServer(SimpleNamespace())
    try:
        for body in ([], {}, {"type": "answer", "sdp": "x"}, {"type": "offer", "sdp": "x" * 60001}):
            with pytest.raises(ValueError):
                server.offer(body)
        with pytest.raises(ValueError):
            server.offer({"type": "offer", "sdp": "invalid"})
        assert server.peer is None
    finally:
        server.close()
