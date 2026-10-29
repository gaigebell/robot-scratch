import json
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("websockets.sync.server")
from websockets.exceptions import InvalidStatus
from websockets.sync.client import connect

from robot_infra.control_socket import ControlSocketServer


def test_dedicated_input_channel_order_errors_and_disconnect_pause():
    class App:
        session = SimpleNamespace(viewer="vr")

        def __init__(self):
            self.calls = []
            self.pauses = []

        def input(self, p):
            if p["seq"] <= len(self.calls) - 1:
                raise ValueError("reordered")
            self.calls.append(p)
            return {"accepted": True}

        def pause(self, reason):
            self.pauses.append(reason)

    app = App()
    server = ControlSocketServer(app, "127.0.0.1", 0, None, 8445)
    try:
        uri = f"ws://127.0.0.1:{server.port}/input"
        with connect(uri, origin="https://127.0.0.1:8445", proxy=None) as ws:
            assert json.loads(ws.recv())["type"] == "ready"
            for n in range(4):
                ws.send(json.dumps({"session_id": "vr", "seq": n}))
                reply = json.loads(ws.recv())
                assert reply["seq"] == n and reply["type"] == "ack"
            ws.send(json.dumps({"session_id": "vr", "seq": 0}))
            assert json.loads(ws.recv())["type"] == "error"
        until = time.monotonic() + 1
        while not app.pauses and time.monotonic() < until:
            time.sleep(0.005)
        assert app.pauses == ["control_socket_closed"] and len(app.calls) == 4
        with pytest.raises(InvalidStatus):
            with connect(uri, origin="https://unrelated.example:8445", proxy=None):
                pass
    finally:
        server.close()
    assert not server.thread.is_alive()
