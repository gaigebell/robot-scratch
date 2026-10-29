import copy
import http.client
import json
import threading
import urllib.error
import urllib.request

import pytest

from robot_infra.webxr_input import InputRecorder, make_server, validate_packet


def packet(seq=0, **kwargs):
    return {
        "schema_version": "webxr-input/1",
        "session_id": "fixture",
        "seq": seq,
        "event": "frame",
        "reference_space": "local-floor",
        "epoch": 0,
        "client_time_ms": 15.5,
        "client_unix_ms": 1000,
        "head": None,
        "inputs": [
            {
                "handedness": "left",
                "grip": {"position_m": [1, 2, 3], "orientation_xyzw": [0, 0, 0, 1]},
                "target_ray": None,
                "gamepad": {
                    "buttons": [{"value": 0.4, "pressed": False, "touched": True}],
                    "axes": [0, -0.2],
                },
            }
        ],
        **kwargs,
    }


@pytest.mark.parametrize(
    "changes",
    [
        {"seq": -1},
        {"epoch": True},
        {"reference_space": "robot-base"},
        {"client_time_ms": float("nan")},
        {"inputs": [{"handedness": "unknown"}]},
        {"head": {"position_m": [0, 0], "orientation_xyzw": [0, 0, 0, 1]}},
    ],
)
def test_invalid_input_does_not_reach_recording(changes):
    with pytest.raises(ValueError):
        validate_packet(packet(**changes))


def test_raw_tracking_loss_reset_and_time_domains_preserved(tmp_path):
    path = tmp_path / "xr.jsonl"
    recorder = InputRecorder(path)
    first = packet()
    recorder.accept(first)
    with pytest.raises(ValueError, match="sequence"):
        recorder.accept(first)
    lost = packet(
        2,
        epoch=1,
        head=None,
        inputs=[{"handedness": "left", "grip": None, "target_ray": None, "gamepad": None}],
    )
    recorder.accept(lost)
    state = recorder.status()["sessions"]["fixture"]
    assert state["sequence_gaps"] == 1 and state["latest"]["inputs"][0]["grip"] is None
    recorder.accept(packet(3, event="session_end", inputs=[]))
    assert not recorder.status()["sessions"]["fixture"]["fresh"]
    recorder.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]["raw_input"] == first and events[1]["raw_input"] == lost
    assert events[0]["clock_relation"] == "unmeasured"
    assert events[0]["host_monotonic_ns"] <= events[1]["host_monotonic_ns"]
    with pytest.raises(FileExistsError):
        InputRecorder(path)


def test_capacity_and_bad_buttons_are_explicit(tmp_path):
    recorder = InputRecorder(tmp_path / "limit.jsonl", max_sessions=1)
    recorder.accept(packet())
    with pytest.raises(ValueError, match="session capacity"):
        recorder.accept(packet(session_id="second"))
    invalid = copy.deepcopy(packet(1))
    invalid["inputs"][0]["gamepad"]["buttons"][0]["value"] = 2
    with pytest.raises(ValueError, match="button"):
        recorder.accept(invalid)
    recorder.close()
    recorder = InputRecorder(tmp_path / "bytes.jsonl", max_bytes=1)
    with pytest.raises(ValueError, match="recording capacity"):
        recorder.accept(packet())
    assert recorder.status()["events"] == 0
    recorder.close()


def test_http_ingest_and_status_expose_input_only(tmp_path):
    recorder = InputRecorder(tmp_path / "http.jsonl")
    server = make_server(("127.0.0.1", 0), recorder)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = "http://127.0.0.1:" + str(server.server_address[1])
    try:
        with urllib.request.urlopen(url + "/") as r:
            assert b"VR" in r.read()
        request = urllib.request.Request(
            url + "/api/input",
            data=json.dumps(packet()).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request) as r:
            assert json.load(r)["accepted"]
        with urllib.request.urlopen(url + "/api/status") as r:
            state = json.load(r)
        assert state["events"] == 1 and state["robot_commands"] == "unavailable"
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(urllib.request.Request(url + "/api/command", data=b"{}"))
        assert exc.value.code == 404
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(request)
        assert exc.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)
        recorder.close()


def test_frame_transport_reuses_http_connection(tmp_path):
    recorder = InputRecorder(tmp_path / "keepalive.jsonl")
    server = make_server(("127.0.0.1", 0), recorder)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = http.client.HTTPConnection(*server.server_address, timeout=2)
    try:
        first_socket = None
        for seq in range(3):
            client.request(
                "POST", "/api/input", json.dumps(packet(seq)), {"Content-Type": "application/json"}
            )
            response = client.getresponse()
            assert response.status == 200
            response.read()
            if first_socket is None:
                first_socket = client.sock
            assert client.sock is first_socket
        assert recorder.status()["events"] == 3
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(3)
        recorder.close()
