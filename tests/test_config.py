from copy import deepcopy

import pytest

from robot_infra.graph import ConfigError, load_graph, validate_graph
from robot_infra.runtime import Session


def test_valid_graph_and_renamed_node_ids(config):
    plan = validate_graph(config)
    assert plan["config_hash"] == validate_graph(deepcopy(config))["config_hash"]
    config["nodes"][0]["id"] = "human"
    for edge in config["edges"]:
        edge["from"] = edge["from"].replace("fake_teleop.", "human.")
    assert validate_graph(config)["nodes_by_kind"]["fake_teleop"] == "human"


def corrupt(config, case):
    if case == "version":
        config["schema_version"] = "0.2"
    elif case == "hardware":
        config["mode"] = "real"
    elif case == "unknown_field":
        config["sdk_address"] = "forbidden"
    elif case == "missing_port":
        del config["nodes"][0]["ports"]["intent"]
    elif case == "type":
        config["nodes"][0]["ports"]["intent"]["type"] = "joint_state"
    elif case == "units":
        config["nodes"][0]["ports"]["intent"]["unit"] = "deg"
    elif case == "frame":
        config["nodes"][0]["ports"]["intent"]["frame_id"] = "world"
    elif case == "joint_order":
        config["nodes"][0]["ports"]["intent"]["joint_names"].reverse()
    elif case == "clock":
        config["nodes"][0]["ports"]["intent"]["clock_domain"] = "wall"
    elif case == "no_guard":
        config["nodes"] = [n for n in config["nodes"] if n["kind"] != "guard"]
    elif case == "bypass":
        config["edges"][0] = {"from": "identity_mapper.proposal", "to": "fake_robot.command"}
    elif case == "multiwriter":
        config["edges"].append({"from": "identity_mapper.proposal", "to": "fake_robot.command"})
    elif case == "duplicate_edge":
        config["edges"][0] = config["edges"][1]
    elif case == "missing_node":
        config["edges"][0]["from"] = "unknown.rgb"
    elif case == "direction":
        config["edges"][0]["from"] = "guard.proposal"
    elif case == "duplicate_node":
        config["nodes"][0]["id"] = config["nodes"][1]["id"]
    elif case == "limits":
        config["asset"]["limits"][0] = [1.0, -1.0]
    elif case == "initial":
        config["asset"]["initial_positions"][0] = 100.0
    elif case == "non_finite":
        config["asset"]["limits"][0][0] = float("-inf")
    elif case == "boolean_time":
        config["timing"]["period_ns"] = True
    return config


@pytest.mark.parametrize(
    "case",
    [
        "version",
        "hardware",
        "unknown_field",
        "missing_port",
        "type",
        "units",
        "frame",
        "joint_order",
        "clock",
        "no_guard",
        "bypass",
        "multiwriter",
        "duplicate_edge",
        "missing_node",
        "direction",
        "duplicate_node",
        "limits",
        "initial",
        "non_finite",
        "boolean_time",
    ],
)
def test_invalid_config_rejected_before_device_construction(config, case, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("invalid config constructed a device")

    monkeypatch.setattr("robot_infra.runtime.FakeRobot", forbidden)
    with pytest.raises(ConfigError):
        Session(corrupt(config, case))


@pytest.mark.parametrize(
    "body", ['{"mode":"fake","mode":"real"}', '{"value":NaN}', '{"value":Infinity}', "{bad"]
)
def test_strict_json(tmp_path, body):
    path = tmp_path / "bad.json"
    path.write_text(body)
    with pytest.raises(ConfigError):
        load_graph(path)


def test_stub_timing_budget_preflight(config):
    config["timing"]["period_ns"] = config["timing"]["max_action_age_ns"]
    with pytest.raises(ConfigError, match="horizon"):
        validate_graph(config)
