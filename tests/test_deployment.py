import json
from pathlib import Path

import pytest

from robot_infra.cli import main
from robot_infra.deployment import load_deployment


def test_portable_deployment_paths_are_resolved_against_manifest(tmp_path):
    c = json.loads((Path(__file__).parents[1] / "configs/deployment.example.json").read_text())
    p = tmp_path / "deployment.json"
    p.write_text(json.dumps(c))
    resolved = load_deployment(p)
    assert resolved["config"] == str(tmp_path / "profiles/robot.session.json")
    assert resolved["python_paths"][0] == str(tmp_path / "current/src")
    c["ros_domain_id"] = True
    p.write_text(json.dumps(c))
    with pytest.raises(ValueError, match="domain"):
        load_deployment(p)


def test_collection_demands_explicit_task_path_number_and_collision_mode():
    with pytest.raises(SystemExit) as error:
        main(["collect", "--deployment", "missing"])
    assert error.value.code == 2


def test_python_entry_demands_deployment_even_with_collection_arguments(capsys):
    with pytest.raises(SystemExit) as error:
        main(
            [
                "collect",
                "--task",
                "demo",
                "--dataset-root",
                "unused",
                "--start-episode",
                "0",
                "--existing",
                "skip",
            ]
        )
    assert error.value.code == 2 and "--deployment" in capsys.readouterr().err


def test_offline_episode_qc_missing_file_does_not_construct_ROS(capsys):
    assert main(["episode-qc", "does-not-exist"]) == 2
    assert "error" in capsys.readouterr().err


def test_collection_cli_image_storage_is_explicit_and_validated(monkeypatch):
    import robot_infra.deployment as deployment

    seen = []
    monkeypatch.setattr(deployment, "launch", lambda args: seen.append(args))
    args = [
        "collect",
        "--deployment",
        "example.json",
        "--task",
        "demo",
        "--dataset-root",
        "unused",
        "--start-episode",
        "0",
        "--existing",
        "skip",
    ]
    assert main(args + ["--image-storage", "mp4", "--check-only"]) == 0
    assert seen[0].image_storage == "mp4" and seen[0].check_only
    with pytest.raises(SystemExit):
        main(args + ["--image-storage", "unknown"])


def collection_device(tmp_path):
    root = Path(__file__).parents[1]
    session = json.loads((root / "configs/teleop-session.example.json").read_text())
    cameras = json.loads((root / "configs/realsense-streams.example.json").read_text())
    for i, camera in enumerate(cameras["cameras"]):
        camera["serial"] = str(100000 + i)
    cameras["cameras"][2]["role"] = "custom_head"
    session["camera_topics"]["custom_head"] = session["camera_topics"].pop("scene_fixed")
    deployment = json.loads((root / "configs/deployment.example.json").read_text())
    deployment.update(config="session.json", camera_config="cameras.json")
    for name, value in [("session", session), ("cameras", cameras), ("deployment", deployment)]:
        (tmp_path / (name + ".json")).write_text(json.dumps(value))
    return tmp_path / "deployment.json"


def test_collection_profile_setup_preserves_identity_and_uses_only_selected_depth(tmp_path):
    from robot_infra.deployment import prepare_collection

    deployment = collection_device(tmp_path)
    original = {p.name: p.read_bytes() for p in tmp_path.glob("*.json")}
    result = prepare_collection(
        deployment,
        tmp_path / "formal",
        trigger_pressed="close",
        teleop_start="follow_trigger",
        image_storage="mp4",
        depth_roles=["custom_head"],
        depth_profile=[640, 480, 30],
    )
    assert not result["physical_commands"] and result["image_storage"] == "mp4"
    assert all(
        a["trigger_pressed"] == "close" and a["teleop_start"] == "follow_trigger"
        for a in result["grippers"].values()
    )
    for role, p in result["camera_pipeline"].items():
        assert (
            p["capture"]["depth"]
            == p["record"]["depth"]
            == ({"width": 640, "height": 480, "fps": 30} if role == "custom_head" else None)
        )
        assert p["capture"]["rgb"]["fps"] == 30 and p["preview"]["fps"] == 8
    assert all((tmp_path / name).read_bytes() == value for name, value in original.items())
    copied = json.loads((tmp_path / "formal/cameras.json").read_text())
    assert copied["cameras"][2]["serial"] == "100002"
    c = load_deployment(result["deployment"])
    assert c["config"] == str(tmp_path / "formal/session.json")
    assert c["python_paths"][0] == str(tmp_path / "current/src")
    with pytest.raises(FileExistsError):
        prepare_collection(
            deployment,
            tmp_path / "formal",
            trigger_pressed="close",
            teleop_start="follow_trigger",
            image_storage="mp4",
            depth_roles=["custom_head"],
            depth_profile=[640, 480, 30],
        )


def test_collection_setup_rejects_unknown_role_before_writing(tmp_path):
    from robot_infra.deployment import prepare_collection

    deployment = collection_device(tmp_path)
    with pytest.raises(ValueError, match="depth role"):
        prepare_collection(
            deployment,
            tmp_path / "bad",
            trigger_pressed="close",
            teleop_start="follow_trigger",
            image_storage="mp4",
            depth_roles=["typo"],
            depth_profile=[640, 480, 30],
        )
    assert not (tmp_path / "bad").exists()


def test_collection_setup_cli_is_passive(tmp_path, capsys):
    deployment = collection_device(tmp_path)
    assert (
        main(
            [
                "collect-configure",
                "--deployment",
                str(deployment),
                "--output",
                str(tmp_path / "formal"),
                "--trigger-pressed",
                "close",
                "--teleop-start",
                "follow_trigger",
                "--image-storage",
                "mp4",
                "--depth-role",
                "custom_head",
                "--depth-profile",
                "640",
                "480",
                "30",
            ]
        )
        == 0
    )
    assert json.loads(capsys.readouterr().out)["physical_commands"] is False
