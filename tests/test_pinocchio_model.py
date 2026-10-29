import time

import pytest

pytest.importorskip("pinocchio")
np = pytest.importorskip("numpy")

from robot_infra.kinematics import SerialModel
from robot_infra.pinocchio_model import PinocchioModel
from robot_infra.solver_worker import SolverProcess


@pytest.fixture
def model_config(tmp_path):
    text = '<robot name="synthetic"><link name="base"/>'
    for i, axis in enumerate(("0 0 1", "0 1 0", "0 1 0"), 1):
        parent = "base" if i == 1 else f"l{i - 1}"
        text += f'<link name="l{i}"/><joint name="j{i}" type="revolute"><parent link="{parent}"/><child link="l{i}"/><origin xyz="{0 if i == 1 else 0.4} 0 0"/><axis xyz="{axis}"/><limit lower="-2" upper="2" velocity="2" effort="10"/></joint>'
    text += '<link name="tip"/><joint name="tool" type="fixed"><parent link="l3"/><child link="tip"/><origin xyz=".4 0 0"/></joint></robot>'
    path = tmp_path / "arm.urdf"
    path.write_text(text)
    return {"path": str(path), "base": "base", "tip": "tip", "joint_names": ["j1", "j2", "j3"]}


def test_pinocchio_fk_matches_independent_chain(model_config):
    m = PinocchioModel(**model_config)
    other = SerialModel(**model_config)
    for q in ([0, 0, 0], [0.4, -0.7, 0.3], [-1.0, 1.0, -0.5]):
        assert np.allclose(m.fk(q), other.fk(q), atol=1e-10)


def test_pinocchio_scipy_local_target_and_unreachable(model_config):
    m = PinocchioModel(**model_config, weights={"smooth": 0.0, "posture": 0.0})
    target = m.fk([0.32, -0.38, 0.18])
    r = m.ik(target, [0.3, -0.4, 0.2], max_ms=500)
    assert r.accepted and r.position_error_m < 0.003 and r.orientation_error_rad < 0.03
    target[0][3] = 10
    r = m.ik(target, [0.3, -0.4, 0.2], max_ms=500, allow_best_effort=False)
    assert not r.accepted and all(lo <= v <= hi for v, (lo, hi) in zip(r.positions, m.limits))


def test_relative_pose_rotation_is_not_head_motion(model_config):
    m = PinocchioModel(**model_config)
    pose = m.fk([0.2, -0.3, 0.1])
    hand0 = {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}
    hand = {"position_m": [0.1, 0.2, 0.3], "orientation_xyzw": [0, 0, np.sin(0.1), np.cos(0.1)]}
    target = m.relative_target(pose, hand0, hand, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 1.0, True)
    assert np.allclose(np.asarray(target)[:3, 3] - np.asarray(pose)[:3, 3], [0.1, 0.2, 0.3])
    assert not np.allclose(np.asarray(target)[:3, :3], np.asarray(pose)[:3, :3])


def test_worker_runs_off_thread_and_cleans_process(model_config):
    options = {
        "deadline_ms": 50,
        "max_iterations": 40,
        "allow_best_effort": True,
        "hang_timeout_s": 2.0,
    }
    worker = SolverProcess(model_config, options)
    try:
        until = time.monotonic() + 15
        while not worker.ready and not worker.error and time.monotonic() < until:
            worker.poll()
            time.sleep(0.01)
        assert worker.ready, worker.error
        model = PinocchioModel(**model_config)
        worker.request(
            {
                "target": model.fk([0.32, -0.38, 0.18]),
                "seed": [0.3, -0.4, 0.2],
                "created_s": time.monotonic(),
                "seq": 1,
                "token": "test",
            }
        )
        until = time.monotonic() + 3
        result = None
        while result is None and not worker.error and time.monotonic() < until:
            worker.poll()
            result = worker.take_result()
            time.sleep(0.005)
        assert result and result["token"] == "test" and result["result"]["accepted"]
    finally:
        worker.close()
    assert not worker.process.is_alive()
