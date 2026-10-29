"""Optional Pinocchio model and SciPy optimizer; no ROS or hardware I/O.

SE(3) residual/Jlog6 follow Pinocchio's inverse-kinematics example. Bounded
optimization is delegated to SciPy; each thread owns separate Pinocchio Data.
"""

import hashlib
import threading
import time
from pathlib import Path

from .kinematics import IKResult


class PinocchioModel:
    def __init__(self, path, base, tip, joint_names, *, sha256=None, weights=None):
        import numpy as np
        import pinocchio as pin
        from scipy.optimize import least_squares

        self.np, self.pin, self.least_squares = np, pin, least_squares
        raw = Path(path).read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        if sha256 is not None and self.sha256 != sha256:
            raise ValueError("model hash mismatch")
        full = pin.buildModelFromUrdf(str(path))
        self.names = list(joint_names)
        if len(set(self.names)) != len(self.names) or any(
            not full.existJointName(n) for n in self.names
        ):
            raise ValueError("unknown or duplicate active joint")
        locked = [i for i, n in enumerate(full.names) if i and n not in self.names]
        self.model = pin.buildReducedModel(full, locked, pin.neutral(full)) if locked else full
        ids = [self.model.getJointId(n) for n in self.names]
        if self.model.nq != len(ids) or any(
            self.model.joints[i].nq != 1 or self.model.joints[i].nv != 1 for i in ids
        ):
            raise ValueError("this joint-position adapter requires scalar bounded joints")
        self.indices = [self.model.joints[i].idx_q for i in ids]
        self.velocity_indices = [self.model.joints[i].idx_v for i in ids]
        if not self.model.existFrame(base) or not self.model.existFrame(tip):
            raise ValueError("explicit base and tip must exist")
        self.base_id, self.tip_id = self.model.getFrameId(base), self.model.getFrameId(tip)
        if self.model.frames[self.base_id].parentJoint != 0:
            raise ValueError("base must be fixed relative to model root")
        self.base, self.tip = base, tip
        self.limits = [
            (float(self.model.lowerPositionLimit[i]), float(self.model.upperPositionLimit[i]))
            for i in self.indices
        ]
        self.velocity_limits = [float(self.model.velocityLimit[i]) for i in self.velocity_indices]
        self.weights = {
            "position": 20.0,
            "orientation": 2.0,
            "smooth": 2.0,
            "posture": 0.00001,
            **(weights or {}),
        }
        self._local = threading.local()

    def _data(self):
        if not hasattr(self._local, "data"):
            self._local.data = self.model.createData()
        return self._local.data

    def _q(self, positions):
        np = self.np
        values = np.asarray(positions, dtype=float)
        if values.shape != (len(self.names),) or not np.isfinite(values).all():
            raise ValueError("invalid named joint vector")
        q = self.pin.neutral(self.model)
        q[self.indices] = values
        return q

    def fk(self, positions):
        data = self._data()
        self.pin.framesForwardKinematics(self.model, data, self._q(positions))
        return (data.oMf[self.base_id].inverse() * data.oMf[self.tip_id]).homogeneous.tolist()

    def relative_target(self, initial_pose, initial_hand, hand, alignment, scale, rotation=True):
        np, pin = self.np, self.pin
        target = np.array(initial_pose, dtype=float)
        align = np.array(alignment, dtype=float)
        target[:3, 3] += (
            scale * align @ (np.array(hand["position_m"]) - np.array(initial_hand["position_m"]))
        )
        if rotation:

            def matrix(pose):
                xyzw = np.asarray(pose["orientation_xyzw"], dtype=float)
                norm = np.linalg.norm(xyzw)
                if not 0.99 <= norm <= 1.01:
                    raise ValueError("invalid tracked quaternion")
                return pin.Quaternion(xyzw / norm).matrix()

            target[:3, :3] = (
                align @ matrix(hand) @ matrix(initial_hand).T @ align.T @ target[:3, :3]
            )
        return target.tolist()

    def ik(
        self,
        target,
        seed,
        *,
        limits=None,
        max_ms=50,
        max_iterations=40,
        seed_tolerance_rad=0.005,
        allow_best_effort=True,
    ):
        np, pin = self.np, self.pin
        start = time.monotonic()
        bounds = np.asarray(self.limits if limits is None else limits, dtype=float)
        raw_seed = np.asarray(seed, dtype=float)
        self._q(raw_seed)
        if (
            bounds.shape != (len(seed), 2)
            or not np.isfinite(bounds).all()
            or np.any(bounds[:, 0] >= bounds[:, 1])
        ):
            raise ValueError("invalid IK joint limits")
        projected = np.clip(raw_seed, bounds[:, 0], bounds[:, 1])
        projection = float(np.max(np.abs(projected - raw_seed)))
        if projection > seed_tolerance_rad:
            raise ValueError("IK seed outside measurement tolerance")
        goal = np.asarray(target, dtype=float)
        if (
            goal.shape != (4, 4)
            or not np.isfinite(goal).all()
            or not np.allclose(goal[3], [0, 0, 0, 1])
        ):
            raise ValueError("invalid SE3 target")
        if (
            not np.allclose(goal[:3, :3].T @ goal[:3, :3], np.eye(3), atol=1e-5)
            or np.linalg.det(goal[:3, :3]) < 0.999
        ):
            raise ValueError("invalid target rotation")
        data = self._data()
        pin.framesForwardKinematics(self.model, data, self._q(projected))
        desired = data.oMf[self.base_id] * pin.SE3(goal[:3, :3], goal[:3, 3])
        weights = np.array([self.weights["position"]] * 3 + [self.weights["orientation"]] * 3)
        smooth, posture = self.weights["smooth"] ** 0.5, self.weights["posture"] ** 0.5
        best = [projected.copy(), float("inf")]
        calls = [0]

        class Deadline(Exception):
            pass

        def residual(x):
            if calls[0] and (time.monotonic() - start) * 1000 > max_ms:
                raise Deadline
            calls[0] += 1
            pin.framesForwardKinematics(self.model, data, self._q(x))
            delta = data.oMf[self.tip_id].actInv(desired)
            error = pin.log6(delta).vector
            result = np.concatenate((weights * error, smooth * (x - projected), posture * x))
            cost = float(result @ result)
            if cost < best[1]:
                best[:] = [np.array(x, copy=True), cost]
            return result

        def jacobian(x):
            q = self._q(x)
            pin.framesForwardKinematics(self.model, data, q)
            delta = data.oMf[self.tip_id].actInv(desired)
            jac = pin.computeFrameJacobian(
                self.model, data, q, self.tip_id, pin.ReferenceFrame.LOCAL
            )
            jac = (-pin.Jlog6(delta.inverse()) @ jac)[:, self.velocity_indices]
            return np.vstack(
                (weights[:, None] * jac, smooth * np.eye(len(seed)), posture * np.eye(len(seed)))
            )

        reason = "best_effort"
        try:
            result = self.least_squares(
                residual,
                projected,
                jac=jacobian,
                bounds=(bounds[:, 0], bounds[:, 1]),
                method="trf",
                max_nfev=max_iterations,
                ftol=1e-5,
                xtol=1e-5,
                gtol=1e-6,
            )
            q = result.x
        except Deadline:
            q, reason = best[0], "deadline_best_effort"
        actual = np.asarray(self.fk(q))
        pe = float(np.linalg.norm(goal[:3, 3] - actual[:3, 3]))
        re = float(np.linalg.norm(pin.log3(actual[:3, :3].T @ goal[:3, :3])))
        converged = pe < 0.003 and re < 0.03
        accepted = bool(np.isfinite(q).all() and (converged or allow_best_effort))
        if converged:
            reason = "converged"
        return IKResult(
            tuple(float(x) for x in q),
            accepted,
            pe,
            re,
            calls[0],
            (time.monotonic() - start) * 1000,
            reason,
            projection,
        )
