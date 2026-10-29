"""Small serial-chain URDF FK / damped least-squares IK, without device I/O.

This solver is intended for bounded, local Cartesian following. It does not
provide collision avoidance or global planning. Models and TCPs are explicit.
"""

import hashlib
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


def eye():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mul(a, b):
    return [[sum(x * y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def origin(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def axis_rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s, 0.0],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s, 0.0],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rotation_error(target, current):
    # Rotation-vector logarithm of R_target R_current^T, in the base frame.
    r = [[sum(target[i][k] * current[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    angle = math.acos(max(-1.0, min(1.0, (sum(r[i][i] for i in range(3)) - 1) / 2)))
    if angle > math.pi - 0.01:
        raise ValueError("local IK does not accept near-pi orientation jumps")
    scale = 0.5 if angle < 1e-7 else angle / (2 * math.sin(angle))
    return [scale * (r[2][1] - r[1][2]), scale * (r[0][2] - r[2][0]), scale * (r[1][0] - r[0][1])]


def solve_linear(a, b):
    rows = [list(row) + [v] for row, v in zip(a, b)]
    n = len(b)
    for i in range(n):
        pivot = max(range(i, n), key=lambda k: abs(rows[k][i]))
        rows[i], rows[pivot] = rows[pivot], rows[i]
        if abs(rows[i][i]) < 1e-14:
            raise ValueError("singular linear system")
        div = rows[i][i]
        rows[i] = [v / div for v in rows[i]]
        for j in range(n):
            if j != i:
                scale = rows[j][i]
                rows[j] = [x - scale * y for x, y in zip(rows[j], rows[i])]
    return [row[-1] for row in rows]


@dataclass(frozen=True)
class IKResult:
    positions: tuple
    accepted: bool
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    elapsed_ms: float
    reason: str
    seed_projection_rad: float = 0.0


class SerialModel:
    def __init__(self, path, base, tip, joint_names, *, sha256=None):
        raw = Path(path).read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        if sha256 is not None and self.sha256 != sha256:
            raise ValueError("model hash mismatch")
        root = ET.fromstring(raw)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link, seen = [], tip, set()
        while link != base:
            if link in seen or link not in by_child:
                raise ValueError("invalid base-to-tip chain")
            seen.add(link)
            j = by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        self.joints, self.names, self.limits = [], [], []
        for j in reversed(chain):
            kind = j.get("type")
            if kind not in ("fixed", "revolute"):
                raise ValueError("only fixed/revolute serial chains supported")
            o = j.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            axis_node = j.find("axis")
            axis = [
                float(x)
                for x in (
                    axis_node.get("xyz", "1 0 0") if axis_node is not None else "1 0 0"
                ).split()
            ]
            if (
                len(xyz) != 3
                or len(rpy) != 3
                or len(axis) != 3
                or abs(sum(x * x for x in axis) - 1) > 1e-6
            ):
                raise ValueError("invalid model origin or axis")
            if kind != "fixed":
                self.names.append(j.get("name"))
                limit = j.find("limit")
                self.limits.append((float(limit.get("lower")), float(limit.get("upper"))))
            self.joints.append((kind, origin(xyz, rpy), axis))
        if self.names != list(joint_names) or len(set(self.names)) != len(self.names):
            raise ValueError("joint order must match model chain")
        self.base, self.tip = base, tip

    def fk(self, q):
        if len(q) != len(self.names) or not all(math.isfinite(v) for v in q):
            raise ValueError("invalid joint state")
        result, i = eye(), 0
        for kind, transform, axis in self.joints:
            result = mul(result, transform)
            if kind != "fixed":
                result = mul(result, axis_rotation(axis, q[i]))
                i += 1
        return result

    def ik(
        self, target, seed, *, limits=None, max_ms=35, max_iterations=50, seed_tolerance_rad=0.0
    ):
        start = time.monotonic()
        q, bounds = list(seed), self.limits if limits is None else limits
        if (
            len(bounds) != len(q)
            or not all(math.isfinite(v) for v in q)
            or any(max(lo - v, v - hi, 0) > seed_tolerance_rad for v, (lo, hi) in zip(q, bounds))
        ):
            raise ValueError("IK seed outside explicit limits")
        projected = [max(lo, min(hi, v)) for v, (lo, hi) in zip(q, bounds)]
        projection = max(abs(a - b) for a, b in zip(projected, q))
        q = projected
        reason = "iteration_limit"
        for iteration in range(max_iterations + 1):
            current = self.fk(q)
            dp = [target[k][3] - current[k][3] for k in range(3)]
            dr = rotation_error(target, current)
            pe, re = math.sqrt(sum(v * v for v in dp)), math.sqrt(sum(v * v for v in dr))
            if pe < 0.0008 and re < 0.008:
                reason = "converged"
                break
            if (time.monotonic() - start) * 1000 > max_ms:
                reason = "deadline"
                break
            columns = []
            for j in range(len(q)):
                perturbed = q.copy()
                perturbed[j] += 1e-5
                f = self.fk(perturbed)
                columns.append(
                    [(f[k][3] - current[k][3]) / 1e-5 for k in range(3)]
                    + [v * 0.25 / 1e-5 for v in rotation_error(f, current)]
                )
            jac = list(map(list, zip(*columns)))
            error = dp + [v * 0.25 for v in dr]
            a = [
                [
                    sum(jac[k][i] * jac[k][j] for k in range(6)) + (0.002**2 if i == j else 0.0)
                    for j in range(len(q))
                ]
                for i in range(len(q))
            ]
            b = [sum(jac[k][i] * error[k] for k in range(6)) for i in range(len(q))]
            step = solve_linear(a, b)
            scale = min(1.0, 0.06 / max(1e-12, max(abs(v) for v in step)))
            q = [max(lo, min(hi, v + scale * d)) for v, d, (lo, hi) in zip(q, step, bounds)]
        return IKResult(
            tuple(q),
            reason == "converged",
            pe,
            re,
            iteration,
            (time.monotonic() - start) * 1000,
            reason,
            projection,
        )
