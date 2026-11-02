"""xArm6 geometry, used by the wrist guard to predict the joint angles a nearby target would need.

The joint frames are UFactory's published xArm6 values (xarm_ros, xarm6 URDF). Checked 2026-11-02 against the
16 different (joint angles, tool pose) pairs that sessions on the lab's arm recorded at their start: the tool
position came out within 1.3-3.6 mm of the controller's, its orientation within 0.8 deg, and joint 5 equal to
the angle between the axes of joints 4 and 6 within 0.1 deg. The few millimetres are the arm's own calibration;
predict() works with changes from the measured pose, so they cancel.

predict() is a local estimate (two damped least-squares steps from the measured joints) for targets a few
centimetres or degrees away, which is all the guard asks of it. It is not a general inverse kinematics.
Pure standard library; positions in metres, angles in degrees at the interface.
"""
from __future__ import annotations

import math
from typing import Sequence

H = math.pi / 2.0
# (origin of joint i in frame i-1 [m], rotation about x before joint i [rad]); every joint turns about its own z
JOINTS = (((0.0, 0.0, 0.267), 0.0), ((0.0, 0.0, 0.0), -H), ((0.0535, -0.2845, 0.0), 0.0),
          ((0.0775, 0.3425, 0.0), -H), ((0.0, 0.0, 0.0), H), ((0.076, 0.097, 0.0), -H))  # fmt: skip

Mat = list


def _mm(a: Mat, b: Mat) -> Mat:
    return [[a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3)] for i in range(3)]


def _mv(a: Mat, v: Sequence[float]) -> tuple:
    return tuple(a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2] for i in range(3))


def _t(a: Mat) -> Mat:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _rx(a: float) -> Mat:
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rz(a: float) -> Mat:
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _cross(a: Sequence[float], b: Sequence[float]) -> tuple:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def q_to_matrix(q: Sequence[float]) -> Mat:
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]  # fmt: skip


def _rotvec(r: Mat) -> tuple:
    """Rotation vector of the rotation matrix r."""
    c = max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0))
    v = (r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1])
    a = math.acos(c)
    s = math.sin(a)
    if s < 1e-6:
        if c > 0.0:
            return (v[0] / 2.0, v[1] / 2.0, v[2] / 2.0)
        d = [math.sqrt(max(0.0, (r[i][i] + 1.0) / 2.0)) for i in range(3)]  # a half turn: the axis from the diagonal
        k = max(range(3), key=lambda i: d[i])
        ax = [r[k][i] / (2.0 * d[k]) if i != k else d[k] for i in range(3)]
        return (ax[0] * a, ax[1] * a, ax[2] * a)
    return (v[0] * a / (2.0 * s), v[1] * a / (2.0 * s), v[2] * a / (2.0 * s))


def fk(joints_deg: Sequence[float]) -> tuple:
    """(tool position [m], tool rotation matrix, [(origin, axis) of each joint]) in the base frame, no TCP offset."""
    r: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    p = (0.0, 0.0, 0.0)
    axes = []
    for (o, ax), a in zip(JOINTS, joints_deg):
        d = _mv(r, o)
        p = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
        r = _mm(_mm(r, _rx(ax)), _rz(math.radians(a)))
        axes.append((p, (r[0][2], r[1][2], r[2][2])))
    return p, r, axes


def _solve(a: list, b: list) -> list:
    """a x = b for a small symmetric positive definite a (Gaussian elimination with pivoting)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        piv = max(range(c, n), key=lambda i: abs(m[i][c]))
        m[c], m[piv] = m[piv], m[c]
        for i in range(c + 1, n):
            f = m[i][c] / m[c][c]
            for j in range(c, n + 1):
                m[i][j] -= f * m[c][j]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (m[i][n] - sum(m[i][j] * x[j] for j in range(i + 1, n))) / m[i][i]
    return x


def predict(joints_deg: Sequence[float], dp_m: Sequence[float], q_from: Sequence[float] | None = None,
            q_to: Sequence[float] | None = None, steps: int = 2, damping: float = 0.01) -> tuple:
    """Joint angles [deg] near joints_deg for the tool moved by dp_m and turned from orientation q_from to q_to
    (unit quaternions x, y, z, w in the base frame; None = not turned)."""
    p0, r0, _ = fk(joints_deg)
    goal_p = (p0[0] + dp_m[0], p0[1] + dp_m[1], p0[2] + dp_m[2])
    goal_r = r0 if q_from is None or q_to is None else _mm(_mm(q_to_matrix(q_to), _t(q_to_matrix(q_from))), r0)
    q = [math.radians(a) for a in joints_deg]
    lam2 = damping * damping
    for _ in range(steps):
        p, r, axes = fk([math.degrees(a) for a in q])
        e = [goal_p[0] - p[0], goal_p[1] - p[1], goal_p[2] - p[2], *_rotvec(_mm(goal_r, _t(r)))]
        cols = []
        for o, z in axes:
            v = _cross(z, (p[0] - o[0], p[1] - o[1], p[2] - o[2]))
            cols.append((v[0], v[1], v[2], z[0], z[1], z[2]))
        jjt = [[sum(c[i] * c[k] for c in cols) + (lam2 if i == k else 0.0) for k in range(6)] for i in range(6)]
        y = _solve(jjt, e)
        for n, c in enumerate(cols):
            q[n] += sum(c[i] * y[i] for i in range(6))
    return tuple(math.degrees(a) for a in q)
