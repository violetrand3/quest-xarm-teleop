"""Rotation support for the teleop adapter: quaternion helpers and the wrist-rotation follower.

Quaternions are (x, y, z, w), unit length, as in the Quest telemetry. A rotation vector is
axis * angle in radians. Everything here is pure math; nothing talks to hardware.

xArm roll/pitch/yaw convention assumed by rpy_to_q / q_to_rpy:  R = Rz(yaw) * Ry(pitch) * Rx(roll)
(rotations about the FIXED base X, then Y, then Z axes). The live program checks this against the
controller's own axis-angle readout of the same pose before it relies on it.

How the tool follows the hand (RotationFollower):

    D        = q_ctrl(now) * inverse(q_ctrl at engagement)        hand rotation since engagement, in LOCAL
    D        = twist of D about "up"                               only in mode "yaw"
    r_base   = scale * R * rotvec(D)                               same rotation expressed in the backend frame
    q_target = exp(r_base) * q_tool at engagement

then q_target is held within max_angle of the tool orientation at the FIRST engagement of the session
(so repeated clutching cannot ratchet the tool further and further), and the commanded orientation
moves toward it at no more than w_max.

With tilt_max and turn_max set, those two replace max_angle (limit_tilt_turn): the tool's tilt is measured
from pointing straight down (its z axis along the backend's -z) and held within tilt_max, or within the tilt
at the first engagement when that is larger; its turn about the vertical is measured from the orientation at
the first engagement and held within +-turn_max. A pose left tilted by another program can then always be
straightened, and the jaws can be turned across an object lying in any direction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def q_norm(q: Sequence[float]) -> tuple:
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_mul(a: Sequence[float], b: Sequence[float]) -> tuple:
    """a * b: the rotation b followed by the rotation a (both in the same fixed frame)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)  # fmt: skip


def q_conj(q: Sequence[float]) -> tuple:
    return (-q[0], -q[1], -q[2], q[3])


def q_from_rotvec(r: Sequence[float]) -> tuple:
    a = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    if a < 1e-12:
        return q_norm((r[0] / 2.0, r[1] / 2.0, r[2] / 2.0, 1.0))
    s = math.sin(a / 2.0) / a
    return (r[0] * s, r[1] * s, r[2] * s, math.cos(a / 2.0))


def q_to_rotvec(q: Sequence[float]) -> tuple:
    """Shortest rotation vector (angle 0..pi)."""
    x, y, z, w = q
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-12:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    a = 2.0 * math.atan2(s, w)
    return (x / s * a, y / s * a, z / s * a)


def q_angle(a: Sequence[float], b: Sequence[float]) -> float:
    """Angle (0..pi) of the rotation that takes orientation a to orientation b."""
    x, y, z, w = q_mul(b, q_conj(a))
    return 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))  # accurate for small angles, unlike acos of the dot product


def q_step_toward(q_from: Sequence[float], q_to: Sequence[float], max_angle: float) -> tuple:
    """Move from q_from toward q_to by at most max_angle (the short way round)."""
    r = q_to_rotvec(q_mul(q_to, q_conj(q_from)))
    a = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    if a <= max_angle:
        return q_norm(q_to)
    k = max_angle / a
    return q_norm(q_mul(q_from_rotvec((r[0] * k, r[1] * k, r[2] * k)), q_from))


def q_twist(q: Sequence[float], axis: Sequence[float]) -> tuple:
    """The part of rotation q that is a rotation about the unit vector `axis` (swing-twist decomposition)."""
    p = q[0] * axis[0] + q[1] * axis[1] + q[2] * axis[2]
    t = (axis[0] * p, axis[1] * p, axis[2] * p, q[3])
    n = math.sqrt(sum(v * v for v in t))
    return IDENTITY if n < 1e-9 else (t[0] / n, t[1] / n, t[2] / n, t[3] / n)


def rpy_to_q(roll: float, pitch: float, yaw: float) -> tuple:
    """R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    qx = (math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0))
    qy = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
    qz = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return q_norm(q_mul(qz, q_mul(qy, qx)))


def q_to_rpy(q: Sequence[float]) -> tuple:
    """Inverse of rpy_to_q. Near pitch = +-90 deg (gimbal lock) roll is set to 0."""
    x, y, z, w = q_norm(q)
    r20 = 2.0 * (x * z - w * y)
    pitch = -math.asin(max(-1.0, min(1.0, r20)))
    if abs(r20) > 1.0 - 1e-9:
        return (0.0, pitch, math.atan2(-2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z)))
    roll = math.atan2(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def mat_vec(R: Sequence[Sequence[float]], v: Sequence[float]) -> tuple:
    return tuple(R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2] for i in range(3))


def tool_axis(q: Sequence[float]) -> tuple:
    """The tool's z axis (the way the jaws point) in the backend frame: the third column of R(q)."""
    x, y, z, w = q_norm(q)
    return (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y))


def tilt_from_down(q: Sequence[float]) -> float:
    """Angle between the tool's z axis and straight down (the backend's -z), radians, 0..pi."""
    return math.acos(max(-1.0, min(1.0, -tool_axis(q)[2])))


def turn_about_vertical(q: Sequence[float], q_ref: Sequence[float]) -> float:
    """Signed turn about the backend's vertical from q_ref to q (the twist of q * q_ref^-1 about z), radians, -pi..pi."""
    t = q_twist(q_mul(q, q_conj(q_ref)), (0.0, 0.0, 1.0))
    a = 2.0 * math.atan2(t[2], t[3])
    return a - 2.0 * math.pi if a > math.pi else a + 2.0 * math.pi if a < -math.pi else a


def limit_tilt_turn(q: Sequence[float], q_ref: Sequence[float], tilt_max: float, turn_max: float) -> tuple:
    """Hold q within tilt_max of straight down (tilted back toward down along the shortest way, which keeps the
    direction it leans in) and then within +-turn_max of q_ref about the vertical. Turning about the vertical does
    not change the tilt, so both hold together."""
    q = q_norm(q)
    a = tool_axis(q)
    t = math.acos(max(-1.0, min(1.0, -a[2])))
    if t > tilt_max:
        k = (-a[1], a[0], 0.0)  # a x down: turning about it moves the tool's axis toward down
        n = math.sqrt(k[0] * k[0] + k[1] * k[1])
        k = (1.0, 0.0, 0.0) if n < 1e-9 else (k[0] / n, k[1] / n, 0.0)
        q = q_norm(q_mul(q_from_rotvec(tuple(v * (t - tilt_max) for v in k)), q))
    th = turn_about_vertical(q, q_ref)
    if abs(th) > turn_max:
        q = q_norm(q_mul(q_from_rotvec((0.0, 0.0, math.copysign(turn_max, th) - th)), q))
    return q


@dataclass(frozen=True)
class RotationConfig:
    mode: str  # "yaw": only the hand's rotation about "up" turns the tool about the backend's z; "full": all three axes
    scale: float
    max_angle_rad: float  # from the tool orientation at the first engagement of the session
    w_max_rad_s: float
    max_error_rad: float  # commanded vs reported tool orientation
    max_step_rad: float  # between two consecutive accepted controller samples
    tilt_max_rad: float | None = None  # with turn_max_rad: replaces max_angle_rad (see limit_tilt_turn)
    turn_max_rad: float | None = None


class RotationFollower:
    def __init__(self, cfg: RotationConfig, R_backend_from_local: Sequence[Sequence[float]]):
        self.cfg, self.R = cfg, R_backend_from_local
        self.up_local = tuple(R_backend_from_local[2][k] for k in range(3))  # R^T * (0, 0, 1): the LOCAL direction mapped to backend +z
        self.q_ref: tuple | None = None  # tool orientation at the first engagement; the angle limit is measured from it
        self.tilt_lim: float | None = None  # tilt_max, or the tilt at the first engagement when that is larger
        self.q_ctrl0 = self.q_tool0 = self.last_q = self.q_req = self.q_lim = None

    def clear(self) -> None:
        self.q_ctrl0 = self.q_tool0 = self.last_q = self.q_req = self.q_lim = None

    def engage(self, q_ctrl: Sequence[float], q_tool: Sequence[float]) -> None:
        q_ctrl, q_tool = q_norm(q_ctrl), q_norm(q_tool)
        if self.q_ref is None:
            self.q_ref = q_tool
            if self.cfg.tilt_max_rad is not None:
                self.tilt_lim = max(self.cfg.tilt_max_rad, tilt_from_down(q_tool) + 1e-6)
        self.q_ctrl0, self.q_tool0, self.last_q, self.q_req, self.q_lim = q_ctrl, q_tool, q_ctrl, q_tool, q_tool

    def on_sample(self, q_ctrl: Sequence[float] | None) -> str | None:
        """A new accepted controller sample. Returns a fault reason or None."""
        if q_ctrl is None:
            return "controller_discontinuity:orientation missing"
        q_ctrl = q_norm(q_ctrl)
        step = q_angle(self.last_q, q_ctrl)
        if step > self.cfg.max_step_rad:
            return f"controller_discontinuity:rotation step {math.degrees(step):.0f}deg > {math.degrees(self.cfg.max_step_rad):.0f}deg"
        self.last_q = q_ctrl
        d = q_mul(q_ctrl, q_conj(self.q_ctrl0))
        if self.cfg.mode == "yaw":
            d = q_twist(d, self.up_local)
        r = mat_vec(self.R, q_to_rotvec(d))
        q = q_norm(q_mul(q_from_rotvec(tuple(self.cfg.scale * v for v in r)), self.q_tool0))
        if self.tilt_lim is not None:
            q = limit_tilt_turn(q, self.q_ref, self.tilt_lim, self.cfg.turn_max_rad)
        else:
            rel = q_to_rotvec(q_mul(q, q_conj(self.q_ref)))
            a = math.sqrt(sum(v * v for v in rel))
            if a > self.cfg.max_angle_rad:  # hold at the limit, on the same axis
                k = self.cfg.max_angle_rad / a
                q = q_norm(q_mul(q_from_rotvec(tuple(v * k for v in rel)), self.q_ref))
        self.q_req = q
        return None

    def step(self, dt_s: float) -> tuple:
        self.q_lim = q_step_toward(self.q_lim, self.q_req, self.cfg.w_max_rad_s * max(0.0, dt_s))
        return self.q_lim

    def tracking_problem(self, q_fb: Sequence[float] | None) -> str | None:
        if q_fb is None:
            return "target_feedback_error:rotation feedback missing"
        e = q_angle(self.q_lim, q_fb)
        if e > self.cfg.max_error_rad:
            return f"target_feedback_error:rotation {math.degrees(e):.1f}deg > {math.degrees(self.cfg.max_error_rad):.0f}deg"
        return None

    def angles_deg(self) -> tuple:
        """(commanded, requested) rotation from the engagement orientation, degrees; for display."""
        if self.q_tool0 is None:
            return None, None
        return round(math.degrees(q_angle(self.q_tool0, self.q_lim)), 2), round(math.degrees(q_angle(self.q_tool0, self.q_req)), 2)
