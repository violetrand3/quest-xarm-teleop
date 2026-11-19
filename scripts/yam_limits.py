"""Bimanual YAM: what a requested tool pose must pass before it is solved for, and what a solved joint step must
pass before it is commanded. Pure math, one instance per arm; nothing here talks to an arm.

In each control step, for each arm:
  1. TargetLimiter.step: a requested pose that jumped (a tracking glitch, not a hand movement) is refused and
     the arm is let go; otherwise the request is held inside the arm's box, and the target moves toward it at
     no more than the tool speed limits, speeding up and slowing down at no more than the tool accelerations
     (so it starts and stops smoothly instead of jumping to full speed), never more than `lag` ahead of the arm
     (so an arm that is slowed or held does not leave a target running away, to be caught up with later).
  2. the IK solves for that target (yam_sim / the arm layer);
  3. joint_step: the solved joints are kept off the end stops, a sudden switch to another arm posture is
     refused, and a step faster than the joint speed limit is scaled down (the arm slows, keeping its path).
Poses are (p, q): p = (x, y, z) in m, q = (x, y, z, w) unit quaternion, both in the world frame (yam_setup).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from teleop_rotation import q_angle, q_norm, q_step_toward
from yam_setup import Limits

Pose = tuple  # ((x, y, z), (qx, qy, qz, qw))
AXES = ("x", "y", "z")


def dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def clamp_box(p: Sequence[float], box: Sequence[float]) -> tuple[tuple, list[str]]:
    """p held inside box (x_min,x_max,y_min,y_max,z_min,z_max); the sides it was held at, e.g. ["x max"]."""
    out, sides = [], []
    for i, v in enumerate(p):
        lo, hi = box[2 * i], box[2 * i + 1]
        if v < lo:
            sides.append(f"{AXES[i]} min")
        elif v > hi:
            sides.append(f"{AXES[i]} max")
        out.append(min(max(v, lo), hi))
    return tuple(out), sides


def stretch_box(box: Sequence[float], p: Sequence[float]) -> tuple:
    """box widened just enough to contain p: an arm that starts outside its box may move back in, never further out."""
    b = list(box)
    for i, v in enumerate(p):
        b[2 * i], b[2 * i + 1] = min(b[2 * i], v), max(b[2 * i + 1], v)
    return tuple(b)


@dataclass
class TargetStep:
    target: Pose
    notes: list[str] = field(default_factory=list)
    glitch: str = ""  # non-empty: the request was refused as a jump; the arm should be let go


class TargetLimiter:
    def __init__(self, box: Sequence[float], lim: Limits):
        self.box0, self.lim = tuple(box), lim
        self.box = self.box0
        self.target: Pose | None = None
        self.prev_request: Pose | None = None
        self.vel = (0.0, 0.0, 0.0)  # the target's velocity, m/s
        self.w = 0.0  # its turning speed, rad/s

    def reset(self, tool_now: Pose) -> list[str]:
        """Start from where the tool is (at engagement)."""
        self.box = stretch_box(self.box0, tool_now[0])
        self.target, self.prev_request = (tuple(tool_now[0]), q_norm(tool_now[1])), None
        self.vel, self.w = (0.0, 0.0, 0.0), 0.0
        return [] if self.box == self.box0 else [f"the tool is outside its box at {_mm(tool_now[0])}; the box is stretched to it"]

    def step(self, request: Pose, tool_now: Pose) -> TargetStep:
        assert self.target is not None, "reset() first"
        lim, (p_t, q_t) = self.lim, self.target
        p_req, q_req = tuple(request[0]), q_norm(request[1])
        if self.prev_request is not None:
            jump, turn = dist(p_req, self.prev_request[0]), q_angle(self.prev_request[1], q_req)
            if jump > lim.jump or turn > lim.jump_rot:
                self.prev_request = None
                return TargetStep(self.target, glitch=f"the hand target jumped {jump * 1000:.0f} mm / {math.degrees(turn):.0f} deg in one step")
        self.prev_request = (p_req, q_req)
        notes = []
        p_c, sides = clamp_box(p_req, self.box)
        if sides:
            notes.append("at the box edge (" + ", ".join(sides) + ")")
        p_new = self._move(p_t, p_c, notes)
        q_new = self._turn(q_t, q_req)
        # Never run ahead of the arm: a step that takes the target further from the tool than `lag` is not taken.
        if dist(p_new, tool_now[0]) > lim.lag and dist(p_new, tool_now[0]) > dist(p_t, tool_now[0]):
            p_new, self.vel = p_t, (0.0, 0.0, 0.0)
            notes.append("waiting for the arm")
        if q_angle(q_new, tool_now[1]) > lim.lag_rot and q_angle(q_new, tool_now[1]) > q_angle(q_t, tool_now[1]):
            q_new, self.w = q_t, 0.0
            notes.append("waiting for the arm to turn")
        self.target = (p_new, q_new)
        return TargetStep(self.target, notes)

    def _move(self, p_t: tuple, p_c: tuple, notes: list[str]) -> tuple:
        """One step of the target toward p_c: the wanted velocity points at p_c, at most the tool speed and no faster
        than allows stopping there at the tool acceleration; the velocity changes by at most accel * dt."""
        lim, dt = self.lim, self.lim.dt
        d = dist(p_c, p_t)
        if d < 1e-9:
            self.vel = (0.0, 0.0, 0.0)
            return p_t
        speed = min(lim.tool_speed, math.sqrt(2.0 * lim.tool_accel * d))
        if lim.tool_speed * dt < d and speed >= lim.tool_speed:
            notes.append("speed limit")
        want = tuple((b - a) / d * speed for a, b in zip(p_t, p_c))
        dv = tuple(w - v for w, v in zip(want, self.vel))
        n = math.sqrt(sum(x * x for x in dv))
        k = 1.0 if n <= lim.tool_accel * dt else lim.tool_accel * dt / n
        vel = tuple(v + x * k for v, x in zip(self.vel, dv))
        step = tuple(v * dt for v in vel)
        if dist(step, (0.0, 0.0, 0.0)) >= d:  # this step reaches p_c: stop there
            self.vel = tuple((b - a) / dt for a, b in zip(p_t, p_c))
            return p_c
        self.vel = vel
        return tuple(a + s for a, s in zip(p_t, step))

    def _turn(self, q_t: tuple, q_req: tuple) -> tuple:
        """The same for the orientation, with the turning speed as one number."""
        lim, dt = self.lim, self.lim.dt
        a = q_angle(q_t, q_req)
        want = min(lim.tool_rot_speed, math.sqrt(2.0 * lim.tool_rot_accel * a))
        self.w = min(max(want, self.w - lim.tool_rot_accel * dt), self.w + lim.tool_rot_accel * dt)
        if self.w * dt >= a:
            self.w = a / dt
            return q_req
        return q_step_toward(q_t, q_req, self.w * dt)


@dataclass
class JointStep:
    q: list[float] | None  # None: refused, the arm keeps its last command
    notes: list[str] = field(default_factory=list)
    refused: str = ""


def joint_step(q_last: Sequence[float], q_ik: Sequence[float], ranges: Sequence[Sequence[float]], lim: Limits) -> JointStep:
    """The joint command for this step, from the last command and the IK answer (arm joints only).

    A joint already inside its margin (parked there by something else) may stay or move out, never further in."""
    notes, q = [], []
    for i, (v, (lo, hi)) in enumerate(zip(q_ik, ranges)):
        lo, hi = min(lo + lim.joint_margin, q_last[i]), max(hi - lim.joint_margin, q_last[i])
        if v < lo or v > hi:
            notes.append(f"joint {i + 1} at its end stop")
        q.append(min(max(v, lo), hi))
    d = [b - a for a, b in zip(q_last, q)]
    i_max = max(range(len(d)), key=lambda i: abs(d[i]))
    if abs(d[i_max]) > lim.joint_flip:
        return JointStep(None, notes, f"joint {i_max + 1} would swing {math.degrees(abs(d[i_max])):.0f} deg at once (another arm posture)")
    step = lim.joint_speed * lim.dt
    if abs(d[i_max]) > step:
        k = step / abs(d[i_max])
        q = [a + x * k for a, x in zip(q_last, d)]
        notes.append(f"slowed: joint {i_max + 1} at its speed limit")
    return JointStep(q, notes)


def joint_velocity(q_prev: Sequence[float] | None, q_new: Sequence[float], lim: Limits) -> list[float]:
    """The joint velocities (rad/s) that go with a position command: the change since the last command over one control
    step, the 6 arm joints; 0 for the gripper. All 0 when there is no last command, or when any joint changed faster than
    the joint speed limit allows (a jump in the commands: a stop where the arm is, a fault, the start), so that a
    velocity is only ever sent for a movement the limits made."""
    n = len(q_new)
    if q_prev is None:
        return [0.0] * n
    v = [(b - a) / lim.dt for a, b in zip(q_prev[:6], q_new[:6])]
    if any(abs(x) > lim.joint_speed * 1.05 + 1e-9 for x in v):
        return [0.0] * n
    return v + [0.0] * (n - 6)


def _mm(p: Sequence[float]) -> str:
    return "(" + ", ".join(f"{v * 1000:.0f}" for v in p) + ") mm"
