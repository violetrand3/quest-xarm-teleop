"""Target limits: fixed workspace, discontinuity rejection, and a velocity/acceleration limiter.

Order matters. A controller jump is rejected BEFORE the rate limiter sees it, so a reset or a
tracking glitch can never be turned into slow, ordinary-looking motion. The workspace is fixed
in the backend frame and does not move with a clutch engagement.

Time comes from the scheduler's monotonic clock (dt between ticks). Packets are not assumed to
be evenly spaced, and nothing is extrapolated.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import arm_kinematics as kin


def dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((a[k] - b[k]) ** 2 for k in range(3)))


CLAMP_INSET_M = 0.0002
EDGE_EPS_M = 1e-7  # floating-point slack at the workspace edge (0.0001 mm)


@dataclass(frozen=True)
class Limits:
    workspace_min_m: tuple
    workspace_max_m: tuple
    max_anchor_displacement_m: float
    max_controller_step_m: float
    max_controller_speed_m_s: float
    v_max_m_s: float
    a_max_m_s2: float
    max_target_feedback_error_m: float
    clamp_to_workspace: bool = False  # True: a request beyond the fixed workspace is held at its edge instead of faulting

    def clamp(self, p: Sequence[float]) -> tuple:
        """Hold a request just INSIDE the workspace (0.2 mm), so that the rate-limited target that follows
        it can never land a rounding error outside the edge."""
        return tuple(min(self.workspace_max_m[k] - CLAMP_INSET_M, max(self.workspace_min_m[k] + CLAMP_INSET_M, p[k])) for k in range(3))

    def workspace_problem(self, p: Sequence[float], what: str) -> str | None:
        for k, ax in enumerate("xyz"):
            if not (self.workspace_min_m[k] - EDGE_EPS_M <= p[k] <= self.workspace_max_m[k] + EDGE_EPS_M):
                return f"workspace_violation:{what} {ax}={p[k] * 1000:+.1f}mm outside [{self.workspace_min_m[k] * 1000:+.0f},{self.workspace_max_m[k] * 1000:+.0f}]"
        return None

    def controller_step_problem(self, prev_p: Sequence[float], prev_t_query_ns: int, p: Sequence[float], t_query_ns: int) -> str | None:
        """Jump between two consecutive accepted controller samples of one engagement."""
        d = dist(prev_p, p)
        if d > self.max_controller_step_m:
            return f"controller_discontinuity:step {d * 1000:.1f}mm > {self.max_controller_step_m * 1000:.0f}mm"
        dt = (t_query_ns - prev_t_query_ns) / 1e9  # both from the same device clock and app run
        if dt <= 0:
            return "controller_discontinuity:source time did not advance"
        if d / dt > self.max_controller_speed_m_s:
            return f"controller_discontinuity:speed {d / dt:.2f}m/s > {self.max_controller_speed_m_s:.2f}m/s"
        return None

    def target_problem(self, p_req: Sequence[float], p_anchor: Sequence[float]) -> str | None:
        d = dist(p_req, p_anchor)
        if d > self.max_anchor_displacement_m:
            return f"anchor_displacement_violation:{d * 1000:.1f}mm > {self.max_anchor_displacement_m * 1000:.0f}mm"
        return self.workspace_problem(p_req, "requested")

    def tracking_problem(self, p_lim: Sequence[float], p_feedback: Sequence[float]) -> str | None:
        e = dist(p_lim, p_feedback)
        if e > self.max_target_feedback_error_m:
            return f"target_feedback_error:{e * 1000:.1f}mm > {self.max_target_feedback_error_m * 1000:.1f}mm"
        return None


class RateLimiter:
    """Moves a limited target toward the requested one with |v| <= v_max and |dv/dt| <= a_max."""

    def __init__(self, v_max: float, a_max: float):
        self.v_max, self.a_max = float(v_max), float(a_max)
        self.p: tuple | None = None
        self.v = (0.0, 0.0, 0.0)

    def reset(self, p: Sequence[float]) -> None:
        self.p, self.v = tuple(float(x) for x in p), (0.0, 0.0, 0.0)

    def clear(self) -> None:
        self.p, self.v = None, (0.0, 0.0, 0.0)

    def confine(self, lo: Sequence[float], hi: Sequence[float]) -> tuple:
        """Keep the limited target inside [lo, hi]. While it chases a request that slides along a wall it
        can otherwise drift a few hundredths of a millimetre past it. An axis that touches a wall loses its
        outward speed."""
        assert self.p is not None
        p, v = list(self.p), list(self.v)
        for k in range(3):
            if p[k] > hi[k]:
                p[k], v[k] = hi[k], min(v[k], 0.0)
            elif p[k] < lo[k]:
                p[k], v[k] = lo[k], max(v[k], 0.0)
        self.p, self.v = tuple(p), (v[0], v[1], v[2])
        return self.p

    def step(self, p_req: Sequence[float], dt_s: float) -> tuple:
        assert self.p is not None, "limiter used without an engagement"
        if dt_s <= 0:
            return self.p
        d = [p_req[k] - self.p[k] for k in range(3)]
        n = math.sqrt(sum(x * x for x in d))
        # Speed that can still be braked to zero at the target after travelling one more step
        # (v*dt + v^2/(2a) <= n), never more than v_max or than needed to arrive this step.
        brake = self.a_max * (math.sqrt(dt_s * dt_s + 2.0 * n / self.a_max) - dt_s)
        speed = min(self.v_max, brake, n / dt_s) if n > 0 else 0.0
        want = [x / n * speed for x in d] if n > 0 else [0.0, 0.0, 0.0]
        dv = [want[k] - self.v[k] for k in range(3)]
        m, cap = math.sqrt(sum(x * x for x in dv)), self.a_max * dt_s
        if m > cap:
            dv = [x * cap / m for x in dv]
        v = [self.v[k] + dv[k] for k in range(3)]
        s = math.sqrt(sum(x * x for x in v))
        if s > self.v_max:
            v = [x * self.v_max / s for x in v]
        self.v = (v[0], v[1], v[2])
        self.p = tuple(self.p[k] + v[k] * dt_s for k in range(3))
        return self.p


class WristGuard:
    """Keeps the wrist away from its turning point and from the limits of joints 4 and 6, using the joint
    angles read every tick (2026-10-31: pointing straight down close to the base took joint 5 to 10 deg;
    joints 4 and 6 spun nearly a full turn each and joint 4 stopped the arm at its 360 deg limit, error 23,
    which only someone at the rig can clear).

    Joint 5 near 0 lines up axes 4 and 6, and the controller then spins both for a small change of the tool.
    Near the warning zone (joint 5 within j5_zone of 0, or joint 4 or 6 within j46_zone of +-360) each target is
    judged by the joint angles it would need, predicted from the measured ones with the arm's geometry
    (arm_kinematics: the xArm6 geometry): a target that keeps the arm out of the zone, or does not take it further in than the best
    target sent since the arm came near the zone, is sent. (Against the best, not the last: a tolerance against
    the last target adds up, and in a test a hand moving on steadily took joint 5 from 15 to 6 deg that way.)
    Otherwise the guard tries the target's position with the last orientation, the last position with the
    target's orientation, and then only those parts of the move along base x, y and z that do not take the wrist
    further in (sliding along the edge of the zone; never a direction the hand did not ask for), with and without
    the target's orientation. It holds the arm only when none of them is acceptable. So any
    move that helps is followed, and only the part of a move that would make the wrist worse is not.
    (The guard of 2026-10-31 followed only moves back toward the last pose outside the zone; a hand cannot retrace
    a pose, and on 2026-11-02 it held the arm for nine grip presses in a row.)
    Joint 4 or 6 closer than j46_stop to its limit anyway is a fault, raised before the controller's own limit."""

    LIMIT_DEG = 360.0  # joints 4 and 6
    NEAR = 1.5  # margin below which targets are predicted (above it a tick's move cannot reach the zone)
    SLACK = 0.01  # margin a target may lose against the last one sent (0.15 deg of joint 5): the prediction's own noise
    HINT_EVERY = 25  # ticks between hints while held (each hint predicts a handful of trial moves)

    def __init__(self, j5_zone_deg: float, j46_zone_deg: float, j46_stop_deg: float):
        self.j5_zone, self.j46_zone, self.j46_stop = j5_zone_deg, j46_zone_deg, j46_stop_deg
        self.last: tuple | None = None  # (p_m, q) of the last target sent
        self.best: float | None = None  # the highest predicted margin of a target sent since the arm came near the zone
        self.note = ""
        self.hint = ""
        self._hint_age = 0

    def engage(self, fb: tuple) -> None:
        """At an engagement the first target is where the arm is."""
        self.last, self.best = fb, None

    def margin(self, j: Sequence[float]) -> float:
        """Below 1: in the warning zone."""
        return min(abs(j[4]) / self.j5_zone, (self.LIMIT_DEG - abs(j[3])) / self.j46_zone, (self.LIMIT_DEG - abs(j[5])) / self.j46_zone)

    def worst(self, j: Sequence[float]) -> str:
        return min(((abs(j[4]) / self.j5_zone, f"joint 5 at {j[4]:.0f}deg, near the wrist's turning point"),
                    ((self.LIMIT_DEG - abs(j[3])) / self.j46_zone, f"joint 4 at {j[3]:.0f}deg, near its limit"),
                    ((self.LIMIT_DEG - abs(j[5])) / self.j46_zone, f"joint 6 at {j[5]:.0f}deg, near its limit")))[1]

    def predicted(self, j: Sequence[float], fb: tuple, t: tuple) -> float:
        """Margin of the target t = (p_m, q), predicted from the joints j measured at the pose fb = (p_m, q)."""
        dp = (t[0][0] - fb[0][0], t[0][1] - fb[0][1], t[0][2] - fb[0][2])
        return self.margin(kin.predict(j, dp, fb[1], t[1]))

    def _helpful_position(self, j: Sequence[float], fb: tuple, target: tuple) -> tuple:
        """The target's position with only the parts of the move (along base x, y, z) that do not lower the margin."""
        p0, q0 = self.last
        m0 = self.predicted(j, fb, self.last)
        p = list(p0)
        for k in range(3):
            d = target[0][k] - p0[k]
            if abs(d) < 1e-6:
                continue
            e = list(p0)
            e[k] += math.copysign(0.001, d)
            if self.predicted(j, fb, (tuple(e), q0)) >= m0:
                p[k] = target[0][k]
        return tuple(p)

    def _hint(self, j: Sequence[float], fb: tuple) -> str:
        """The simple move of the hand that helps the wrist most, found by predicting a few trial moves."""
        p, q = fb
        trials = []
        if q is not None:
            z = kin.q_to_matrix(q)
            z = (z[0][2], z[1][2], z[2][2])  # the gripper's axis
            k = (-z[1], z[0], 0.0)  # z x down: tilting about it brings the gripper toward straight down
            n = math.hypot(k[0], k[1])
            if n > 0.03:
                for a, what in ((0.17, "tilt the gripper back toward straight down"), (-0.17, "tilt the gripper further from straight down")):
                    trials.append((what, p, _q_turn(q, (k[0] / n * a, k[1] / n * a, 0.0))))
            if not self.worst(j).startswith("joint 5"):
                for a in (0.35, -0.35):  # +a about the gripper's axis is clockwise seen from above while it points down
                    cw = (a > 0) == (z[2] < 0)
                    trials.append((f"turn the gripper {'clockwise' if cw else 'counter-clockwise'} (seen from above)", p, _q_turn(q, (z[0] * a, z[1] * a, z[2] * a))))
        r = math.hypot(p[0], p[1])
        if r > 1e-3:
            for d, what in ((0.03, "move the gripper away from the robot's base"), (-0.03, "move the gripper toward the robot's base")):
                trials.append((what, (p[0] + p[0] / r * d, p[1] + p[1] / r * d, p[2]), q))
        trials += [("move the gripper up", (p[0], p[1], p[2] + 0.03), q), ("move the gripper down", (p[0], p[1], p[2] - 0.03), q)]
        now = self.margin(j)
        best = max(trials, key=lambda t: self.predicted(j, fb, (t[1], t[2])))
        return best[0] if self.predicted(j, fb, (best[1], best[2])) > now + 0.02 else "move the hand back the way it came"

    def check(self, joints_deg: Sequence[float] | None, fb: tuple, p_m: Sequence[float], q: Sequence[float] | None) -> tuple:
        """(fault reason or None, the target (p_m, q) to send, or None to hold the arm), for the target (p_m, q),
        given the joints and the pose fb = (p_m, q) read in the same tick. A target that is sent is remembered."""
        if joints_deg is None:
            return "wrist_guard:no joint angles from the backend", None
        j = joints_deg
        for k in (3, 5):
            if self.LIMIT_DEG - abs(j[k]) < self.j46_stop:
                return f"wrist_limit:joint {k + 1} at {j[k]:.1f}deg, within {self.j46_stop:g}deg of its limit", None
        target = (tuple(p_m), None if q is None else tuple(q))
        chosen = target
        if self.margin(j) >= self.NEAR or self.last is None:
            self.best = None
        else:
            if self.best is None:
                self.best = self.predicted(j, fb, self.last)
            floor = min(1.0, self.best) - self.SLACK
            parts = [t for t in (target, (target[0], self.last[1]), (self.last[0], target[1])) if t != self.last]
            parts += [lambda: (self._helpful_position(j, fb, target), target[1]),  # computed only when needed
                      lambda: (self._helpful_position(j, fb, target), self.last[1])]
            for t in parts:  # the last target itself is a hold
                t = t() if callable(t) else t
                if t == self.last:
                    continue
                m = self.predicted(j, fb, t)
                if m >= floor:
                    chosen, self.best = t, max(self.best, m)
                    break
            else:
                chosen = None
        if chosen is target and self.margin(j) >= 1.0:
            self.note = ""
        elif chosen is None:
            if not self.note.startswith("held") or self._hint_age >= self.HINT_EVERY:
                self.hint, self._hint_age = self._hint(j, fb), 0
            self._hint_age += 1
            self.note = f"held: {self.worst(j)}; try: {self.hint}"
        else:
            self.note = ("following" if chosen is target else "following part of the move") + f": {self.worst(j)}"
        if chosen is not None:
            self.last = chosen
        return None, chosen


def _q_turn(q: Sequence[float], r: Sequence[float]) -> tuple:
    """q turned by the rotation vector r (base frame)."""
    a = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    s = math.sin(a / 2.0) / a if a > 1e-12 else 0.5
    t = (r[0] * s, r[1] * s, r[2] * s, math.cos(a / 2.0))
    x, y, z, w = q
    return (t[3] * x + t[0] * w + t[1] * z - t[2] * y, t[3] * y - t[0] * z + t[1] * w + t[2] * x,
            t[3] * z + t[0] * y - t[1] * x + t[2] * w, t[3] * w - t[0] * x - t[1] * y - t[2] * z)  # fmt: skip
