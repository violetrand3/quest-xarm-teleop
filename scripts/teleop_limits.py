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


def _q_deg(a: Sequence[float] | None, b: Sequence[float] | None) -> float:
    if a is None or b is None:
        return 0.0
    d = min(1.0, abs(sum(a[k] * b[k] for k in range(4))))
    return math.degrees(2.0 * math.acos(d))


class WristGuard:
    """Keeps the wrist away from its turning point and from the limits of joints 4 and 6, using the joint
    angles read every tick (2026-10-31: pointing straight down close to the base took joint 5 to 10 deg;
    joints 4 and 6 spun nearly a full turn each and joint 4 stopped the arm at its 360 deg limit, error 23,
    which only someone at the rig can clear).

    Joint 5 near 0 lines up axes 4 and 6, and the controller then spins both for a small change of the tool.
    This program does not know the arm's geometry, so it cannot predict joint angles; it reacts to the ones it
    reads. In the warning zone (joint 5 within j5_zone of 0, or joint 4 or 6 within j46_zone of +-360) a target
    is sent only if it leads back toward the last pose MEASURED outside the zone (the arm reaches each target a
    little after it is sent, so the last target sent from outside can already be inside); otherwise the arm is
    held where it is. Joint 4 or 6 closer than j46_stop to its limit anyway is a fault, raised before the
    controller's own limit."""

    LIMIT_DEG = 360.0  # joints 4 and 6
    MM_PER_DEG = 2.0  # how a turn of the tool counts against a movement when comparing poses

    def __init__(self, j5_zone_deg: float, j46_zone_deg: float, j46_stop_deg: float):
        self.j5_zone, self.j46_zone, self.j46_stop = j5_zone_deg, j46_zone_deg, j46_stop_deg
        self.safe: tuple | None = None  # (p_m, q) of the arm when its joints were last read outside the zone
        self.last: tuple | None = None  # (p_m, q) of the last target sent
        self.note = ""

    def margin(self, j: Sequence[float]) -> float:
        """Below 1: in the warning zone."""
        return min(abs(j[4]) / self.j5_zone, (self.LIMIT_DEG - abs(j[3])) / self.j46_zone, (self.LIMIT_DEG - abs(j[5])) / self.j46_zone)

    def _d(self, a: tuple, b: tuple) -> float:
        return dist(a[0], b[0]) * 1000.0 + self.MM_PER_DEG * _q_deg(a[1], b[1])

    def check(self, joints_deg: Sequence[float] | None, fb: tuple, p_m: Sequence[float], q: Sequence[float] | None) -> tuple:
        """(fault reason or None, send this target?) for the target (p_m, q), given the joints and the pose fb = (p_m, q)
        read in the same tick. A target that is sent is remembered."""
        if joints_deg is None:
            return "wrist_guard:no joint angles from the backend", False
        j = joints_deg
        for k in (3, 5):
            if self.LIMIT_DEG - abs(j[k]) < self.j46_stop:
                return f"wrist_limit:joint {k + 1} at {j[k]:.1f}deg, within {self.j46_stop:g}deg of its limit", False
        target = (tuple(p_m), None if q is None else tuple(q))
        m = self.margin(j)
        if m >= 1.0:
            send, self.note, self.safe = True, "", (tuple(fb[0]), None if fb[1] is None else tuple(fb[1]))
        elif self.safe is None or self.last is None:
            send, self.note = True, ""  # entered the zone before any target was sent outside it: nothing to lead back to
        else:
            send = self._d(target, self.safe) < self._d(self.last, self.safe) - 1e-6
            worst = min(((abs(j[4]) / self.j5_zone, f"joint 5 at {j[4]:.0f}deg, near the wrist's turning point"),
                         ((self.LIMIT_DEG - abs(j[3])) / self.j46_zone, f"joint 4 at {j[3]:.0f}deg, near its limit"),
                         ((self.LIMIT_DEG - abs(j[5])) / self.j46_zone, f"joint 6 at {j[5]:.0f}deg, near its limit")))[1]
            self.note = ("leading back: " if send else "held: ") + worst
        if send:
            self.last = target
        return None, send
