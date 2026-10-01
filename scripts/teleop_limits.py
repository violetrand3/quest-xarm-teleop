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

    def workspace_problem(self, p: Sequence[float], what: str) -> str | None:
        for k, ax in enumerate("xyz"):
            if not (self.workspace_min_m[k] <= p[k] <= self.workspace_max_m[k]):
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
