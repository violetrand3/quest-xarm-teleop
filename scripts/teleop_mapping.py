"""Controller-to-tool mapping: anchored translation with orientation held, and the gripper preview.

    p_target = p_tool_at_engagement + scale * R * (p_controller - p_controller_at_engagement)

R maps a controller displacement expressed in OpenXR LOCAL axes into the backend's Cartesian
frame. R is configuration. It is validated as a proper rotation and nothing else: it is never
derived from headset orientation, from where things stand in the room, or from the name
"LOCAL". The example R shipped with the dry run is an arbitrary illustration and is not
approved for live motion; its signs must be checked during commissioning.

Controller orientation is not used at all. The tool orientation captured at engagement is
passed back unchanged.

Gripper (custom gripctl driver, not the stock xArm gripper): the index trigger (0 = released,
1 = fully pressed) maps to percent closed for `POST /api/do/grip {"value": pct}` (0 = open,
100 = closed). A missing trigger stays missing. Following starts only after the trigger-derived
value has come within a tolerance of the gripper's reported position (pickup), so a trigger
that does not match the jaws cannot cause a jump.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

from teleop_backend import GripperProposal

GRIPPER_OPEN_TICKS = 3033  # actuator.py:44-45
GRIPPER_CLOSED_TICKS = 1181
GRIPPER_SPAN_TICKS = GRIPPER_OPEN_TICKS - GRIPPER_CLOSED_TICKS  # 1852, stepper.py:117


class ConfigError(ValueError):
    pass


def validate_rotation(R: Any, tol: float = 1e-6) -> tuple:
    """Return R as a 3x3 tuple if it is a proper rotation (orthonormal, det = +1), else raise."""
    try:
        m = tuple(tuple(float(v) for v in row) for row in R)
    except (TypeError, ValueError):
        raise ConfigError("R must be a 3x3 matrix of numbers") from None
    if len(m) != 3 or any(len(r) != 3 for r in m) or not all(math.isfinite(v) for r in m for v in r):
        raise ConfigError("R must be a finite 3x3 matrix")
    for i in range(3):
        for j in range(3):
            dot = sum(m[i][k] * m[j][k] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > tol:
                raise ConfigError("R is not orthonormal (rows must be unit length and mutually perpendicular)")
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))  # fmt: skip
    if abs(det - 1.0) > tol:
        raise ConfigError(f"R is not a proper rotation (det = {det:+.6f}; a reflection is not allowed)")
    return m


class AnchoredTranslation:
    def __init__(self, scale: float, R_backend_from_local: Any):
        if not (isinstance(scale, (int, float)) and math.isfinite(scale) and 0.0 < scale <= 1.0):
            raise ConfigError("scale must be in (0, 1]")
        self.scale = float(scale)
        self.R = validate_rotation(R_backend_from_local)

    def target(self, p_tool0_m: Sequence[float], p_ctrl0_m: Sequence[float], p_ctrl_m: Sequence[float]) -> tuple:
        d = [p_ctrl_m[k] - p_ctrl0_m[k] for k in range(3)]
        return tuple(p_tool0_m[i] + self.scale * sum(self.R[i][k] * d[k] for k in range(3)) for i in range(3))


def trigger_to_pct_closed(trigger: float | None, open_pct: float = 0.0, closed_pct: float = 100.0) -> float | None:
    """0..1 trigger -> open_pct..closed_pct (% closed, default 0..100). None (or a non-finite /
    out-of-range value) stays None. A narrower range keeps the jaws off their end stops."""
    if trigger is None or isinstance(trigger, bool) or not isinstance(trigger, (int, float)):
        return None
    if not math.isfinite(trigger) or trigger < -1e-6 or trigger > 1.0 + 1e-6:
        return None
    return open_pct + (closed_pct - open_pct) * min(1.0, max(0.0, float(trigger)))


def pct_closed_to_ticks(pct: float) -> int:
    return int(round(GRIPPER_OPEN_TICKS - pct / 100.0 * GRIPPER_SPAN_TICKS))


class GripperFollower:
    """Preview only. Produces proposals; whoever receives them decides nothing here is executed."""

    def __init__(self, pickup_tolerance_pct: float, max_rate_pct_s: float, open_pct: float = 0.0, closed_pct: float = 100.0):
        if not (0.0 <= open_pct < closed_pct <= 100.0):
            raise ConfigError("need 0 <= open_pct < closed_pct <= 100")
        self.tol, self.rate = float(pickup_tolerance_pct), float(max_rate_pct_s)
        self.open_pct, self.closed_pct = float(open_pct), float(closed_pct)
        self.following = False
        self._last: float | None = None

    def release(self) -> None:
        """Arm disengaged or input lost: stop following. No opening or closing request is made."""
        self.following, self._last = False, None

    def update(self, trigger: float | None, feedback_pct: float | None, dt_s: float) -> tuple:
        """-> (status, GripperProposal | None)."""
        want = trigger_to_pct_closed(trigger, self.open_pct, self.closed_pct)
        if want is None:
            self.release()
            return "trigger_missing", None
        if not self.following:
            if feedback_pct is None:
                return "no_gripper_feedback", None
            if abs(want - feedback_pct) > self.tol:
                return "waiting_pickup", None
            self.following, self._last = True, float(feedback_pct)
        step = self.rate * max(0.0, dt_s)
        pct = min(self._last + step, max(self._last - step, want))  # type: ignore[operator]
        self._last = pct
        return "following", GripperProposal(round(pct, 2), pct_closed_to_ticks(pct), float(trigger))  # type: ignore[arg-type]
