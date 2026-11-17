"""Bimanual YAM teleop step: Quest hands -> arm requests -> BimanualControl. Pure Python; no arm code here.

Per hand, every control step:
  - clutch (teleop_input.Engagement): hold the side grip (middle finger) to move the arm, release to stop;
    a grip already held, or input that comes back with the grip held, never engages;
  - while engaged, the tool follows the hand from where both were at engagement:
        p_tool = p_tool0 + scale * R * (p_hand - p_hand0)
        q_tool = exp(R * rotvec(q_hand * q_hand0^-1)) * q_tool0          (hand rotation 1:1)
    R maps the Quest's LOCAL axes to the world frame (yam_setup) for an operator whose forward is turned by
    operator_yaw from the headset's forward, exactly as on the xArm (0: headset worn or on the forehead);
  - the gripper follows the index trigger (pressed = closed: opening = 1 - trigger) once the trigger has come
    within PICKUP of the gripper's command, so a trigger that does not match the jaws never moves them
    (an arm holding an object does not drop it when the hand comes back with the trigger released). Losing the
    hand's input drops the pickup.
Everything that limits motion (box, speeds, jumps, joints, keeping clear) is in BimanualControl (yam_control).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

from teleop_input import ACTIVE, Engagement, InputView
from teleop_rotation import mat_vec, q_conj, q_from_rotvec, q_mul, q_norm, q_to_rotvec
from yam_control import BimanualControl, Request
from yam_input import GRIP_HOLD_MIN, GRIP_PRESSED_MIN, GRIP_RELEASED_MAX

R_WORLD_FROM_LOCAL = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))  # LOCAL -z (forward) -> +x, -x -> +y, +y -> +z
PICKUP = 0.1


def world_from_local(operator_yaw_deg: float) -> tuple:
    """R for an operator whose forward is turned by yaw (deg, counter-clockwise seen from above) from the headset's."""
    c, s = math.cos(math.radians(operator_yaw_deg)), math.sin(math.radians(operator_yaw_deg))
    ry_t = ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))
    return tuple(tuple(sum(R_WORLD_FROM_LOCAL[i][k] * ry_t[k][j] for k in range(3)) for j in range(3)) for i in range(3))


class HandMap:
    def __init__(self, scale: float, R: Sequence[Sequence[float]]):
        if not 0.0 < scale <= 1.5:
            raise ValueError("scale must be in (0, 1.5]")
        self.scale, self.R = scale, R
        self.p_hand0 = self.q_hand0 = self.tool0 = None

    def anchor(self, p_hand: Sequence[float], q_hand: Sequence[float], tool: tuple) -> None:
        self.p_hand0, self.q_hand0, self.tool0 = tuple(p_hand), q_norm(q_hand), (tuple(tool[0]), q_norm(tool[1]))

    def target(self, p_hand: Sequence[float], q_hand: Sequence[float]) -> tuple:
        d = mat_vec(self.R, [a - b for a, b in zip(p_hand, self.p_hand0)])
        p = tuple(t + self.scale * v for t, v in zip(self.tool0[0], d))
        r = mat_vec(self.R, q_to_rotvec(q_mul(q_norm(q_hand), q_conj(self.q_hand0))))
        return p, q_norm(q_mul(q_from_rotvec(r), self.tool0[1]))


@dataclass
class HandReport:
    clutch: str  # WAIT_RELEASE / ARMED / ACTIVE / FAULT_LATCHED
    arm: str  # the arm's state from BimanualControl
    gripper: str  # "following" / "waiting for the trigger to match" / "no input"
    notes: list[str] = field(default_factory=list)


class YamTeleop:
    def __init__(self, control: BimanualControl, scale: float, operator_yaw_deg: float, input_fresh_ms: float):
        self.control = control
        self.names = tuple(control.arms)
        R = world_from_local(operator_yaw_deg)
        self.map = {h: HandMap(scale, R) for h in self.names}
        self.eng = {h: Engagement(GRIP_RELEASED_MAX, GRIP_PRESSED_MIN, GRIP_HOLD_MIN, int(input_fresh_ms * 1e6)) for h in self.names}
        self.picked = {h: False for h in self.names}

    def tick(self, views: Mapping[str, InputView], now_ns: int) -> dict[str, HandReport]:
        requests: dict[str, Request] = {}
        notes: dict[str, list[str]] = {h: [] for h in self.names}
        grip_state: dict[str, str] = {}
        for h in self.names:
            e, v = self.eng[h], views.get(h)
            s = None if v is None else v.sample
            if e.state == ACTIVE:
                why, latch = e.active_problem(v, now_ns)
                if why is not None:
                    e.disengage(why, latch)
                    self.control.release(h)
                    notes[h].append(f"let go: {why}")
            elif v is not None and e.step_disengaged(v, now_ns) == "press":
                if s.q_xyzw is None:
                    e.refuse_engagement("no controller orientation")
                else:
                    said = self.control.engage(h)
                    notes[h] += said
                    refused = [n for n in said if n.startswith("not engaged")]
                    if refused:
                        e.refuse_engagement(refused[0])
                    else:
                        tool = self.control.pair.fk(h, self.control.q[h][:6])
                        e.engage(v, tool[0], tool[1], now_ns)
                        self.map[h].anchor(s.p_m, s.q_xyzw, tool)
                        notes[h].append("engaged")
            req = Request()
            if e.state == ACTIVE:
                req.pose = self.map[h].target(s.p_m, s.q_xyzw)
            req.grip, grip_state[h] = self._grip(h, v, now_ns)
            requests[h] = req

        reports = self.control.tick(requests)
        out = {}
        for h in self.names:
            r = reports[h]
            if self.eng[h].state == ACTIVE and not self.control.engaged[h]:  # the control let this arm go
                self.eng[h].disengage("; ".join(r.notes) or r.state, False)
            out[h] = HandReport(self.eng[h].state, r.state, grip_state[h], notes[h] + r.notes)
        return out

    def acknowledge(self) -> list[str]:
        """The operator's explicit acknowledgement of latched hand problems and of a stopped-arms fault (never
        engages anything: a hand must still be released and pressed again)."""
        out = [f"{h}: {self.eng[h].acknowledge()[1]}" for h in self.names]
        cleared = self.control.clear_fault()
        return out + ([cleared] if cleared else [])

    def _grip(self, h: str, v: InputView | None, now_ns: int) -> tuple[float | None, str]:
        e = self.eng[h]
        problem = "no input" if v is None else e.input_problem(v, now_ns, None)[0]
        trig = None if problem else v.sample.trigger
        if trig is None:
            self.picked[h] = False
            return None, "no input"
        want = 1.0 - min(max(trig, 0.0), 1.0)
        if not self.picked[h]:
            if abs(want - self.control.q[h][6]) > PICKUP:
                return None, "waiting for the trigger to match"
            self.picked[h] = True
        return want, "following"
