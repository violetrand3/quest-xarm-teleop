"""Bimanual YAM teleop step: Quest hands -> arm requests -> BimanualControl. Pure Python; no arm code here.

Per hand, every control step:
  - clutch (teleop_input.Engagement): hold the side grip (middle finger) to move the arm, release to stop;
    a grip already held, or input that comes back with the grip held, never engages;
  - while engaged, the tool follows the hand from where both were at engagement:
        p_tool = p_tool0 + scale * R * (p_hand - p_hand0)
        q_tool = exp(R * rotvec(q_hand * q_hand0^-1)) * q_tool0          (hand rotation 1:1)
    R maps the Quest's LOCAL axes to the world frame (yam_setup) for an operator whose forward is turned by
    operator_yaw from the headset's forward, exactly as on the xArm (0: headset worn or on the forehead);
  - while engaged (only then, as on the xArm), the gripper follows the index trigger (pressed = closed:
    opening = 1 - trigger), at its speed limit. Once the operator has moved a gripper with the trigger, it follows
    again after a release only when the trigger has come within PICKUP of it, so an arm holding something does not
    drop it when the hand comes back with the trigger released. A gripper the operator has not moved yet in this
    session (where the start-up or the last lowering to rest left it) follows at once: at 40% open with the trigger
    released, it waited for a 60% press that nobody knew about (2026-11-17).
  - a short link stall (input older than input_fresh_ms: the operator's PC to the lab host) lets the arm go, as
    any input problem does. Unlike the xArm, it then RESUMES BY ITSELF when, within RESUME_S, fresh input comes back
    from the same headset session and continuity (nothing reset on the headset) with the side grip held: it
    re-anchors at the hand and the tool where they are then, so the arm never jumps to make up for the stall.
    Anything else (a release, a longer stall, any other problem) needs the usual release and press. Measured
    2026-11-17: stalls every ~10 s, 0.4-2 s long, each one forcing a re-grip.
Everything that limits motion (box, speeds, jumps, joints, keeping clear) is in BimanualControl (yam_control).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

from teleop_input import ACTIVE, ARMED, Engagement, InputView
from teleop_rotation import mat_vec, q_conj, q_from_rotvec, q_mul, q_norm, q_to_rotvec
from yam_control import BimanualControl, Request
from yam_input import GRIP_HOLD_MIN, GRIP_PRESSED_MIN, GRIP_RELEASED_MAX

R_WORLD_FROM_LOCAL = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))  # LOCAL -z (forward) -> +x, -x -> +y, +y -> +z
PICKUP = 0.1
RESUME_S = 2.0  # a stall longer than this needs the usual release and press
RESUMABLE = ("input_stale",)


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
        self.operator_set = {h: False for h in self.names}  # this gripper was moved by the operator's trigger
        self.stalled: dict[str, tuple | None] = {h: None for h in self.names}  # (deadline_ns, identity, continuity_gen)
        self._picked_before_stall: dict[str, bool] = {}

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
                    ref = e.ref
                    e.disengage(why, latch)
                    self.control.release(h)
                    if why in RESUMABLE and not latch:
                        self.stalled[h] = (now_ns + int(RESUME_S * 1e9), ref.identity, ref.continuity_gen)
                        self._picked_before_stall[h] = self.picked[h]
                        notes[h].append(f"paused: {why} (link stall); keep holding the side grip, it resumes by itself")
                    else:
                        notes[h].append(f"let go: {why}")
            elif self.stalled[h] is not None and v is not None:
                self._try_resume(h, v, now_ns, notes[h])
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
            extra = []
            if self.eng[h].state == "FAULT_LATCHED":
                extra = [f"latched ({self.eng[h].blocked.replace('acknowledgement required: ', '')}): press Enter after checking the headset's view"]
            clutch = "STALLED" if self.stalled[h] is not None else self.eng[h].state
            out[h] = HandReport(clutch, r.state, grip_state[h], notes[h] + r.notes + extra)
        return out

    def acknowledge(self) -> list[str]:
        """The operator's explicit acknowledgement of latched hand problems and of a stopped-arms fault (never
        engages anything: a hand must still be released and pressed again)."""
        out = [f"{h}: {self.eng[h].acknowledge()[1]}" for h in self.names]
        cleared = self.control.clear_fault()
        return out + ([cleared] if cleared else [])

    def _try_resume(self, h: str, v: InputView, now_ns: int, notes: list[str]) -> None:
        """After a link stall: re-engage once fresh input from the same headset session and continuity comes back
        with the side grip held, within RESUME_S; otherwise fall back to the usual release and press."""
        e, (deadline, identity, gen) = self.eng[h], self.stalled[h]
        if now_ns > deadline:
            self.stalled[h] = None
            notes.append(f"the stall lasted over {RESUME_S:g} s: release the side grip and press it again")
            return
        why, latch = e.input_problem(v, now_ns, (identity, gen))
        if latch:
            self.stalled[h] = None
            e.latch(why)
            return
        if why == "input_stale":
            return  # still stalled: wait
        s = v.sample
        if why is not None or s.grip < e.hold_min or s.q_xyzw is None:
            self.stalled[h] = None  # released, or something else changed: the usual release and press
            notes.append(f"not resumed ({why or 'side grip released'}): release the side grip and press it again")
            return
        said = self.control.engage(h)
        if any(n.startswith("not engaged") for n in said):
            self.stalled[h] = None
            notes.extend(said)
            return
        tool = self.control.pair.fk(h, self.control.q[h][:6])
        e.state = ARMED  # the stall stands in for the release; re-anchored where hand and tool are now
        e.engage(v, tool[0], tool[1], now_ns)
        self.map[h].anchor(s.p_m, s.q_xyzw, tool)
        self.stalled[h] = None
        self.picked[h] = self._picked_before_stall.pop(h, False)  # the gripper carries on following the trigger
        notes.extend(said + ["resumed after the link stall"])

    def _grip(self, h: str, v: InputView | None, now_ns: int) -> tuple[float | None, str]:
        e = self.eng[h]
        if e.state != ACTIVE:  # as on the xArm: the trigger works only while the side grip holds the arm
            self.picked[h] = False
            return None, "holds (the trigger works while the side grip is held)"
        problem = "no input" if v is None else e.input_problem(v, now_ns, None)[0]
        trig = None if problem else v.sample.trigger
        if trig is None:
            self.picked[h] = False
            return None, "no input"
        want = 1.0 - min(max(trig, 0.0), 1.0)
        if not self.picked[h]:
            have = self.control.q[h][6]
            if self.operator_set[h] and abs(want - have) > PICKUP:
                return None, (f"waiting for the trigger to match: the gripper is {have * 100:.0f}% open, "
                              f"the trigger asks {want * 100:.0f}%; {'press' if want > have else 'release'} it to {(1 - have) * 100:.0f}%")
            self.picked[h] = True
        self.operator_set[h] = True
        return want, "following"
