"""Bimanual YAM: one control step for both arms. Pure Python; the arms are reached through `pair`.

`pair` is the arm layer (yam_sim.SimYamPair today; the real arms later) and provides:
    names                          the arms, e.g. ("left", "right")
    joint_ranges(name)             [(lo, hi)] of the 6 arm joints, rad
    joints(name)                   measured [6 joints + gripper opening 0..1]
    fk(name, q6)                   tool (grasp point) pose in the world frame
    solve(name, pose, q6_init)     (q6, position error m, rotation error rad): the IK, from q6_init
    clearance_problems(q_by_name)  {what: distance m} for every pair of parts closer than allowed
    command(name, q7)              6 joints + gripper opening
Every step every arm is commanded: an arm that is idle or held gets its last command again, so it stays put.

Per step, for each engaged arm with a request: TargetLimiter -> IK -> joint_step (yam_limits). Then the
proposed joints of both arms are checked together: a move that brings any two parts (the two arms, an arm and
itself, an arm and the table) closer than allowed, and closer than they already are, is not made; the other
arm still moves if its own move is fine. A request missing for an engaged arm (input lost) lets that arm go.
A joint measured further than track_err from its command (real arms) stops both arms where they are, latched.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from yam_limits import TargetLimiter, dist, joint_step
from yam_setup import ArmSetup, Limits

GRIP_KEEP = None  # a request's grip of None keeps the gripper's current command


@dataclass
class Request:
    pose: Any = None  # ((x, y, z), (qx, qy, qz, qw)) in the world frame, or None (no input this step)
    grip: float | None = GRIP_KEEP  # gripper opening 0..1


@dataclass
class ArmReport:
    state: str  # "idle" | "following" | "held" | "let go" | "fault" | "parking"
    q: list[float]  # what was commanded this step (6 joints + gripper)
    target: Any = None
    notes: list[str] = field(default_factory=list)
    reach_err_m: float = 0.0


class BimanualControl:
    def __init__(self, pair: Any, arms: Sequence[ArmSetup], lim: Limits):
        self.pair, self.lim = pair, lim
        self.arms = {a.name: a for a in arms}
        assert set(self.arms) == set(pair.names), "the arm layer and the setup name different arms"
        self.limiter = {n: TargetLimiter(a.box, lim) for n, a in self.arms.items()}
        self.q = {n: list(pair.joints(n)) for n in self.arms}  # last command per arm: 6 joints + gripper
        self.engaged = {n: False for n in self.arms}
        self.fault = ""
        self._recent = {n: collections.deque([list(self.q[n][:6])], maxlen=max(2, round(0.5 / lim.dt))) for n in self.arms}

    # --- clutch ---------------------------------------------------------------------------------------------
    def engage(self, name: str) -> list[str]:
        if self.fault:
            return [f"not engaged: {self.fault}"]
        self.engaged[name] = True
        return self.limiter[name].reset(self.pair.fk(name, self.q[name][:6]))

    def release(self, name: str) -> None:
        self.engaged[name] = False

    # --- one step -------------------------------------------------------------------------------------------
    def tick(self, requests: Mapping[str, Request]) -> dict[str, ArmReport]:
        if not self.fault:
            self._check_tracking()
        if self.fault:
            for n in self.arms:
                self.pair.command(n, self.q[n])
            return {n: ArmReport("fault", list(self.q[n]), notes=[self.fault]) for n in self.arms}

        reports: dict[str, ArmReport] = {}
        proposal = {n: list(self.q[n]) for n in self.arms}
        for n in self.arms:
            req = requests.get(n)
            if req is not None and req.grip is not None:  # the gripper follows the trigger, engaged or not
                g = min(max(req.grip, 0.0), 1.0)
                step = self.lim.grip_speed * self.lim.dt
                proposal[n][6] = self.q[n][6] + max(-step, min(step, g - self.q[n][6]))
            if not self.engaged[n]:
                reports[n] = ArmReport("idle", proposal[n])
                continue
            if req is None or req.pose is None:
                self.engaged[n] = False
                reports[n] = ArmReport("let go", proposal[n], notes=["no hand input: the arm is let go and stays where it is"])
                continue
            q6 = self.q[n][:6]
            ts = self.limiter[n].step(req.pose, self.pair.fk(n, q6))
            if ts.glitch:
                self.engaged[n] = False
                reports[n] = ArmReport("let go", proposal[n], notes=[ts.glitch + ": the arm is let go; release and grip again"])
                continue
            q_ik, e_pos, e_rot = self.pair.solve(n, ts.target, q6)
            notes = list(ts.notes)
            if e_pos > self.lim.reach_tol or e_rot > self.lim.reach_tol_rot:
                notes.append(f"out of reach by {e_pos * 1000:.0f} mm: following as far as it goes")
            js = joint_step(q6, q_ik, self.pair.joint_ranges(n), self.lim)
            notes += js.notes
            if js.q is None:
                reports[n] = ArmReport("held", proposal[n], ts.target, notes + [js.refused], e_pos)
                continue
            proposal[n][:6] = js.q
            reports[n] = ArmReport("following", proposal[n], ts.target, notes, e_pos)

        self._keep_clear(proposal, reports)
        for n in self.arms:
            self.q[n] = list(reports[n].q)
            self.pair.command(n, self.q[n])
            self._recent[n].append(list(self.q[n][:6]))
        return reports

    # --- lowering to rest (every way a session ends) ---------------------------------------------------------
    def park_tick(self, rest: Mapping[str, Sequence[float]], joint_speed: float, grip_max: float | None = None) -> tuple[bool, dict[str, str]]:
        """One step of lowering both arms to their rest poses (6 joints each). A gripper more open than grip_max is
        closed to it at the gripper speed limit (None: grippers stay as they are).

        Both arms are let go first. Each arm's joints move straight toward rest, the largest change limited to
        joint_speed * dt, under the same rule as teleop: no step may bring parts closer than allowed and closer
        than they are. An arm whose step is refused waits while the other moves (usually out of its way).
        Returns (both at rest, {arm: what it did})."""
        for n in self.arms:
            self.engaged[n] = False
        step = joint_speed * self.lim.dt
        proposal, said = {}, {}
        for n in self.arms:
            q = list(self.q[n])
            d = [r - a for a, r in zip(q[:6], rest[n])]
            m = max(abs(x) for x in d)
            k = 1.0 if m <= step else step / m
            q[:6] = [a + x * k for a, x in zip(q[:6], d)]
            if grip_max is not None and q[6] > grip_max:
                q[6] = max(grip_max, q[6] - self.lim.grip_speed * self.lim.dt)
            proposal[n] = q
            said[n] = "lowering"
        reports = {n: ArmReport("parking", list(proposal[n])) for n in self.arms}
        self._keep_clear(proposal, reports)
        for n in self.arms:
            self.q[n] = list(reports[n].q)
            self.pair.command(n, self.q[n])
            if reports[n].state == "held":
                said[n] = "waiting: " + "; ".join(reports[n].notes)
            elif max(abs(a - r) for a, r in zip(self.q[n][:6], rest[n])) < 1e-6 and (grip_max is None or self.q[n][6] <= grip_max + 1e-9):
                said[n] = "at rest"
        return all(s == "at rest" for s in said.values()), said

    # --- checks ---------------------------------------------------------------------------------------------
    def _keep_clear(self, proposal: dict, reports: dict) -> None:
        """Moves that would bring parts closer than allowed (and closer than now) are not made."""
        now = self.pair.clearance_problems(self.q)
        moving = [n for n in self.arms if proposal[n][:6] != self.q[n][:6]]

        def worse(q_by_name: Mapping[str, Sequence[float]]) -> list[str]:
            new = self.pair.clearance_problems(q_by_name)
            return [f"{what} {d * 1000:.0f} mm" for what, d in new.items() if what not in now or d < now[what] - 1e-4]

        for n in moving:  # each arm's own move, against the other arm where it is
            bad = worse({m: (proposal[m] if m == n else self.q[m]) for m in self.arms})
            if bad:
                self._hold(n, reports, "too close: " + "; ".join(bad))
        if len([n for n in moving if reports[n].state != "held"]) > 1:
            bad = worse({m: (reports[m].q) for m in self.arms})
            if bad:  # each move alone is fine, both together are not
                for n in moving:
                    self._hold(n, reports, "too close: " + "; ".join(bad))

    def _hold(self, n: str, reports: dict, why: str) -> None:
        r = reports[n]
        r.q[:6] = self.q[n][:6]
        r.state, r.notes = "held", r.notes + [why]

    def allowed_tracking_error(self, n: str) -> list[float]:
        """Per joint: track_err plus track_lag_s of the fastest commanded speed in the last 0.5 s."""
        h = list(self._recent[n])
        fastest = [max((abs(b[i] - a[i]) for a, b in zip(h, h[1:])), default=0.0) / self.lim.dt for i in range(6)]
        return [self.lim.track_err + self.lim.track_lag_s * v for v in fastest]

    def clear_fault(self) -> str:
        """The operator's acknowledgement of a tracking fault: the arms keep holding where they were stopped; the
        hands must be released and pressed again to move them."""
        if not self.fault:
            return ""
        was, self.fault = self.fault, ""
        for n in self.arms:
            self.q[n][:6] = list(self.pair.joints(n))[:6]
            self._recent[n].clear()
            self._recent[n].append(list(self.q[n][:6]))
        return f"fault cleared ({was.split(':')[0]})"

    def _check_tracking(self) -> None:
        for n in self.arms:
            meas = self.pair.joints(n)
            allowed = self.allowed_tracking_error(n)
            for i in range(6):
                if abs(meas[i] - self.q[n][i]) > allowed[i]:
                    self.fault = (f"{n} arm joint {i + 1} is {abs(meas[i] - self.q[n][i]):.2f} rad from its command "
                                  f"(allowed {allowed[i]:.2f} at its recent speed): both arms stopped where they are; Enter clears it")
                    for m in self.arms:  # stop where they are
                        self.q[m][:6] = list(self.pair.joints(m))[:6]
                        self.engaged[m] = False
                    return


def tool_gap(pair: Any, name: str, q6: Sequence[float], target: Any) -> float:
    """Distance from the tool to a target, m (for reports and tests)."""
    return dist(pair.fk(name, q6)[0], target[0])
