#!/usr/bin/env python3
"""Bimanual YAM, SIMULATION ONLY: drives the two simulated arms through scripted requests and checks that each
safety rule does what it should. No hardware is opened (yam_sim). Run on the lab host in I2RT's environment:

    /home/dev/yam/i2rt/.venv/bin/python scripts/yam_sim_check.py

Prints one line per scenario (PASS/FAIL and the numbers behind it) and exits 1 if any failed.
"""

from __future__ import annotations

import dataclasses
import math
import sys
import time
from typing import Callable

from teleop_rotation import q_angle, q_from_rotvec, q_mul
from yam_control import BimanualControl, Request
from yam_limits import dist
from yam_setup import ARMS, LEFT, Limits

WORK = [0.0, 1.0, 1.0, 0.0, 0.0, 0.0]  # elbow bent, gripper pointing forward: tool 0.38 m ahead, 0.40 m up
DOWN = (0.0, 1.0, 0.0, 0.0)  # gripper pointing straight down (its approach axis along -z)
OPEN = 1.0
STEP_MS: list = []  # every control step of every scenario


class Run:
    def __init__(self, pair, arms=ARMS, lim: Limits = Limits(), start=WORK):
        self.pair, self.lim = pair, lim
        for n in pair.names:
            q = start
            if callable(start):  # a tool pose per arm, reached with the IK from WORK (set-up only)
                q, goal = WORK, start(pair.fk(n, WORK))
                for _ in range(50):  # the IK takes small steps; a set-up jump needs many
                    q, e, er = pair.solve(n, goal, q)
                    if e < 1e-4 and er < 1e-3:
                        break
                assert e < 1e-3 and er < 1e-2, f"set-up pose out of reach ({e * 1000:.0f} mm, {math.degrees(er):.0f} deg)"
            pair.place(n, list(q) + [OPEN])
        self.c = BimanualControl(pair, arms, lim)
        for n in pair.names:
            self.c.engage(n)
        self.start = {n: pair.fk(n, self.c.q[n][:6]) for n in pair.names}
        self.reports, self.ms, self.problems = [], [], []

    def tool(self, n):
        return self.pair.fk(n, self.c.q[n][:6])

    def go(self, steps: int, request: Callable[[int], dict]) -> None:
        for k in range(steps):
            req = request(k)
            reqs = {n: req.get(n, Request(self.tool(n))) for n in self.pair.names}  # unnamed arms: asked to stay
            t = time.perf_counter()
            self.reports.append(self.c.tick(reqs))
            self.ms.append((time.perf_counter() - t) * 1000)
            STEP_MS.append(self.ms[-1])
            self.problems.append(self.pair.clearance_problems(self.c.q))

    def notes(self, n) -> set:
        return {x.split(":")[0].split(" (")[0] for r in self.reports for x in r[n].notes}

    def max_step(self, n) -> float:
        """Largest tool movement in one step, m."""
        poses = [self.start[n]] + [self.pair.fk(n, r[n].q[:6]) for r in self.reports]
        return max(dist(a[0], b[0]) for a, b in zip(poses, poses[1:]))


def moved(pose, dx=0.0, dy=0.0, dz=0.0):
    return ((pose[0][0] + dx, pose[0][1] + dy, pose[0][2] + dz), pose[1])


def scenarios(pair, lim: Limits) -> list[tuple[str, bool, str]]:
    out = []
    STEP_MS.clear()  # this run's steps only (the tests run every speed level in one process)
    v = lim.tool_speed * lim.dt

    r = Run(pair, lim=lim)
    goal = moved(r.start["left"], dx=0.06)
    r.go(75, lambda k: {"left": Request(goal)})
    e = dist(r.tool("left")[0], goal[0])
    out.append(("follows a 6 cm move", e < 0.002 and r.max_step("left") <= v + 1e-4,
                f"ends {e * 1000:.1f} mm from the target; fastest step {r.max_step('left') * 1000:.1f} mm (limit {v * 1000:.1f})"))

    r = Run(pair, lim=lim)
    r.go(200, lambda k: {"left": Request(moved(r.start["left"], dz=0.5))})
    z = r.tool("left")[0][2]
    out.append(("stops at the box ceiling", abs(z - LEFT.box[5]) < 0.003 and "at the box edge" in r.notes("left"),
                f"grasp point at z {z * 1000:.0f} mm, box top {LEFT.box[5] * 1000:.0f} mm"))

    r = Run(pair, lim=lim)
    r.go(5, lambda k: {"left": Request(moved(r.start["left"], dx=0.004 * k + (0.12 if k == 4 else 0)))})
    st = r.reports[-1]["left"]
    out.append(("a 12 cm jump in one step lets the arm go", st.state == "let go" and not r.c.engaged["left"] and r.max_step("left") <= v + 1e-4,
                f"state {st.state}: {st.notes[0] if st.notes else ''}"))

    r = Run(pair, lim=lim)
    r.go(150, lambda k: {"left": Request(moved(r.start["left"], dy=-0.62)), "right": Request(moved(r.start["right"], dy=0.62))})
    cross = [w for p in r.problems for w in p if w.startswith("left") and "right" in w and "and the table" not in w]
    gap = dist(r.tool("left")[0], r.tool("right")[0])
    out.append(("the two arms stop before touching", not cross and "too close" in r.notes("left") | r.notes("right"),
                f"grasp points {gap * 1000:.0f} mm apart when stopped; no part closer than {lim.clearance * 1000:.0f} mm at any step"))

    low = [dataclasses.replace(a, box=a.box[:4] + (-0.20, a.box[5])) for a in ARMS]  # box floor below the table
    down = lambda work: ((0.30, work[0][1], 0.15), DOWN)  # noqa: E731  gripper pointing down, 15 cm up (pointing down it reaches ~20 cm at most)
    r = Run(pair, arms=low, start=down, lim=lim)
    r.go(250, lambda k: {"left": Request(moved(r.start["left"], dz=-0.4))})
    table = [w for p in r.problems for w in p if "table" in w]
    out.append(("stops above the table when the box allows lower", not table and "too close" in r.notes("left"),
                f"grasp point stopped at z {r.tool('left')[0][2] * 1000:.0f} mm"))

    r = Run(pair, lim=lim)
    r.go(10, lambda k: {"left": Request(moved(r.start["left"], dx=0.01))})
    before = list(r.c.q["left"])
    r.go(10, lambda k: {"left": Request(None)})
    out.append(("no hand input: let go, stays put", r.reports[10]["left"].state == "let go" and r.c.q["left"] == before and not r.c.engaged["left"],
                "arm commands unchanged after the input stopped"))

    big = [dataclasses.replace(a, box=(-1, 1, -1, 1, 0.03, 1)) for a in ARMS]
    r = Run(pair, arms=big, lim=lim)
    r.go(400, lambda k: {"left": Request(moved(r.start["left"], dx=0.6))})
    lo_hi = pair.joint_ranges("left")
    inside = all(lo - 1e-9 <= q <= hi + 1e-9 for rep in r.reports for q, (lo, hi) in zip(rep["left"].q[:6], lo_hi))
    qs = [WORK] + [rep["left"].q[:6] for rep in r.reports]
    fastest = max(abs(b - a) for x, y in zip(qs, qs[1:]) for a, b in zip(x, y))
    out.append(("out of reach: follows as far as it goes", "out of reach by" in " ".join(x for rep in r.reports for x in rep["left"].notes)
                and inside and fastest <= lim.joint_speed * lim.dt + 1e-9,
                f"stopped {dist(r.tool('left')[0], moved(r.start['left'], dx=0.6)[0]) * 1000:.0f} mm short; joints inside their ranges; "
                f"fastest joint step {math.degrees(fastest):.1f} deg (limit {math.degrees(lim.joint_speed * lim.dt):.1f})"))

    # The hand pulls back toward the operator: the grasp point comes closer to its base than the old 10 cm edge,
    # stopping at the box edge (BOX_NEAR) or earlier, without any part of the arm running into itself or the table.
    r = Run(pair, lim=lim)
    r.go(int(0.5 / (lim.tool_speed * lim.dt)) + 60, lambda k: {"left": Request(moved(r.start["left"], dx=-0.6))})
    x = r.tool("left")[0][0]
    own = [w for p in r.problems for w in p if w.startswith("left") and ("overlap" in w or "table" in w)]
    out.append(("pulls back close to its base without hitting itself", not own and x < 0.10,
                f"grasp point stopped {x * 1000:.0f} mm in front of the base (box edge {LEFT.box[0] * 1000:.0f} mm, was 100 mm)"
                + (f"; problems: {own[:2]}" if own else "")))

    r = Run(pair, lim=lim)
    turned = (r.start["left"][0], q_mul(r.start["left"][1], q_from_rotvec((0, 0, math.radians(60)))))  # about the gripper's own axis
    r.go(int(math.radians(60) / (lim.tool_rot_speed * lim.dt)) + 20, lambda k: {"left": Request(turned)})  # enough steps at this level
    angles = [q_angle(a[1], b[1]) for a, b in zip([r.start["left"]] + [pair.fk("left", x["left"].q[:6]) for x in r.reports],
                                                     [pair.fk("left", x["left"].q[:6]) for x in r.reports])]  # fmt: skip
    w = math.degrees(max(angles)) / lim.dt
    out.append(("turns at the turn speed limit", q_angle(r.tool("left")[1], turned[1]) < math.radians(1) and w <= math.degrees(lim.tool_rot_speed) * 1.05,
                f"fastest turn {w:.0f} deg/s (limit {math.degrees(lim.tool_rot_speed):.0f})"))

    # The hand turns far past what the wrist allows while moving 5 cm: the gripper angle stops short, the grasp
    # point still goes where the hand goes (before 2026-11-18 the whole arm was held back at a wrist end stop).
    worst, gave, short = 0.0, 0, []
    for axis in ((0, 1, 0), (1, 0, 0), (0, 0, 1)):
        r = Run(pair, lim=lim)
        goal = moved(r.start["left"], dx=0.05)
        turned = (goal[0], q_mul(q_from_rotvec(tuple(math.radians(170) * v for v in axis)), r.start["left"][1]))
        r.go(int(math.radians(170) / (lim.tool_rot_speed * lim.dt)) + 100, lambda k: {"left": Request(turned)})
        worst = max(worst, dist(r.tool("left")[0], goal[0]))
        gave += "wrist at its limit" in r.notes("left")
        short.append(f"{math.degrees(q_angle(r.tool('left')[1], turned[1])):.0f}")
    out.append(("a turn past the wrist's range: the grasp point still follows", worst < lim.reach_tol + 0.001 and gave >= 1,
                f"grasp point at most {worst * 1000:.1f} mm from the hand's; the angle gave way for {gave} of 3 turns "
                f"(gripper angle short by {', '.join(short)} deg)"))

    r = Run(pair, lim=lim)
    r.go(int(OPEN / (lim.grip_speed * lim.dt)) + 5, lambda k: {"left": Request(r.start["left"], grip=0.0)})  # enough steps at this level
    g = [x["left"].q[6] for x in r.reports]
    out.append(("gripper closes at its speed limit", g[-1] == 0.0 and max(a - b for a, b in zip([OPEN] + g, g)) <= lim.grip_speed * lim.dt + 1e-9,
                f"opening {OPEN} -> {g[-1]}" + (f" in {g.index(0.0) + 1} steps" if 0.0 in g else "")))

    r = Run(pair, lim=lim)  # both arms reach toward the middle, then everything is lowered to rest
    r.go(150, lambda k: {"left": Request(moved(r.start["left"], dy=-0.3, dz=-0.1)), "right": Request(moved(r.start["right"], dy=0.3, dz=-0.1))})
    rest = {n: pair.rest(n) for n in pair.names}
    steps, done, worse = 0, False, []
    while not done and steps < 3000:
        before = pair.clearance_problems(r.c.q)
        done, said = r.c.park_tick(rest, 0.3)
        after = pair.clearance_problems(r.c.q)
        worse += [w for w, d in after.items() if w not in before or d < before[w] - 1e-4]
        steps += 1
    out.append(("lowers both arms to rest from the middle", done and not worse,
                f"at rest after {steps * lim.dt:.1f} s at 0.3 rad/s; no part brought closer than allowed on the way"))

    # 99.9% of the steps within half a period, none over two. A rare late step is harmless (I2RT keeps sending the
    # last command) and happens on the shared lab host when other users' programs run: 10-20 ms once in ~8000 steps
    # in a full test run on 2026-11-18, never above 8 ms run alone; real sessions showed up to 33 ms before then.
    ms = sorted(STEP_MS)
    p999 = ms[min(len(ms) - 1, int(len(ms) * 0.999))]
    out.append(("one control step for both arms is fast enough", p999 < lim.dt * 1000 / 2 and ms[-1] < lim.dt * 1000 * 2,
                f"{ms[len(ms) // 2]:.1f} ms typical, 99.9% within {p999:.1f} ms, {ms[-1]:.1f} ms worst (step is {lim.dt * 1000:.0f} ms)"))
    return out


def main(argv: list[str] | None = None) -> int:
    import argparse

    from yam_setup import SPEEDS, limits_at
    from yam_sim import SimYamPair

    ap = argparse.ArgumentParser(description="drive the simulated arms through every safety rule")
    ap.add_argument("--speed", type=float, default=15, help=f"speed level in cm/s: {', '.join(f'{s:g}' for s in SPEEDS)}")
    lim = limits_at(ap.parse_args(argv).speed)
    print(f"speed level: tool {lim.tool_speed:g} m/s, {math.degrees(lim.tool_rot_speed):.0f} deg/s, joints {lim.joint_speed:g} rad/s, "
          f"clearance {lim.clearance * 1000:.0f} mm")
    pair = SimYamPair(ARMS, lim)
    try:
        results = scenarios(pair, lim)
    finally:
        pair.close()
    for name, ok, why in results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {why}")
    return 0 if all(ok for _, ok, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
