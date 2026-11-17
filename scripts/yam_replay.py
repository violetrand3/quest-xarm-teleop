#!/usr/bin/env python3
"""Bimanual YAM, SIMULATION ONLY: replays recorded Quest hand motion through the whole teleop step (clutch,
mapping, limits, IK, keeping clear) on the two simulated arms, and reports what happened. No hardware.

Input: a source log the xArm sessions write on the operator's PC (data/sessions/src_<session>.jsonl): every
headset record, with the side grip, the trigger and both controllers. The xArm sessions used the right hand
only, so by default the left hand is the right hand MIRRORED (left/right swapped), which drives the two arms
toward each other at the same time; --left recorded uses the recorded left controller instead, --left none
leaves the left arm alone. --left-delay shifts the mirrored hand in time.

Run on the lab host in I2RT's environment:
    /home/dev/yam/i2rt/.venv/bin/python scripts/yam_replay.py <src log> [--log ticks.jsonl]
"""

from __future__ import annotations

import argparse
import bisect
import collections
import json
import math
import sys
import time
from pathlib import Path

import quest_pose_v1 as qp
from teleop_input import InputView
from yam_control import BimanualControl
from yam_input import hand_sample
from yam_setup import ARMS, Limits
from yam_teleop import YamTeleop

IDENT = ("replay", 1)


def mirrored(raw: dict) -> dict:
    """The record with the right controller copied to the left, reflected across the operator's mid-plane."""
    out = json.loads(json.dumps(raw))
    g = out["grip_r"]
    m = dict(g)
    if g.get("p") is not None:
        m["p"] = [-g["p"][0], g["p"][1], g["p"][2]]
    if g.get("q") is not None:
        m["q"] = [g["q"][0], -g["q"][1], -g["q"][2], g["q"][3]]  # reflection x -> -x: (qx, -qy, -qz, qw)
    out["grip_l"] = m
    i = out["in_r"]
    out["in_l"] = {k: i.get(k) for k in ("trigger", "squeeze", "thumbstick", "thumbstick_click", "thumbstick_touch",
                                          "trigger_touch", "thumbrest_touch")}  # fmt: skip
    out["in_l"].update({"x": i.get("a"), "y": i.get("b"), "menu": None})
    return out


def load(path: Path) -> list[tuple[int, dict]]:
    out = []
    for line in path.open(encoding="utf-8"):
        r = json.loads(line)
        if r.get("kind") == "pose":
            out.append((r["host_ns"], json.loads(r["raw"])))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src_log", type=Path)
    ap.add_argument("--left", choices=("mirror", "recorded", "none"), default="mirror")
    ap.add_argument("--left-delay", type=float, default=0.0, help="seconds the mirrored left hand lags the right")
    ap.add_argument("--scale", type=float, default=0.667, help="tool movement per hand movement")
    ap.add_argument("--operator-yaw", type=float, default=0.0)
    ap.add_argument("--max-s", type=float, default=0.0, help="replay at most this many seconds (0 = all)")
    ap.add_argument("--log", type=Path, help="write every step here (JSON lines)")
    a = ap.parse_args()

    from yam_sim import SimYamPair

    recs = load(a.src_log)
    if not recs:
        print("no pose records in", a.src_log)
        return 2
    times = [t for t, _ in recs]
    lim = Limits()
    pair = SimYamPair(ARMS, lim)
    work = [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    for n in pair.names:
        pair.place(n, work)
    control = BimanualControl(pair, ARMS, lim)
    teleop = YamTeleop(control, a.scale, a.operator_yaw, 400)
    log = a.log.open("w", encoding="utf-8") if a.log else None

    dt_ns = int(lim.dt * 1e9)
    t0, t_end = times[0], times[-1] if not a.max_s else min(times[-1], times[0] + int(a.max_s * 1e9))
    delay = int(a.left_delay * 1e9)
    seen = collections.defaultdict(collections.Counter)
    engaged_s = collections.Counter()
    path_m = collections.Counter()
    last_tool = {n: pair.fk(n, control.q[n][:6]) for n in pair.names}
    worst_problems: dict[str, float] = {}
    step_ms = []
    t = t0
    while t <= t_end:
        views = {}
        for hand in pair.names:
            src_t = t - (delay if hand == "left" and a.left == "mirror" else 0)
            i = bisect.bisect_right(times, src_t) - 1
            if i < 0 or (hand == "left" and a.left == "none"):
                views[hand] = None
                continue
            arrival, raw = recs[i]
            if hand == "left" and a.left == "mirror":
                raw = mirrored(raw)
            rec = qp.decode_record(json.dumps(raw))
            s = hand_sample(rec, hand, arrival + (t - src_t), i, IDENT + (rec.build, rec.sid, 1, rec.local_epoch, rec.local_change_time_ns), 1)
            views[hand] = InputView(True, True, False, 1, s, (), 0)
        c0 = time.perf_counter()
        reports = teleop.tick(views, t)
        step_ms.append((time.perf_counter() - c0) * 1000)
        for n, r in reports.items():
            for note in r.notes:
                seen[n][note.split(":")[0].split(" (")[0]] += 1
            if r.clutch == "ACTIVE":
                engaged_s[n] += lim.dt
            tool = pair.fk(n, control.q[n][:6])
            path_m[n] += math.dist(tool[0], last_tool[n][0])
            last_tool[n] = tool
        for what, d in pair.clearance_problems(control.q).items():
            worst_problems[what] = min(d, worst_problems.get(what, 1.0))
        if log:
            log.write(json.dumps({"t": round((t - t0) / 1e9, 3), **{n: {"clutch": r.clutch, "arm": r.arm, "grip": r.gripper,
                                  "q": [round(v, 4) for v in control.q[n]], "tool": [round(v, 4) for v in last_tool[n][0]],
                                  "notes": r.notes} for n, r in reports.items()}}) + "\n")  # fmt: skip
        t += dt_ns
    pair.close()
    if log:
        log.close()

    secs = (t_end - t0) / 1e9
    print(f"replayed {secs:.0f} s of {a.src_log.name} ({len(step_ms)} steps), left hand: {a.left}"
          + (f" {a.left_delay:g} s late" if a.left == "mirror" and a.left_delay else ""))
    for n in pair.names:
        print(f"{n}: engaged {engaged_s[n]:.0f} s, tool moved {path_m[n]:.2f} m")
        for what, k in seen[n].most_common(8):
            print(f"    {k:6d} x  {what}")
    print("closest approaches below the allowed clearance at any step:", worst_problems or "none")
    ms = sorted(step_ms)
    print(f"control step: {ms[len(ms) // 2]:.1f} ms typical, {ms[int(len(ms) * 0.999)]:.1f} ms at 99.9%, {ms[-1]:.1f} ms worst")
    return 0


if __name__ == "__main__":
    sys.exit(main())
