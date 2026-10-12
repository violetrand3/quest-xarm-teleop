#!/usr/bin/env python3
"""REAL GRIPPER step check after a repair: move the jaws by a set amount and back. MOVES THE JAWS.

Loopback gripctl service only, with the same client as the trigger path (three requests:
GET /api/calibration, GET /api/state, POST /api/do/grip). Sequence: start -> start + delta
(percent closed, per the driver's own calibration) -> start. The servo position is logged every
20 ms with the lab's monotonic clock so it can be compared with camera frames: the servo's own
feedback does not show whether the jaws followed.
Needs --moves-real-gripper and --human-confirmation.
"""
import argparse
import json
import sys
import time
from pathlib import Path

from gripctl_client import GripctlClient

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("--moves-real-gripper", action="store_true")
ap.add_argument("--human-confirmation", default="")
ap.add_argument("--delta-pct", type=float, default=30.0)
ap.add_argument("--targets", default="", help="comma list of percent-closed plateaus instead of one delta (each step at most 35 %%)")
ap.add_argument("--dwell", type=float, default=2.5)
ap.add_argument("--out", type=Path, required=True)
a = ap.parse_args()
if not a.moves_real_gripper or len(a.human_confirmation.strip()) < 10 or not (5 <= abs(a.delta_pct) <= 60):
    print("error: needs --moves-real-gripper, --human-confirmation and a delta of 5..60 %; nothing was sent", file=sys.stderr)
    sys.exit(2)
g = GripctlClient("http://127.0.0.1:8088")
cal = g.load_calibration()
s0 = g.refresh_state()
if not s0 or not s0.get("connected") or s0.get("moving"):
    print(f"error: gripper not connected and still ({s0}); nothing was sent", file=sys.stderr)
    sys.exit(3)
start = round(s0["pct"], 2)
plateaus = [round(float(x), 2) for x in a.targets.split(",") if x.strip()] or [round(start + a.delta_pct, 2)]
seq, prev = [], start
for t in plateaus + [start]:
    if (t != start and not (2.0 <= t <= 98.0)) or abs(t - prev) > 35.0 + 1e-6:
        if t == start and abs(t - prev) > 35.0:  # come back in steps of at most 35 %
            while abs(t - prev) > 35.0:
                prev = round(prev - 35.0 if prev > t else prev + 35.0, 2)
                seq.append((f"back_via_{prev:g}", prev))
        else:
            print(f"error: plateau {t} % is outside 2..98 % or more than 35 % from the previous one; nothing was sent", file=sys.stderr)
            sys.exit(3)
    seq.append(("back_to_start" if t == start else f"to_{t:g}", t))
    prev = t
out = {"tool": "gripper_step_check", "calibration": cal, "start": s0, "human_confirmation": a.human_confirmation, "steps": []}
for label, pct in seq:
    t_ns = time.monotonic_ns()
    r = g._post_grip(pct)
    step = {"label": label, "pct": pct, "t_mono_ns": t_ns, "post_ms": round((time.monotonic_ns() - t_ns) / 1e6, 1), "reply": r, "samples": []}
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.dwell:
        s = g.refresh_state() or {}
        step["samples"].append((time.monotonic_ns(), s.get("ticks"), None if s.get("pct") is None else round(s["pct"], 2), s.get("load")))
        time.sleep(0.02)
    out["steps"].append(step)
    print(label, "->", pct, "% :", "ok" if r.get("ok") else r, "| final", step["samples"][-1][1:], flush=True)
    if not r.get("ok"):
        break
out["end"] = g.refresh_state()
a.out.write_text(json.dumps(out) + "\n")
