#!/usr/bin/env python3
"""Summarise one live teleop session from its saved files (reads files only).
Usage: analyze_live_teleop.py <data dir> <session name>"""
import bisect
import collections
import json
import math
import sys

D, N = sys.argv[1].rstrip("/") + "/", sys.argv[2]
o = json.load(open(D + f"teleop_{N}.summary.json"))
print({k: o.get(k) for k in ("result", "engagements", "targets_sent", "stops", "halts", "final_state", "final_latch", "elapsed_s")})
print("faults (excluding start-up notices):", [(x["t_s"], x["reason"][:110]) for x in o["faults"] if "reference" not in x["reason"]])
ends = collections.Counter(x["reason"].split(":")[0].split(" (")[0] for x in o["transitions"] if x["old"] == "ACTIVE")
print("why engagements ended:", dict(ends))
print("after:", [round(v, 2) for v in o["after"]["pose"][:3]], [round(math.degrees(v), 2) for v in o["after"]["pose"][3:]],
      "state", o["after"]["state"], "error", o["after"]["error"], "warn", o["after"]["warn"])
print("refused by the link:", o["ledger"]["api_calls_refused"], len(o["ledger"]["wire_frames_refused"]),
      "| sent:", {k: v for k, v in o["ledger"]["wire_frames_sent_by_register"].items() if not k.split()[1].startswith("GET")})
rows = [json.loads(x) for x in open(D + f"teleop_{N}.jsonl")]
tk = [r for r in rows if r["k"] == "tick"]
ws = o["workspace_mm"]
s0 = ws["start"]
act = [r for r in tk if r["state"] == "ACTIVE"]
print("time engaged: %.1f s in %d engagements" % (len(act) * 0.02, o["engagements"]))
for k, ax in enumerate("xyz"):
    v = [r["fb_mm"][k] for r in act]
    print(f"  arm {ax}: {min(v):.1f} .. {max(v):.1f} mm (box {ws['min'][k]} .. {ws['max'][k]}; start {s0[k]})")
lag = sorted(math.dist(r["lim_mm"], r["fb_mm"]) for r in act if r["lim_mm"])
print("target-to-arm distance mm: median %.2f p99 %.2f max %.2f" % (lag[len(lag) // 2], lag[int(0.99 * len(lag))], lag[-1]))
ages = sorted(r["input_age_ms"] for r in act)
print("input age ms: median", ages[len(ages) // 2], "p99", ages[int(0.99 * len(ages))], "max", ages[-1])
sp = sorted(math.dist(a["fb_mm"], b["fb_mm"]) / ((b["t"] - a["t"]) / 1e9) for a, b in zip(act, act[5:]) if 0.09 < (b["t"] - a["t"]) / 1e9 < 0.12)
print("arm speed mm/s: p50 %.1f p90 %.1f p99 %.1f max %.1f" % (sp[len(sp) // 2], sp[int(0.9 * len(sp))], sp[int(0.99 * len(sp))], sp[-1]))
print("orientation targets used:", len({tuple(r["ori"]) for r in act}))
if "gripper" in o:
    g = [r for r in act if r.get("gripper_fb_pct") is not None]
    cm = [r for r in rows if r["k"] == "gripper_command"]
    print("gripper: commands", len(cm), "ok", sum(c["ok"] for c in cm), "| servo %.1f..%.1f %% closed" % (min(r["gripper_fb_pct"] for r in g), max(r["gripper_fb_pct"] for r in g)),
          "| follower", dict(collections.Counter(r["gripper"] for r in act)), "| end", {k: o["gripper"]["end"].get(k) for k in ("ticks", "connected")})
try:
    t = json.load(open(D + f"tx_{N}.summary.json"))
    print("RTT", t["rtt_sender_clock"], "| source gaps", t["source"]["source_gap_records"])
except OSError:
    pass
T = [r["t"] for r in tk]
for key in ("iphone", "zed"):
    try:
        d = json.load(open(D + f"refdiff_{N}_{key}.json"))
    except OSError:
        continue
    xs, ys = [], []
    for f in d:
        i = bisect.bisect_left(T, f["t"])
        if 0 < i < len(T):
            xs.append(math.dist(tk[i]["fb_mm"], s0))
            ys.append(f["diff"])
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    c = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / math.sqrt(sum((x - mx) ** 2 for x in xs) * sum((y - my) ** 2 for y in ys))
    print(f"{key}: {len(d)} frames; picture change vs arm distance from start: correlation {c:.2f}")
