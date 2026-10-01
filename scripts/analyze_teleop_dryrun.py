#!/usr/bin/env python3
"""Analyse one live teleop DRY RUN (fake backend). NO ROBOT OR GRIPPER WAS CONNECTED.

Reads the adapter recording made on the lab host, the link log, and the local sender log and
cue marks. Lab-clock and local-clock times are never subtracted: cue marks (local clock) are
placed on the lab timeline through the transport sequence number of the packet sent nearest
to the mark, so cue windows are approximate by one transit time.
All tool positions are SYNTHETIC values from FakeRobotBackend.
"""
from __future__ import annotations

import bisect
import collections
import json
import math
import sys
from pathlib import Path


def rng(vals):  # type: ignore[no-untyped-def]
    return [round(min(vals), 1), round(max(vals), 1)] if vals else None


def qangle(a, b):  # type: ignore[no-untyped-def]
    d = abs(sum(a[k] * b[k] for k in range(4)))
    return math.degrees(2 * math.acos(min(1.0, d)))


def main() -> int:
    d, name = Path(sys.argv[1]), sys.argv[2]
    rows = [json.loads(x) for x in open(d / f"teleop_{name}.jsonl")]
    head, ticks = rows[0], [r for r in rows if r["k"] == "tick"]
    cfg = head["config"]
    R, scale = cfg["mapping"]["R_backend_from_local"], cfg["mapping"]["scale"]
    t0 = ticks[0]["t"]
    rel = lambda t: round((t - t0) / 1e9, 3)  # noqa: E731  (lab clock, s after the first tick)
    out = {"run": name, "banner": head["banner"], "backend": head["backend"], "config_label": cfg["label"],
           "mapping_label": cfg["mapping"]["label"], "scale": scale}  # fmt: skip

    # cue marks -> lab timeline via packet sequence numbers
    tx = sorted((p["t"], p["seq"]) for p in map(json.loads, open(d / f"tx_{name}.jsonl")) if p.get("dir") == "out")
    link = {p["seq"]: p["t"] for p in map(json.loads, open(d / f"teleop_link_{name}.jsonl")) if p.get("seq") is not None}
    marks = []
    for ln in open(d / f"marks_{name}.txt"):
        t, label = ln.rstrip("\n").split(" ", 1)
        i = min(bisect.bisect_left(tx, (int(t), 0)), len(tx) - 1)
        lab = link.get(tx[i][1])
        marks.append({"label": label, "lab_s": None if lab is None else rel(lab)})
    out["cue_marks_on_lab_timeline_approx"] = marks

    trans = [r for r in rows if r["k"] == "transition"]
    out["transitions"] = [{"t_s": rel(r["t"]), "old": r["old"], "new": r["new"], "reason": r["reason"]} for r in trans]
    out["faults"] = [{"t_s": rel(r["t"]), "reason": r["reason"], "limit_violation": r["limit"]} for r in rows if r["k"] == "fault"]
    out["limit_violations"] = sum(1 for r in rows if r["k"] == "fault" and r["limit"])
    out["stop_requests"] = [{"t_s": rel(r["t"]), "reason": r["reason"], "by": r["by"], "synthetic": r["synthetic"]} for r in rows if r["k"] == "stop"]
    out["acknowledgements"] = [{"t_s": rel(r["t"]), "accepted": r["accepted"], "detail": r["detail"]} for r in rows if r["k"] == "ack"]

    # engagements
    engs, cur = [], None
    for i, t in enumerate(ticks):
        if t["state"] == "ACTIVE":
            if cur is None:
                cur = {"first": i}
            cur["last"] = i
        elif cur is not None:
            engs.append(cur)
            cur = None
    if cur is not None:
        engs.append(cur)
    vmax = amax = 0.0
    eng_out = []
    for e in engs:
        seg = ticks[e["first"]:e["last"] + 1]
        before, after = ticks[e["first"] - 1], ticks[min(e["last"] + 1, len(ticks) - 1)]
        c0, p0 = seg[0]["ctrl_mm"], seg[0]["cmd_mm"]
        map_err = lag = 0.0
        prev_p, prev_v = None, (0.0, 0.0, 0.0)
        for t in seg:
            dl = [t["ctrl_mm"][k] - c0[k] for k in range(3)]
            want = [p0[i] + scale * sum(R[i][k] * dl[k] for k in range(3)) for i in range(3)]
            if t["req_mm"] is not None and t["input_age_ms"] is not None:
                map_err = max(map_err, max(abs(want[k] - t["req_mm"][k]) for k in range(3)))
            lag = max(lag, math.dist(t["req_mm"], t["lim_mm"]))
            if prev_p is not None and t["dt_ms"]:
                dt = t["dt_ms"] / 1000.0
                v = tuple((t["cmd_mm"][k] - prev_p[k]) / dt for k in range(3))
                vmax = max(vmax, math.sqrt(sum(x * x for x in v)))
                amax = max(amax, math.sqrt(sum((v[k] - prev_v[k]) ** 2 for k in range(3))) / dt)
                prev_v = v
            prev_p = t["cmd_mm"]
        q0 = seg[0]["ctrl_q"]
        end_reason = next((r["reason"] for r in trans if r["old"] == "ACTIVE" and r["t"] >= seg[-1]["t"]), None)
        eng_out.append({
            "start_s": rel(seg[0]["t"]), "end_s": rel(seg[-1]["t"]), "duration_s": round((seg[-1]["t"] - seg[0]["t"]) / 1e9, 2),
            "end_reason": end_reason, "target_requests": len(seg), "continuity_gen": sorted({t["gen"] for t in seg}),
            "source": sorted({(t["src"][0], t["src"][1][:8]) for t in seg}),
            "grip_at_engagement": seg[0]["grip"], "grip_before": before["grip"],
            "tool_feedback_before_mm": before["fb_mm"], "first_target_mm": seg[0]["cmd_mm"],
            "engagement_jump_mm": round(math.dist(before["fb_mm"], seg[0]["cmd_mm"]), 6),
            "controller_travel_range_mm": {ax: rng([t["ctrl_mm"][k] - c0[k] for t in seg]) for k, ax in enumerate("xyz")},
            "requested_range_rel_anchor_mm": {ax: rng([t["req_mm"][k] - p0[k] for t in seg]) for k, ax in enumerate("xyz")},
            "limited_range_rel_anchor_mm": {ax: rng([t["lim_mm"][k] - p0[k] for t in seg]) for k, ax in enumerate("xyz")},
            "max_request_minus_limited_mm": round(lag, 2), "max_mapping_error_mm": round(map_err, 9),
            "max_controller_rotation_from_start_deg": round(max(qangle(q0, t["ctrl_q"]) for t in seg), 1),
            "orientation_outputs": sorted({tuple(t["ori"]) for t in seg}),
            "max_input_age_ms": max(t["input_age_ms"] for t in seg),
            "gripper_preview": dict(collections.Counter(t["gripper"] for t in seg)),
            "gripper_pct_range": rng([t["gripper_pct"] for t in seg if t["gripper_pct"] is not None]),
            "trigger_range": rng([t["trig"] * 100 for t in seg if t["trig"] is not None]),
            "first_tick_after": {"state": after["state"], "blocked": after["blocked"], "req_mm": after["req_mm"], "cmd_mm": after["cmd_mm"], "ori": after["ori"]},
        })  # fmt: skip
    out["engagements"] = eng_out
    out["limits_observed"] = {"max_target_speed_mm_s": round(vmax, 2), "configured_v_max_mm_s": cfg["limits"]["v_max_m_s"] * 1000,
                              "max_target_accel_mm_s2": round(amax, 1), "configured_a_max_mm_s2": cfg["limits"]["a_max_m_s2"] * 1000}  # fmt: skip

    # wrist-rotation window: orientation output vs controller rotation and requested translation
    def lab(label):  # type: ignore[no-untyped-def]
        return next((m["lab_s"] for m in marks if m["label"].startswith(label)), None)

    a, b = lab("cue rotate"), lab("state before release cue")
    if a is not None and b is not None:
        seg = [t for t in ticks if a <= rel(t["t"]) <= b and t["state"] == "ACTIVE"]
        if seg:
            out["wrist_rotation_window"] = {
                "lab_s": [a, b], "ticks": len(seg),
                "controller_rotation_span_deg": round(max(qangle(seg[0]["ctrl_q"], t["ctrl_q"]) for t in seg), 1),
                "orientation_outputs": sorted({tuple(t["ori"]) for t in seg}),
                "controller_position_span_mm": {ax: round(max(t["ctrl_mm"][k] for t in seg) - min(t["ctrl_mm"][k] for t in seg), 1) for k, ax in enumerate("xyz")},
                "requested_span_mm": {ax: round(max(t["req_mm"][k] for t in seg) - min(t["req_mm"][k] for t in seg), 1) for k, ax in enumerate("xyz")},
            }  # fmt: skip
    for cue in ("cue right", "cue up", "cue forward", "cue left"):
        a = lab(cue)
        if a is None:
            continue
        seg = [t for t in ticks if a <= rel(t["t"]) <= a + 3.5 and t["state"] == "ACTIVE"]
        if seg:
            out.setdefault("cue_windows", {})[cue] = {
                "controller_delta_local_mm": [round(seg[-1]["ctrl_mm"][k] - seg[0]["ctrl_mm"][k], 1) for k in range(3)],
                "controller_peak_delta_local_mm": [round(max((t["ctrl_mm"][k] - seg[0]["ctrl_mm"][k] for t in seg), key=abs), 1) for k in range(3)],
                "requested_peak_delta_synthetic_mm": [round(max((t["req_mm"][k] - seg[0]["req_mm"][k] for t in seg), key=abs), 1) for k in range(3)],
                "limited_peak_delta_synthetic_mm": [round(max((t["lim_mm"][k] - seg[0]["lim_mm"][k] for t in seg), key=abs), 1) for k in range(3)],
            }  # fmt: skip

    inactive = [t for t in ticks if t["state"] != "ACTIVE"]
    out["while_disengaged"] = {"ticks": len(inactive), "target_requests": sum(1 for t in inactive if t["cmd_mm"] is not None),
                               "gripper_proposals": sum(1 for t in inactive if t["gripper_pct"] is not None),
                               "ticks_with_grip_held_over_pressed_threshold": sum(1 for t in inactive if (t["grip"] or 0) >= cfg["engagement"]["grip_pressed_min"]),
                               "blocked_reasons": dict(collections.Counter(t["blocked"].split(" (")[0].split(":")[0] for t in inactive).most_common(12))}  # fmt: skip
    dts = sorted(t["dt_ms"] for t in ticks if t["dt_ms"] is not None)
    out["scheduler"] = {"ticks": len(ticks), "dt_ms_median": dts[len(dts) // 2], "dt_ms_p99": dts[int(0.99 * (len(dts) - 1))], "dt_ms_max": dts[-1]}
    out["all_ticks_synthetic"] = all(t["synthetic"] for t in ticks)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
