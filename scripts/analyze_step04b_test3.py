#!/usr/bin/env python3
"""Offline timeline analysis for Step 4B Test 3 (focus loss/return + grip release/repress).

Uses only the recorder JSONL (raw payloads, host receive time, status_after, proc_ns) and the
cue marks (host monotonic, same clock). XrTime values are reported as-is, never subtracted from
host time. "Available" = recorder status FRESH_TRACKED (focused, right pose action active,
VALID+TRACKED, grip action active). Release/repress evidence uses only available records.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import quest_pose_v1 as qp  # noqa: E402
from quest_transport import stats_ms  # noqa: E402


def main(base: Path) -> dict:
    L = [json.loads(x) for x in base.with_suffix(".jsonl").open()]
    marks = [(int(r.split(" ", 1)[0]), r.split(" ", 1)[1].strip()) for r in base.with_suffix(".marks.txt").read_text().splitlines()]
    t0 = next(t for t, l in marks if l == "phase_start baseline")
    rel = lambda h: round((h - t0) / 1e9, 3)  # noqa: E731
    rows = []
    for r in L:
        if "error" in r:
            rows.append({"t": rel(r["host_ns"]), "kind": "parse_error", "r": r})
            continue
        rec = qp.decode_record(r["raw"])
        rows.append({"t": rel(r["host_ns"]), "host": r["host_ns"], "rec": rec, "status": r.get("status_after"),
                     "proc_ns": r.get("proc_ns"), "kind": r.get("type")})  # fmt: skip
    poses = [x for x in rows if x["kind"] == "pose"]
    events = [x for x in rows if x["kind"] == "event"]
    out: dict = {"base": str(base), "marks_rel_s": {l: rel(t) for t, l in marks}}
    out["events"] = [{"t": e["t"], "seq": e["rec"].seq, "event": e["rec"].event, "state": e["rec"].session_state,
                      "xr_time_ns": e["rec"].time_ns, "status_after": e["status"], "proc_us": round(e["proc_ns"] / 1e3, 1)}
                     for e in events]  # fmt: skip
    avail = [x["status"] == "FRESH_TRACKED" for x in poses]
    first_unavail = next(i for i, a in enumerate(avail) if not a and poses[i]["t"] > 0) if any(not a for a in avail) else None
    tl = {}
    if first_unavail is not None:
        last_ok = poses[first_unavail - 1]
        tl["last_available_before"] = {"t": last_ok["t"], "seq": last_ok["rec"].seq, "grip": last_ok["rec"].inputs["in_r"]["squeeze"]}
        fu = poses[first_unavail]
        g = fu["rec"].poses["grip_r"]
        tl["first_unavailable_pose"] = {"t": fu["t"], "seq": fu["rec"].seq, "status": fu["status"], "focused": fu["rec"].focused,
                                        "state": fu["rec"].session_state, "r_active": g.active, "r_flags": g.flags,
                                        "grip": fu["rec"].inputs["in_r"]["squeeze"]}  # fmt: skip
        foc = next((x for x in poses if not x["rec"].focused), None)
        tl["first_unfocused_pose"] = foc and {"t": foc["t"], "seq": foc["rec"].seq, "state": foc["rec"].session_state,
                                              "r_active": foc["rec"].poses["grip_r"].active, "grip": foc["rec"].inputs["in_r"]["squeeze"]}  # fmt: skip
        ina = next((x for x in poses if x["rec"].poses["grip_r"].active is not True and x["t"] > 0), None)
        tl["first_right_action_inactive"] = ina and {"t": ina["t"], "seq": ina["rec"].seq, "focused": ina["rec"].focused}
        gnull = next((x for x in poses if x["rec"].inputs["in_r"]["squeeze"] is None and x["t"] > 0), None)
        tl["first_grip_null"] = gnull and {"t": gnull["t"], "seq": gnull["rec"].seq}
        rec_i = next((i for i in range(first_unavail, len(poses)) if avail[i]), None)
        if rec_i is not None:
            fr = poses[rec_i]
            tl["first_available_after"] = {"t": fr["t"], "seq": fr["rec"].seq, "grip": fr["rec"].inputs["in_r"]["squeeze"],
                                           "next5_grip": [p["rec"].inputs["in_r"]["squeeze"] for p in poses[rec_i:rec_i + 5]]}  # fmt: skip
            tl["unavailable_span_host_s"] = round(fr["t"] - last_ok["t"], 3)
            tl["unavailable_span_xr_ms"] = round((fr["rec"].t_query_ns - last_ok["rec"].t_query_ns) / 1e6, 1)
            tl["statuses_during_unavailable"] = sorted({p["status"] for p in poses[first_unavail:rec_i]})
            refoc = next((x for x in poses[first_unavail:] if x["rec"].focused), None)
            tl["first_refocused_pose"] = refoc and {"t": refoc["t"], "seq": refoc["rec"].seq, "status": refoc["status"],
                                                    "r_active": refoc["rec"].poses["grip_r"].active}  # fmt: skip
    out["timeline"] = tl
    # pause / watchdog
    hs = [x["host"] for x in rows if "host" in x]
    gaps = sorted(((b - a) / 1e6, rel(a)) for a, b in zip(hs, hs[1:]))
    out["largest_host_gaps_ms"] = [{"gap_ms": round(g, 1), "after_t": t} for g, t in gaps[-5:]]
    out["host_interarrival_all_lines"] = stats_ms([int((b - a)) for a, b in zip(hs, hs[1:])])
    # release / repress among available records after the release cue
    t_rel = out["marks_rel_s"].get("phase_start release")
    ok_after = [(x["t"], x["rec"].inputs["in_r"]["squeeze"]) for x in poses if t_rel is not None and x["t"] >= t_rel - 0.2 and x["status"] == "FRESH_TRACKED"]
    rr = {}
    if ok_after:
        low = next(((t, v) for t, v in ok_after if v < 0.5), None)
        rr["first_below_0.5"] = low
        if low:
            after = [(t, v) for t, v in ok_after if t >= low[0]]
            rr["min_during_release"] = min(after[: next((i for i, (t, v) in enumerate(after) if v >= 0.5), len(after))], key=lambda tv: tv[1], default=None)
            rr["first_back_at_or_above_0.5"] = next(((t, v) for t, v in after if v >= 0.5), None)
            rr["released_window_s"] = (round(rr["first_back_at_or_above_0.5"][0] - low[0], 3) if rr["first_back_at_or_above_0.5"] else None)
        rr["available_records_after_release_cue"] = len(ok_after)
        rr["unavailable_records_after_release_cue"] = sum(1 for x in poses if t_rel is not None and x["t"] >= t_rel and x["status"] != "FRESH_TRACKED")
    out["release_repress"] = rr
    # grip below 0.5 anywhere among available records (other than the release window)
    out["available_grip_below_0.5_times"] = [x["t"] for x in poses if x["status"] == "FRESH_TRACKED" and x["rec"].inputs["in_r"]["squeeze"] < 0.5][:20]
    # contradictions
    contra = []
    for x in poses:
        r = x["rec"]
        g = r.poses["grip_r"]
        if r.focused != (r.session_state == "FOCUSED"):
            contra.append((x["t"], "focused flag disagrees with session_state", r.session_state, r.focused))
        if g.active is False and g.flags != 0:
            contra.append((x["t"], "inactive pose action but nonzero flags", g.flags))
        if (g.active is True) != (r.inputs["in_r"]["squeeze"] is not None):
            contra.append((x["t"], "pose-action activity differs from grip-action activity", g.active, r.inputs["in_r"]["squeeze"]))
        if not r.focused and x["status"] == "FRESH_TRACKED":
            contra.append((x["t"], "unfocused but status FRESH_TRACKED"))
    out["contradictions"] = {"count": len(contra), "first": contra[:10]}
    # sequence continuity
    seqs = sorted(x["rec"].seq for x in rows if "rec" in x)
    missing = [s for a, b in zip(seqs, seqs[1:]) for s in range(a + 1, b)]
    out["seq_missing"] = {"count": len(missing), "list": missing[:40], "note": "pose and event records share one counter"}
    bys = {x["rec"].seq: x for x in rows if "rec" in x}
    out["seq_missing_context"] = [{"seq": m, "t_prev": bys.get(m - 1, {}).get("t"), "prev_kind": bys.get(m - 1, {}).get("kind"),
                                   "next_kind": bys.get(m + 1, {}).get("kind")} for m in missing[:40]]  # fmt: skip
    out["proc_delay_us_all_lines"] = {k: v for k, v in stats_ms([x["proc_ns"] for x in rows if x.get("proc_ns")]).items()}
    out["counts"] = {"lines": len(L), "pose": len(poses), "events": len(events), "parse_errors": sum(1 for x in rows if x["kind"] == "parse_error"),
                     "overflow": sum(1 for x in rows if x["kind"] == "overflow"), "max_bytes": max(r["bytes"] for r in L)}  # fmt: skip
    return out


if __name__ == "__main__":
    for a in sys.argv[1:]:
        res = main(Path(a))
        Path(a).with_suffix(".test3.json").write_text(json.dumps(res, indent=1, default=str))
        print(json.dumps(res, indent=1, default=str))
