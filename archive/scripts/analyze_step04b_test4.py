#!/usr/bin/env python3
"""Offline timeline analysis for Step 4B Test 4 (right-controller tracking loss/recovery).

Reads only the recorder JSONL + cue marks. Reports, per record, every concurrent reason the
right input is unavailable (not just the displayed primary status), flag transitions with host
receive time (monotonic, relative to the baseline cue) and query XrTime (never mixed).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))  # archived: the modules stay in scripts/
import quest_pose_v1 as qp  # noqa: E402
from quest_transport import stats_ms  # noqa: E402


def reasons(r: qp.PoseRecord) -> list[str]:
    g = r.poses["grip_r"]
    out = []
    if not r.focused or r.session_state != "FOCUSED":
        out.append("unfocused")
    if g.active is not True:
        out.append("pose_action_inactive")
    if not g.pos_valid:
        out.append("pos_invalid")
    if not g.ori_valid:
        out.append("ori_invalid")
    if g.pos_valid and not g.pos_tracked:
        out.append("pos_valid_untracked")
    if g.ori_valid and not g.ori_tracked:
        out.append("ori_valid_untracked")
    if r.inputs["in_r"]["squeeze"] is None:
        out.append("grip_action_inactive")
    if r.poses["head"].status != qp.TRACKED:
        out.append(f"head_{r.poses['head'].status}")
    return out


def main(base: Path) -> dict:
    L = [json.loads(x) for x in base.with_suffix(".jsonl").open()]
    marks = [(int(r.split(" ", 1)[0]), r.split(" ", 1)[1].strip()) for r in base.with_suffix(".marks.txt").read_text().splitlines()]
    t0 = next(t for t, l in marks if l == "phase_start baseline")
    rel = lambda h: round((h - t0) / 1e9, 3)  # noqa: E731
    P, other = [], []
    for r in L:
        if "error" in r:
            other.append(("parse_error", rel(r["host_ns"])))
            continue
        rec = qp.decode_record(r["raw"])
        if isinstance(rec, qp.PoseRecord):
            P.append({"t": rel(r["host_ns"]), "host": r["host_ns"], "rec": rec, "status": r.get("status_after"), "proc_ns": r.get("proc_ns")})
        else:
            other.append((type(rec).__name__, rel(r["host_ns"]), getattr(rec, "event", None)))
    out: dict = {"base": str(base), "marks_rel_s": {l: rel(t) for t, l in marks}, "non_pose_records": other}
    trans, prev = [], None
    for x in P:
        r, g = x["rec"], x["rec"].poses["grip_r"]
        key = (g.active, g.pos_valid, g.ori_valid, g.pos_tracked, g.ori_tracked, r.inputs["in_r"]["squeeze"] is None,
               r.focused, r.session_state, r.poses["head"].flags, x["status"])  # fmt: skip
        if key != prev:
            trans.append({"t": x["t"], "seq": r.seq, "t_query_ns": r.t_query_ns, "active": g.active, "pv": g.pos_valid, "ov": g.ori_valid,
                          "pt": g.pos_tracked, "ot": g.ori_tracked, "flags": g.flags, "grip": r.inputs["in_r"]["squeeze"],
                          "focused": r.focused, "state": r.session_state, "head_flags": r.poses["head"].flags,
                          "status_after": x["status"], "reasons": reasons(r), "proc_us": round((x["proc_ns"] or 0) / 1e3, 1)})  # fmt: skip
            prev = key
    out["transitions"] = trans
    # intervals of unavailability with all concurrent reasons
    ivs, cur = [], None
    for x in P:
        un = x["status"] != "FRESH_TRACKED"
        if un and cur is None:
            cur = {"start_t": x["t"], "start_seq": x["rec"].seq, "start_xr": x["rec"].t_query_ns, "n": 0, "reasons": {}, "statuses": {}}
            ivs.append(cur)
        if un:
            cur["end_t"], cur["end_seq"], cur["end_xr"] = x["t"], x["rec"].seq, x["rec"].t_query_ns
            cur["n"] += 1
            for rs in reasons(x["rec"]) or ["(none of the input reasons)"]:
                cur["reasons"][rs] = cur["reasons"].get(rs, 0) + 1
            cur["statuses"][x["status"]] = cur["statuses"].get(x["status"], 0) + 1
        else:
            cur = None
    for iv in ivs:
        iv["xr_span_ms"] = round((iv["end_xr"] - iv["start_xr"]) / 1e6, 1)
    out["unavailable_intervals"] = ivs
    # isolated valid-but-untracked samples: focused + action active + (pv&!pt or ov&!ot), and status
    iso = [x for x in P if x["rec"].focused and x["rec"].session_state == "FOCUSED" and x["rec"].poses["grip_r"].active is True
           and (("pos_valid_untracked" in reasons(x["rec"])) or ("ori_valid_untracked" in reasons(x["rec"])))]
    out["isolated_valid_untracked"] = {
        "count": len(iso),
        "first_t": iso[0]["t"] if iso else None, "last_t": iso[-1]["t"] if iso else None,
        "flag_combos": sorted({x["rec"].poses["grip_r"].flags for x in iso}),
        "statuses": sorted({x["status"] for x in iso}),
        "grip_action_inactive_in_any": any(x["rec"].inputs["in_r"]["squeeze"] is None for x in iso),
        "head_not_tracked_in_any": any(x["rec"].poses["head"].status != qp.TRACKED for x in iso),
    }
    # last available before first degradation, first available after
    first_un = next((i for i, x in enumerate(P) if x["status"] != "FRESH_TRACKED" and x["t"] > 0), None)
    if first_un is not None:
        lb = P[first_un - 1]
        out["last_available_before"] = {"t": lb["t"], "seq": lb["rec"].seq, "grip": lb["rec"].inputs["in_r"]["squeeze"]}
        nxt = next((i for i in range(first_un, len(P)) if P[i]["status"] == "FRESH_TRACKED"), None)
        if nxt is not None:
            fa = P[nxt]
            out["first_available_after"] = {"t": fa["t"], "seq": fa["rec"].seq, "grip": fa["rec"].inputs["in_r"]["squeeze"],
                                            "next5_grip": [p["rec"].inputs["in_r"]["squeeze"] for p in P[nxt:nxt + 5]]}  # fmt: skip
            # before/after pose difference (valid components only; controller physically moved)
            a, b = lb["rec"].poses["grip_r"], fa["rec"].poses["grip_r"]
            out["pose_before_after"] = {"before_p": a.p, "after_p": b.p, "dist_mm": round(1000 * math.dist(a.p, b.p), 1) if a.p and b.p else None,
                                        "note": "controller physically moved; NOT a drift/accuracy measurement"}  # fmt: skip
    grips_avail = [x["rec"].inputs["in_r"]["squeeze"] for x in P if x["status"] == "FRESH_TRACKED"]
    out["grip_available_min_max"] = (min(grips_avail), max(grips_avail)) if grips_avail else None
    hs = [x["host"] for x in P]
    out["host_interarrival"] = stats_ms([b - a for a, b in zip(hs, hs[1:])])
    out["proc_delay"] = stats_ms([x["proc_ns"] for x in P if x["proc_ns"]])
    seqs = sorted(x["rec"].seq for x in P)
    out["seq_missing"] = [s for a, b in zip(seqs, seqs[1:]) for s in range(a + 1, b)]
    out["counts"] = {"lines": len(L), "pose": len(P), "max_bytes": max(r["bytes"] for r in L)}
    return out


if __name__ == "__main__":
    for a in sys.argv[1:]:
        res = main(Path(a))
        Path(a).with_suffix(".test4.json").write_text(json.dumps(res, indent=1, default=str))
        print(json.dumps({k: v for k, v in res.items() if k != "transitions"}, indent=1, default=str))
        print("TRANSITIONS:")
        for t in res["transitions"]:
            print(" ", t)
