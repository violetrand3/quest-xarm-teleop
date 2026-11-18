#!/usr/bin/env python3
"""Offline analysis for Step 4B Test 2 (stationary right controller, moving head).

Inputs: a recorder JSONL (<base>.jsonl) and its cue marks (<base>.marks.txt).
Nothing here touches the device; raw records are never modified.

Conventions (docs/step04a_openxr_reader.md, verified in tests/test_quest_pose_v1.py):
  T_LOCAL_X = pose_matrix(p, q): 4x4, column-vector convention, maps X-frame points into
  LOCAL (p_L = R(q) p_X + p). Head-relative: T_HEAD_RIGHT = inverse(T_LOCAL_HEAD) @ T_LOCAL_RIGHT,
  computed only from head and right poses of the SAME record (same t_query_ns).

Eligibility (common set for the three-way comparison): pose record decoded and accepted;
focused and session_state FOCUSED; right pose action active; head and right grip status
TRACKED (position+orientation VALID and TRACKED) with finite p/q (decoder-enforced).

Baseline rule (fixed per trial, chosen before looking at movement): the FIRST eligible record
whose host receive time is >= (still_1 phase-start mark + 2.0 s) and < move phase-start mark.
Not reset later. Phases are delimited by the host-side cue marks (same host monotonic clock as
the recorder's host_ns). Note: phase marks are set when each spoken cue STARTS.

Angular distance: quaternions -> 2*acos(|<q1,q2>|) (sign-invariant); head-relative rotation
matrices -> acos((trace(R1^T R2) - 1) / 2). Both clamped to [-1, 1] before acos. No Euler angles.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))  # archived: the modules stay in scripts/
import quest_pose_v1 as qp  # noqa: E402

BASELINE_OFFSET_S = 2.0


def q_angle_deg(q1: tuple[float, ...], q2: tuple[float, ...]) -> float:
    n1 = math.sqrt(sum(x * x for x in q1))
    n2 = math.sqrt(sum(x * x for x in q2))
    d = abs(sum(a * b for a, b in zip(q1, q2))) / (n1 * n2)
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, d))))


def r_angle_deg(a: list[list[float]], b: list[list[float]]) -> float:
    tr = sum(a[k][i] * b[k][i] for i in range(3) for k in range(3))  # trace(Ra^T Rb)
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2))))


def stat(xs: list[float]) -> dict[str, Any]:
    if not xs:
        return {"n": 0}
    s = sorted(xs)
    p95 = s[min(len(s) - 1, max(0, math.ceil(0.95 * len(s)) - 1))]
    return {"n": len(s), "median": round(statistics.median(s), 2), "p95": round(p95, 2), "max": round(s[-1], 2)}


def exclusion_reason(rec: qp.PoseRecord) -> str | None:
    if not rec.focused or rec.session_state != "FOCUSED":
        return "unfocused"
    g, h = rec.poses["grip_r"], rec.poses["head"]
    if g.active is not True:
        return "right_action_inactive"
    if not (g.pos_valid and g.ori_valid) or g.p is None or g.q is None:
        return "right_not_valid"
    if not (g.pos_tracked and g.ori_tracked):
        return "right_valid_untracked(" + ("pos" if not g.pos_tracked else "") + ("ori" if not g.ori_tracked else "") + ")"
    if h.status != qp.TRACKED:
        return f"head_{h.status}"
    return None


def intervals(items: list[tuple[int, int, bool]]) -> list[dict[str, Any]]:
    """items: (host_ns, t_query_ns, flag) per pose record -> intervals where flag is True."""
    out, cur = [], None
    for host, tq, flag in items:
        if flag and cur is None:
            cur = [host, host, tq, tq, 1]
        elif flag:
            cur[1], cur[3], cur[4] = host, tq, cur[4] + 1
        elif cur is not None:
            out.append(cur)
            cur = None
    if cur is not None:
        out.append(cur)
    return [{"host_start_ns": a, "host_end_ns": b, "xr_start_ns": c, "xr_end_ns": d, "records": n,
             "xr_span_ms": round((d - c) / 1e6, 1)} for a, b, c, d, n in out]  # fmt: skip


def analyze(base: Path) -> dict[str, Any]:
    lines = [json.loads(x) for x in base.with_suffix(".jsonl").open()]
    marks: dict[str, int] = {}
    order: list[tuple[str, int]] = []
    for row in base.with_suffix(".marks.txt").read_text().splitlines():
        t, label = row.split(" ", 1)
        marks.setdefault(label, int(t))
        if label.startswith("phase_start "):
            order.append((label.split(" ", 1)[1], int(t)))
    # The measurement ends at phase_end, or at the first input_lost mark (a later return of input
    # is NOT treated as a continuation of the measurement).
    lost = [v for k, v in marks.items() if k.startswith("input_lost")]
    t_end = min(lost) if lost else marks.get("phase_end")
    if not order or t_end is None:
        return {"base": str(base), "error": "phase marks incomplete", "marks": marks}
    bounds = [(name, start, (order[i + 1][1] if i + 1 < len(order) else t_end)) for i, (name, start) in enumerate(order)]
    bounds = [(n, a, min(b, t_end)) for n, a, b in bounds if a < t_end]
    t0 = order[0][1]
    t_s1 = t0
    base_phase = next(((a, b) for n, a, b in bounds if n == "still_1"), (bounds[0][1], bounds[0][2]))
    poses: list[tuple[int, qp.PoseRecord]] = []
    parse_err = overflow = 0
    events = []
    seqs = []
    for r in lines:
        if "error" in r:
            parse_err += 1
            continue
        rec = qp.decode_record(r["raw"])
        seqs.append(rec.seq)
        if isinstance(rec, qp.PoseRecord):
            poses.append((r["host_ns"], rec))
        elif isinstance(rec, qp.EventRecord):
            events.append({"host_rel_s": round((r["host_ns"] - t0) / 1e9, 3), "seq": rec.seq, "event": rec.event,
                           "time_ns": rec.time_ns, "state": rec.session_state})  # fmt: skip
        elif isinstance(rec, qp.OverflowRecord):
            overflow += 1
    sids = sorted({p.sid for _, p in poses})
    # LOCAL changes inside this file (session-start change precedes these recordings)
    local_changes = [e for e in events if e["event"] == "local_space_change_pending"]
    trial_xr = [p.t_query_ns for h, p in poses if t_s1 <= h < t_end]
    xr_lo, xr_hi = (min(trial_xr), max(trial_xr)) if trial_xr else (0, 0)
    change_in_trial = [e for e in local_changes if xr_lo <= e["time_ns"] <= xr_hi]

    def phase_of(h: int) -> str | None:
        for n, a, b in bounds:
            if a <= h < b:
                return n
        return None

    # baseline
    base_rec = None
    for h, p in poses:
        if base_phase[0] + int(BASELINE_OFFSET_S * 1e9) <= h < base_phase[1] and exclusion_reason(p) is None:
            base_rec = (h, p)
            break
    res: dict[str, Any] = {"base": str(base), "sids": sids, "builds": sorted({p.build for _, p in poses}),
                           "parse_errors": parse_err, "overflow": overflow, "local_changes_in_file": local_changes,
                           "local_change_within_trial_xr_window": change_in_trial}  # fmt: skip
    if base_rec is None:
        res["verdict"] = "INCONCLUSIVE: no eligible baseline record in still_1 (>= 2 s after its start)"
    bh, bp = base_rec if base_rec else (None, None)
    if bp is not None:
        T_bh, T_br = bp.poses["head"].matrix(), bp.poses["grip_r"].matrix()
        T_bhr = qp.matmul(qp.invert_rigid(T_bh), T_br)  # type: ignore[arg-type]
        res["baseline"] = {"host_rel_s": round((bh - t0) / 1e9, 3), "seq": bp.seq, "t_query_ns": bp.t_query_ns,
                           "head_p": bp.poses["head"].p, "right_p": bp.poses["grip_r"].p}  # fmt: skip
    phases: dict[str, Any] = {}
    for ph, ph_a, ph_b in bounds:
        recs = [(h, p) for h, p in poses if phase_of(h) == ph]
        sq_all = [p.inputs["in_r"]["squeeze"] for _, p in recs]
        excl: dict[str, int] = {}
        hd, ha, rd, ra, hrd, hra = [], [], [], [], [], []
        elig_t = []
        for h, p in recs:
            why = exclusion_reason(p)
            if why:
                excl[why] = excl.get(why, 0) + 1
                continue
            elig_t.append(h)
            if bp is None:
                continue
            H, R = p.poses["head"], p.poses["grip_r"]
            hd.append(1000 * math.dist(H.p, bp.poses["head"].p))  # type: ignore[arg-type]
            ha.append(q_angle_deg(H.q, bp.poses["head"].q))  # type: ignore[arg-type]
            rd.append(1000 * math.dist(R.p, bp.poses["grip_r"].p))  # type: ignore[arg-type]
            ra.append(q_angle_deg(R.q, bp.poses["grip_r"].q))  # type: ignore[arg-type]
            T_hr = qp.matmul(qp.invert_rigid(H.matrix()), R.matrix())  # type: ignore[arg-type]
            hrd.append(1000 * math.dist([T_hr[i][3] for i in range(3)], [T_bhr[i][3] for i in range(3)]))
            hra.append(r_angle_deg(T_hr, T_bhr))
        gaps = [(b - a) / 1e6 for a, b in zip(elig_t, elig_t[1:])]
        dur = (ph_b - ph_a) / 1e9
        sq_vals = [v for v in sq_all if v is not None]
        phases[ph] = {
            "right_grip_squeeze_value": stat(sq_vals),
            "right_grip_squeeze_min": round(min(sq_vals), 4) if sq_vals else None,
            "right_squeeze_action_active_records": len(sq_vals),
            "right_squeeze_null_records": len(sq_all) - len(sq_vals),
            "duration_s": round(dur, 2), "pose_records": len(recs), "eligible": len(elig_t),
            "coverage": round(len(elig_t) / len(recs), 3) if recs else None,
            "longest_gap_between_eligible_ms": round(max(gaps), 1) if gaps else None,
            "excluded": excl,
            "head_pos_mm": stat(hd), "head_ang_deg": stat(ha),
            "right_local_pos_mm": stat(rd), "right_local_ang_deg": stat(ra),
            "right_head_rel_pos_mm": stat(hrd), "right_head_rel_ang_deg": stat(hra),
        }  # fmt: skip
    res["phases"] = phases
    # transitions / intervals over the whole file (host and XrTime)
    items = [(h, p) for h, p in poses]
    res["right_inactive_intervals"] = intervals([(h, p.t_query_ns, p.poses["grip_r"].active is not True) for h, p in items])
    trans = []
    prev = None
    for h, p in items:
        g, hd_ = p.poses["grip_r"], p.poses["head"]
        key = (g.active, g.pos_valid, g.ori_valid, g.pos_tracked, g.ori_tracked, hd_.flags, p.focused, p.session_state)
        if key != prev:
            trans.append({"host_rel_s": round((h - t0) / 1e9, 3), "t_query_ns": p.t_query_ns, "seq": p.seq,
                          "r_active": g.active, "r_pv": g.pos_valid, "r_ov": g.ori_valid, "r_pt": g.pos_tracked,
                          "r_ot": g.ori_tracked, "head_flags": hd_.flags, "focused": p.focused, "state": p.session_state})  # fmt: skip
            prev = key
    res["transitions"] = trans
    res["events"] = events
    # sequence continuity (pose + event share one counter)
    s_sorted = sorted(seqs)
    missing = [s for a, b in zip(s_sorted, s_sorted[1:]) for s in range(a + 1, b)]
    res["seq_missing"] = missing
    res["seq_note"] = "pose and event records share one counter; a missing seq's record type is unknown"
    hs = [h for h, _ in poses]
    res["host_pose_interarrival_ms"] = stat([(b - a) / 1e6 for a, b in zip(hs, hs[1:])])
    xs = [p.t_query_ns for _, p in poses]
    res["xr_query_step_ms"] = stat([(b - a) / 1e6 for a, b in zip(xs, xs[1:])])
    res["marks_rel_s"] = {k: round((v - t0) / 1e9, 3) for k, v in marks.items()}
    res["measurement_end"] = "input_lost" if lost else "phase_end"
    res["profile_paths_in_telemetry"] = None  # interaction_profile_changed carries no path in schema v1
    return res


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        out = analyze(Path(arg))
        Path(arg).with_suffix(".analysis.json").write_text(json.dumps(out, indent=1))
        print(json.dumps({k: out[k] for k in out if k not in ("transitions", "events", "right_inactive_intervals")}, indent=1))
