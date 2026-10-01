#!/usr/bin/env python3
"""Offline analysis for Step 4B Test 5 (recenter / LOCAL reference-space change).

Reads only the recorder JSONL (raw payloads + host receive ns) and cue marks.

Epoch rule (OpenXR XrEventDataReferenceSpaceChangePending): within ONE app run (sid), a pose
record belongs to LOCAL epoch k = number of `local_space_change_pending` events of that sid whose
integer changeTime <= the record's integer t_query_ns. Arrival order is irrelevant (an event may
arrive before or after poses on either side of its boundary). `other_space_change_pending`
(non-LOCAL; raw type not emitted by build s4a-ff0da2692500) never changes the LOCAL epoch.
Integer XrTime is compared directly; nothing is converted to float seconds before comparison.
poseInPreviousSpace is kept for diagnosis only (never applied); poseValid=false -> unknown.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
import quest_pose_v1 as qp  # noqa: E402
from quest_transport import stats_ms  # noqa: E402


@dataclass(frozen=True)
class LocalChange:
    sid: str
    seq: int
    change_time_ns: int
    host_ns: int | None
    pose_valid: bool | None
    pose_in_previous: dict[str, Any] | None  # raw, diagnostic only


def local_changes(events: list[tuple[int | None, qp.EventRecord, dict[str, Any]]]) -> list[LocalChange]:
    out = []
    for host, ev, raw in events:
        if ev.event == "local_space_change_pending":
            prev = raw.get("pose_in_previous_space") if ev.pose_valid else None  # unknown unless poseValid
            out.append(LocalChange(ev.sid, ev.seq, ev.time_ns, host, ev.pose_valid, prev))
    return sorted(out, key=lambda c: (c.sid, c.change_time_ns, c.seq))


def epoch_of(rec: qp.PoseRecord, changes: list[LocalChange]) -> int:
    """LOCAL epoch of a pose record by integer query time, same sid only."""
    return sum(1 for c in changes if c.sid == rec.sid and c.change_time_ns <= rec.t_query_ns)


def eligible(rec: qp.PoseRecord) -> bool:
    g, h = rec.poses["grip_r"], rec.poses["head"]
    return (rec.focused and rec.session_state == "FOCUSED" and g.active is True and g.status == qp.TRACKED
            and h.status == qp.TRACKED and rec.inputs["in_r"]["squeeze"] is not None)  # fmt: skip


def q_angle_deg(q1: tuple[float, ...], q2: tuple[float, ...]) -> float:
    d = abs(sum(a * b for a, b in zip(q1, q2))) / (math.sqrt(sum(a * a for a in q1)) * math.sqrt(sum(b * b for b in q2)))
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, d))))


def snapshot(host: int, rec: qp.PoseRecord, t0: int) -> dict[str, Any]:
    g, h = rec.poses["grip_r"], rec.poses["head"]
    return {"seq": rec.seq, "t_query_ns": rec.t_query_ns, "host_rel_s": round((host - t0) / 1e9, 3), "focused": rec.focused,
            "state": rec.session_state, "r_active": g.active, "r_flags": g.flags, "head_flags": h.flags,
            "grip": rec.inputs["in_r"]["squeeze"], "app_local_epoch_field": rec.local_epoch,
            "head_p": h.p, "head_q": h.q, "right_p": g.p, "right_q": g.q}  # fmt: skip


def analyze(base: Path) -> dict[str, Any]:
    L = [json.loads(x) for x in base.with_suffix(".jsonl").open()]
    mk = base.with_suffix(".marks.txt")
    marks = [(int(r.split(" ", 1)[0]), r.split(" ", 1)[1].strip()) for r in mk.read_text().splitlines()] if mk.exists() else []
    t0 = next((t for t, l in marks if l.startswith("phase_start")), L[0]["host_ns"])
    poses: list[tuple[int, qp.PoseRecord]] = []
    events: list[tuple[int | None, qp.EventRecord, dict[str, Any]]] = []
    errors = overflow = 0
    for r in L:
        if "error" in r:
            errors += 1
            continue
        rec = qp.decode_record(r["raw"])
        if isinstance(rec, qp.PoseRecord):
            poses.append((r["host_ns"], rec))
        elif isinstance(rec, qp.EventRecord):
            events.append((r["host_ns"], rec, json.loads(r["raw"])))
        elif isinstance(rec, qp.OverflowRecord):
            overflow += 1
    ch = local_changes(events)
    res: dict[str, Any] = {"base": str(base), "marks_rel_s": {l: round((t - t0) / 1e9, 3) for t, l in marks}}
    res["all_events"] = [{"host_rel_s": round((h - t0) / 1e9, 3), "seq": e.seq, "sid": e.sid, "event": e.event,
                          "time_ns": e.time_ns, "state": e.session_state, "pose_valid": e.pose_valid,
                          "local_epoch_field": e.local_epoch, "raw": raw} for h, e, raw in events]  # fmt: skip
    res["local_changes"] = []
    for c in ch:
        same = [(h, p) for h, p in poses if p.sid == c.sid]
        before = [(h, p) for h, p in same if p.t_query_ns < c.change_time_ns]
        after = [(h, p) for h, p in same if p.t_query_ns >= c.change_time_ns]
        eb = [(h, p) for h, p in before if eligible(p)]
        ea = [(h, p) for h, p in after if eligible(p)]
        info: dict[str, Any] = {"sid": c.sid, "seq": c.seq, "change_time_ns": c.change_time_ns,
                                "host_receipt_rel_s": None if c.host_ns is None else round((c.host_ns - t0) / 1e9, 3),
                                "pose_valid": c.pose_valid, "pose_in_previous_space": c.pose_in_previous,
                                "poses_before": len(before), "poses_after": len(after)}  # fmt: skip
        # arrival order vs boundary: poses received AFTER the event but queried BEFORE changeTime, and vice versa
        if c.host_ns is not None:
            info["received_after_event_but_old_epoch"] = sum(1 for h, p in before if h > c.host_ns)
            info["received_before_event_but_new_epoch"] = sum(1 for h, p in after if h < c.host_ns)
        if eb and ea:
            (hb, pb), (ha, pa) = eb[-1], ea[0]
            info["nearest_eligible_before"] = snapshot(hb, pb, t0)
            info["nearest_eligible_after"] = snapshot(ha, pa, t0)
            info["query_gap_ms_between_them"] = round((pa.t_query_ns - pb.t_query_ns) / 1e6, 3)
            for k, lab in (("head", "head"), ("grip_r", "right")):
                a, b = pb.poses[k], pa.poses[k]
                info[f"{lab}_coordinate_change"] = {"pos_mm": round(1000 * math.dist(a.p, b.p), 1),
                                                    "delta_p_mm": [round(1000 * (y - x), 1) for x, y in zip(a.p, b.p)],
                                                    "ang_deg": round(q_angle_deg(a.q, b.q), 2)}  # fmt: skip
            # immediately adjacent records (any eligibility) across the boundary
            lb, la = before[-1], after[0]
            info["adjacent_records"] = {"last_before": snapshot(lb[0], lb[1], t0), "first_after": snapshot(la[0], la[1], t0)}
        # unavailable records within +/- 1 s of the boundary (by query time)
        win = [(h, p) for h, p in same if abs(p.t_query_ns - c.change_time_ns) <= 1_000_000_000]
        info["unavailable_within_1s_of_boundary"] = sum(1 for h, p in win if not eligible(p))
        info["records_within_1s_of_boundary"] = len(win)
        res["local_changes"].append(info)
    # epoch summary
    ep: dict[str, dict[int, int]] = {}
    for h, p in poses:
        e = epoch_of(p, ch)
        ep.setdefault(p.sid, {}).setdefault(e, 0)
        ep[p.sid][e] += 1
    res["pose_epochs_by_sid"] = {s: {str(k): v for k, v in d.items()} for s, d in ep.items()}
    # Note: the app's own pose field `local_epoch` counts change events RECEIVED by the app (incremented at
    # receipt), not effective changeTime; it is reported in snapshots but not used for classification.
    # continuity / stream
    seqs = sorted([p.seq for _, p in poses] + [e.seq for _, e, _ in events])
    res["seq_missing"] = [s for a, b in zip(seqs, seqs[1:]) for s in range(a + 1, b)]
    hs = sorted([h for h, _ in poses] + [h for h, _, _ in events if h is not None])
    res["host_interarrival"] = stats_ms([b - a for a, b in zip(hs, hs[1:])])
    res["counts"] = {"lines": len(L), "pose": len(poses), "events": len(events), "parse_errors": errors, "overflow": overflow,
                     "max_bytes": max((r.get("bytes", 0) for r in L), default=0)}  # fmt: skip
    # availability transitions (raw-input status from the recorder)
    trans, prev = [], None
    for r in L:
        if r.get("type") != "pose":
            continue
        s = r.get("status_after")
        if s != prev:
            rec = qp.decode_record(r["raw"])
            trans.append({"host_rel_s": round((r["host_ns"] - t0) / 1e9, 3), "seq": rec.seq, "status": s,
                          "focused": rec.focused, "r_active": rec.poses["grip_r"].active, "r_flags": rec.poses["grip_r"].flags,
                          "grip": rec.inputs["in_r"]["squeeze"], "epoch": epoch_of(rec, ch)})  # fmt: skip
            prev = s
    res["status_transitions"] = trans
    return res


if __name__ == "__main__":
    for a in sys.argv[1:]:
        out = analyze(Path(a))
        Path(a).with_suffix(".test5.json").write_text(json.dumps(out, indent=1, default=str))
        print(json.dumps(out, indent=1, default=str))
