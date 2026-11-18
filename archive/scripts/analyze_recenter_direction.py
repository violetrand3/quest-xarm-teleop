#!/usr/bin/env python3
"""Checkpoint 5, part A: which way does the recorded poseInPreviousSpace (E) map coordinates?

DIAGNOSTIC ONLY. Reads saved Step 4B Test 5 recordings. Nothing here is applied to live data:
no recenter compensation exists anywhere in the pipeline.

For each LOCAL change event with poseValid, the two adjacent pose records with the largest head
coordinate jump near the event are taken as (old, new). Both candidate relations are evaluated
for the head and the right grip pose:
    hypothesis A:  T_new = inverse(E) * T_old      (E = T_old_new, as the event field name suggests)
    hypothesis B:  T_new = E * T_old
Residuals include whatever physical motion happened between the two records (one frame).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))  # archived: the modules stay in scripts/
import quest_pose_v1 as qp  # noqa: E402


def resid(a, b):
    d = qp.matmul(qp.invert_rigid(a), b)
    tr = max(-1.0, min(3.0, d[0][0] + d[1][1] + d[2][2]))
    return round(1000 * math.dist([r[3] for r in a[:3]], [r[3] for r in b[:3]]), 2), round(math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))), 3)


def main() -> int:
    out = []
    for f in sys.argv[1:]:
        rows = [json.loads(x) for x in open(f)]
        recs = [(r["host_ns"], qp.decode_record(r["raw"]), json.loads(r["raw"])) for r in rows if "error" not in r]
        poses = [(h, r) for h, r, _ in recs if isinstance(r, qp.PoseRecord) and r.poses["head"].p is not None]
        for h, ev, raw in recs:
            if not (isinstance(ev, qp.EventRecord) and ev.event == "local_space_change_pending" and ev.pose_valid):
                continue
            e = raw["pose_in_previous_space"]
            E = qp.pose_matrix(tuple(e["p"]), tuple(e["q"]))
            near = [i for i in range(1, len(poses)) if poses[i][1].sid == ev.sid == poses[i - 1][1].sid
                    and abs(poses[i][1].t_query_ns - ev.time_ns) < 300_000_000]
            if not near:
                continue
            i = max(near, key=lambda k: math.dist(poses[k][1].poses["head"].p, poses[k - 1][1].poses["head"].p))
            old, new = poses[i - 1][1], poses[i][1]
            row = {"file": Path(f).name, "event_seq": ev.seq, "change_time_ns": ev.time_ns, "E_p": e["p"], "E_q": e["q"],
                   "old_seq": old.seq, "new_seq": new.seq, "query_gap_ms": round((new.t_query_ns - old.t_query_ns) / 1e6, 2),
                   "switch_after_change_time_ms": round((new.t_query_ns - ev.time_ns) / 1e6, 2)}
            for k in ("head", "grip_r"):
                a, b = old.poses[k], new.poses[k]
                if a.p is None or b.p is None:
                    continue
                To, Tn = qp.pose_matrix(a.p, a.q), qp.pose_matrix(b.p, b.q)
                row[k] = {"raw_jump_mm_deg": resid(To, Tn),
                          "A_inverse(E)*T_old_residual_mm_deg": resid(qp.matmul(qp.invert_rigid(E), To), Tn),
                          "B_E*T_old_residual_mm_deg": resid(qp.matmul(E, To), Tn)}
            out.append(row)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
