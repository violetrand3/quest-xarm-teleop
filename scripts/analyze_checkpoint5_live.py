#!/usr/bin/env python3
"""Analyse one checkpoint-5 live run (new-reader diagnostic transport, app restart, acquisition pause).

DIAGNOSTIC ONLY. Reads saved logs; talks to nothing. Sender-clock and receiver-clock
times are never subtracted from each other: packets are matched by sequence number.
"""
from __future__ import annotations

import collections
import json
import re
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import quest_pose_v1 as qp  # noqa: E402


def stats(v):
    if not v:
        return None
    s = sorted(v)
    return {"n": len(s), "median": round(statistics.median(s), 1), "p95": round(s[int(0.95 * (len(s) - 1))], 1), "max": round(s[-1], 1)}


def main() -> int:
    d, name = Path(sys.argv[1]), sys.argv[2]
    marks = [(int(a), b) for a, b in (ln.rstrip("\n").split(" ", 1) for ln in open(d / f"marks_{name}.txt"))]
    M = {b.split(" (")[0]: a for a, b in marks}
    t0 = marks[0][0]
    rel = lambda ns: round((ns - t0) / 1e9, 3)  # noqa: E731  (sender host monotonic, s after first mark)
    src = [json.loads(ln) for ln in open(d / f"src_{name}.jsonl")]
    for r in src:
        r["rec"] = json.loads(r["raw"])
    tx = [json.loads(ln) for ln in open(d / f"tx_{name}.jsonl")]
    rx = [json.loads(ln) for ln in open(d / f"rx5_{name}.jsonl")]
    out = {"run": name, "marks_s": [(rel(a), b) for a, b in marks]}

    # --- source identities, counts, gaps -------------------------------------------------
    ids = collections.OrderedDict()
    gaps = []
    for r in src:
        k = (r["gen"], r["rec"]["sid"])
        e = ids.setdefault(k, {"acq_gen": k[0], "sid": k[1], "records": 0, "poses": 0, "available": 0, "events": 0,
                               "first_s": rel(r["host_ns"]), "first_seq": r["rec"]["seq"], "prev": None})
        if e["prev"] is not None and r["rec"]["seq"] != e["prev"] + 1:
            gaps.append({"sid": k[1][:8], "after_seq": e["prev"], "next_seq": r["rec"]["seq"],
                         "missing": r["rec"]["seq"] - e["prev"] - 1, "t_s": rel(r["host_ns"])})
        e["prev"] = r["rec"]["seq"]
        e["records"] += 1
        e["poses" if r["kind"] == "pose" else "events"] += 1
        e["available"] += bool(r["available"])
        e["last_s"], e["last_seq"] = rel(r["host_ns"]), r["rec"]["seq"]
    for e in ids.values():
        e.pop("prev")
        e["first_seq_missing_before"] = e["first_seq"] - 1
    out["source_identities"] = list(ids.values())
    out["source_gaps"] = {"count": len(gaps), "missing_records": sum(g["missing"] for g in gaps), "detail": gaps}

    # --- packets matched by transport seq --------------------------------------------------
    txp = {p["seq"]: p for p in tx if p.get("dir") == "out"}
    rxp = {p["seq"]: p for p in rx if p.get("type") in ("pose", "status", "source_event", "bye")}
    out["packets"] = {"sent": len(txp), "received": len(rxp), "sent_not_received": sorted(set(txp) - set(rxp)),
                      "received_not_sent": sorted(set(rxp) - set(txp)),
                      "replies": dict(collections.Counter(p["reply"] for p in rxp.values())),
                      "rejects": [p for p in rxp.values() if p.get("reject")]}
    src_by = {(r["rec"]["sid"], r["rec"]["seq"]): r for r in src}
    sent_pose = {(p["sid"], p["source_seq"]) for p in rxp.values() if p["type"] == "pose"}
    out["poses"] = {"source": sum(r["kind"] == "pose" for r in src), "received": len(sent_pose),
                    "received_not_in_source_log": len(sent_pose - set(src_by)),
                    "decimated_reported_to_receiver": sum(p["decimated"] or 0 for p in rxp.values() if p["type"] == "pose"),
                    "source_minus_received": sum(r["kind"] == "pose" for r in src) - len(sent_pose)}
    # availability flag carried == availability decided at source for the same record
    out["poses"]["availability_mismatch"] = sum(
        1 for p in rxp.values() if p["type"] == "pose" and bool(src_by[(p["sid"], p["source_seq"])]["available"]) != bool(p["available"]))

    # --- every continuity generation step at the source reaches the receiver ---------------
    steps, prev = [], 0
    for r in src:
        if r["cont_gen"] != prev:
            steps.append({"t_s": rel(r["host_ns"]), "to_gen": r["cont_gen"], "kind": r["kind"],
                          "carried_by_own_packet": (r["rec"]["sid"], r["rec"]["seq"]) in sent_pose or r["kind"] == "event"})
            prev = r["cont_gen"]
    rgen, seen, order_ok = None, 0, True
    listed = collections.Counter()
    for s in sorted(rxp):
        p = rxp[s]
        if p.get("gen") is None:
            continue
        if rgen is not None and p["gen"] < rgen:
            order_ok = False
        if rgen is not None and p["gen"] != rgen:
            seen += 1
            if len(p["breaks"] or []) < p["gen"] - rgen:
                listed["step_with_missing_reasons"] += 1
        listed.update(p["breaks"] or [])
        rgen = p["gen"]
    out["continuity"] = {
        "source_log_generation_steps_on_records": len(steps),
        "of_which_on_a_decimated_pose": sum(1 for s in steps if not s["carried_by_own_packet"]),
        "receiver_generation_changes": seen, "receiver_last_gen": rgen, "receiver_gen_monotonic": order_ok,
        "receiver_break_reasons_listed": dict(listed),
        "available_poses_marked_continuous": sum(1 for p in rxp.values() if p["type"] == "pose" and p["continuous"]),
        "first_pose_of_each_identity_continuous": {},
    }
    firsts = {}
    for s in sorted(rxp):
        p = rxp[s]
        if p["type"] == "pose" and (p["acq_gen"], p["sid"]) not in firsts:
            firsts[(p["acq_gen"], p["sid"])] = p
    out["continuity"]["first_pose_of_each_identity_continuous"] = {
        f"acq{k[0]}/{k[1][:8]}": {"source_seq": p["source_seq"], "gen": p["gen"], "continuous": p["continuous"], "available": p["available"]}
        for k, p in firsts.items()}

    # --- normal window (by sender seq, so no cross-clock subtraction) -----------------------
    lo, hi = M["normal_start"], M["normal_end"]
    wsrc = [r for r in src if lo <= r["host_ns"] < hi and r["kind"] == "pose"]
    wtx = [p for p in txp.values() if lo <= p["t"] < hi]
    wseq = {p["seq"] for p in wtx if p["type"] == "pose"}
    wrx = sorted((rxp[s] for s in wseq if s in rxp), key=lambda p: p["seq"])
    src_dt = [(b["host_ns"] - a["host_ns"]) / 1e6 for a, b in zip(wsrc, wsrc[1:])]
    rx_dt = [(b["t"] - a["t"]) / 1e6 for a, b in zip(wrx, wrx[1:])]
    tx_dt = [(b["t"] - a["t"]) / 1e6 for a, b in zip(sorted((p for p in wtx if p["type"] == "pose"), key=lambda p: p["seq"]),
                                                    sorted((p for p in wtx if p["type"] == "pose"), key=lambda p: p["seq"])[1:])]
    out["normal_window"] = {
        "duration_s": round((hi - lo) / 1e9, 2), "source_poses": len(wsrc), "source_available": sum(bool(r["available"]) for r in wsrc),
        "source_rate_hz": round(len(wsrc) / ((hi - lo) / 1e9), 1), "pose_packets_sent": len(wseq), "pose_packets_received": len(wrx),
        "sent_rate_hz": round(len(wseq) / ((hi - lo) / 1e9), 1), "received_available": sum(bool(p["available"]) for p in wrx),
        "received_continuous": sum(bool(p["continuous"]) for p in wrx), "other_packets_sent": len(wtx) - len(wseq),
        "continuity_gens_in_window": sorted({p["gen"] for p in wrx}),
        "source_gaps_in_window": [g for g in gaps if rel(lo) <= g["t_s"] < rel(hi)],
        "source_host_arrival_dt_ms(sender clock)": stats(src_dt), "pose_send_dt_ms(sender clock)": stats(tx_dt),
        "pose_arrival_dt_ms(receiver clock)": stats(rx_dt),
    }

    # --- phases: what the receiver was told between marks -----------------------------------
    def rx_between(a_ns, b_ns):
        seqs = sorted(p["seq"] for p in txp.values() if a_ns <= p["t"] < b_ns)
        c = collections.Counter()
        for s in seqs:
            p = rxp.get(s)
            if p is None:
                c["not_received"] += 1
            elif p["type"] == "pose":
                c[f"pose avail={p['available']} sid={p['sid'][:8]} acq={p['acq_gen']}"] += 1
            elif p["type"] == "status":
                c[f"status source_up={p['source_up']} stale={p['stale']}"] += 1
            else:
                c[p["type"]] += 1
        return dict(c)

    def last_src(before_ns, pred=lambda r: True):
        c = [r for r in src if r["host_ns"] <= before_ns and pred(r)]
        return c[-1] if c else None

    def first_src(after_ns, pred=lambda r: True):
        return next((r for r in src if r["host_ns"] > after_ns and pred(r)), None)

    summ = json.load(open(d / f"tx_{name}.summary.json"))
    stale = summ["stale_detections"]
    A = {"stop": M["force-stop app"], "relaunch": M["relaunch app"], "sigstop": next(a for a, b in marks if b.startswith("SIGSTOP")),
         "abort": next(a for a, b in marks if b.startswith("abort generation")), "stop2": M["force-stop app before new generation"],
         "newgen": next(a for a, b in marks if b.startswith("start new acquisition")), "launch3": M["launch app"] if False else next(a for a, b in marks if b.startswith("launch app (session 3")),
         "end": M["end"]}
    avail = lambda r: bool(r["available"])  # noqa: E731

    # app stop / restart
    lr, la = last_src(A["relaunch"], lambda r: r["host_ns"] < A["relaunch"]), last_src(A["relaunch"], avail)
    f2, f2a = first_src(A["relaunch"]), first_src(A["relaunch"], avail)
    st = next(s for s in stale if s["detected_host_mono_ns"] > A["stop"])
    out["app_restart"] = {
        "force_stop_s": rel(A["stop"]), "relaunch_s": rel(A["relaunch"]),
        "last_record_before": {"t_s": rel(lr["host_ns"]), "sid": lr["rec"]["sid"][:8], "seq": lr["rec"]["seq"], "kind": lr["kind"], "available": avail(lr)},
        "last_available_before": {"t_s": rel(la["host_ns"]), "seq": la["rec"]["seq"], "ms_after_force_stop_mark": round((la["host_ns"] - A["stop"]) / 1e6, 1)},
        "stale_detected": {"t_s": rel(st["detected_host_mono_ns"]), "ms_after_last_valid": st["delay_ms"], "input_was_still_available": st["had_available_input"]},
        "tx_status_reasons_stop_to_relaunch": dict(collections.Counter(p.get("reason") for p in txp.values() if A["stop"] <= p["t"] < A["relaunch"] and p["type"] == "status")),
        "receiver_saw_stop_to_relaunch": rx_between(la["host_ns"] + 1, A["relaunch"]),
        "first_record_after": {"t_s": rel(f2["host_ns"]), "sid": f2["rec"]["sid"][:8], "seq": f2["rec"]["seq"], "kind": f2["kind"], "event": f2["rec"].get("event")},
        "first_available_after": {"t_s": rel(f2a["host_ns"]), "sid": f2a["rec"]["sid"][:8], "seq": f2a["rec"]["seq"], "local_epoch": f2a["rec"]["local_epoch"]},
        "old_sid_records_after_relaunch": sum(1 for r in src if r["host_ns"] > A["relaunch"] and r["rec"]["sid"] == lr["rec"]["sid"]),
        "input_unavailable_span_s(sender clock)": round((f2a["host_ns"] - la["host_ns"]) / 1e9, 3),
    }

    # acquisition pause
    lp, lpa = last_src(A["newgen"]), last_src(A["newgen"], avail)
    sp = next(s for s in stale if s["detected_host_mono_ns"] > A["sigstop"])
    f3, f3a = first_src(A["newgen"]), first_src(A["newgen"], avail)
    out["acquisition_pause"] = {
        "sigstop_s": rel(A["sigstop"]), "abort_s": rel(A["abort"]), "new_generation_s": rel(A["newgen"]), "launch3_s": rel(A["launch3"]),
        "last_record_from_paused_child": {"t_s": rel(lp["host_ns"]), "gen": lp["gen"], "sid": lp["rec"]["sid"][:8], "seq": lp["rec"]["seq"],
                                           "ms_relative_to_sigstop_mark": round((lp["host_ns"] - A["sigstop"]) / 1e6, 1)},
        "last_available": {"t_s": rel(lpa["host_ns"]), "seq": lpa["rec"]["seq"]},
        "watchdog": {"detected_s": rel(sp["detected_host_mono_ns"]), "ms_after_last_valid_host_received_sample": sp["delay_ms"],
                     "ms_after_sigstop_mark": round((sp["detected_host_mono_ns"] - A["sigstop"]) / 1e6, 1),
                     "input_was_available_when_detected": sp["had_available_input"], "acq_gen": sp["acq_gen"]},
        "records_host_received_between_sigstop_and_new_generation": sum(1 for r in src if lp["host_ns"] < r["host_ns"] <= A["newgen"]),
        "records_from_acq_gen_1_after_abort": sum(1 for r in src if r["gen"] == 1 and r["host_ns"] > A["abort"]),
        "records_of_paused_sid_after_sigstop": sum(1 for r in src if r["rec"]["sid"] == lp["rec"]["sid"] and r["host_ns"] > lp["host_ns"]),
        "stale_generation_lines_discarded": summ["source"]["stale_generation_lines_discarded"],
        "tx_status_reasons_pause_to_first_new_record": dict(collections.Counter(
            p.get("reason") for p in txp.values() if lp["host_ns"] < p["t"] < f3["host_ns"] and p["type"] == "status")),
        "receiver_saw_pause_to_first_new_record": rx_between(lp["host_ns"] + 1, f3["host_ns"]),
        "transport_packets_acked_during_pause": sum(1 for p in txp.values() if lp["host_ns"] < p["t"] < f3["host_ns"] and p["seq"] in rxp),
        "first_record_new_generation": {"t_s": rel(f3["host_ns"]), "gen": f3["gen"], "sid": f3["rec"]["sid"][:8], "seq": f3["rec"]["seq"], "event": f3["rec"].get("event")},
        "first_available_new_generation": {"t_s": rel(f3a["host_ns"]), "gen": f3a["gen"], "sid": f3a["rec"]["sid"][:8], "seq": f3a["rec"]["seq"]},
        "input_unavailable_span_s(sender clock)": round((f3a["host_ns"] - lpa["host_ns"]) / 1e9, 3),
        "acquisition_events": summ["acquisition_events"],
    }
    out["stale_detections"] = [dict(s, detected_s=rel(s["detected_host_mono_ns"])) for s in stale]

    # --- receiver console values vs. source record (independent end-to-end spot check) -----
    pat = re.compile(r"input=PRESENT .*?src=acq(\d+)/([0-9a-f]{8}) seq=(\d+) .*?R\.pos=\(([-+\d.]+),([-+\d.]+),([-+\d.]+)\) grip=([\d.]+) trig=([\d.]+) A=(\d) B=(\d) flags=(\d+)")
    short = {(r["gen"], r["rec"]["sid"][:8], r["rec"]["seq"]): r for r in src}
    n = bad = 0
    for ln in open(d / f"rx_{name}.console.txt"):
        m = pat.search(ln)
        if not m:
            continue
        n += 1
        r = short.get((int(m[1]), m[2], int(m[3])))
        if r is None:
            bad += 1
            continue
        rec = qp.decode_record(r["raw"])
        g, i = rec.poses["grip_r"], rec.inputs["in_r"]
        ok = (all(abs(float(m[4 + k]) - g.p[k]) <= 0.00051 for k in range(3)) and abs(float(m[7]) - i["squeeze"]) <= 0.0051
              and abs(float(m[8]) - i["trigger"]) <= 0.0051 and int(m[9]) == int(bool(i["a"])) and int(m[10]) == int(bool(i["b"]))
              and int(m[11]) == g.flags)
        bad += not ok
    out["receiver_console_vs_source"] = {"present_lines_checked": n, "mismatches": bad}
    out["sender_summary"] = {k: summ[k] for k in ("rtt_sender_clock", "acquisition_to_send_age_sender_clock", "sent", "continuity", "robot_connection_attempted")}
    rs = json.load(open(d / f"rx5_{name}.summary.json"))
    out["receiver_summary"] = {"errors_total": rs["errors_total"], "robot_connection_attempted": rs.get("robot_connection_attempted"),
                               **{k: rs["sessions"][0][k] for k in ("rejected", "breaks_not_listed", "identity_changed_without_generation",
                                                                    "packet_interarrival_receiver_clock", "available_interarrival_receiver_clock")}}
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
