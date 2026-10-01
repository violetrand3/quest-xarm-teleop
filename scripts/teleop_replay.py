#!/usr/bin/env python3
"""REPLAY of a saved Quest source recording through the teleop adapter and a fake backend.

REPLAY EVIDENCE, NOT LIVE QUEST DATA AND NOT A ROBOT TEST. The recorded source records are fed,
unmodified and in recorded order, through the same sender-side ContinuityTracker and v2 packet
builder, the same receiver state and the same adapter as the live dry run. One virtual clock
(the recording's host receive times) drives both ends, so there is no network delay in a replay.

Input formats: Step 4B recordings ({"host_ns", "raw"} or {"host_ns", "error"}) and checkpoint-5
source logs ({"host_ns", "gen", "raw"}; a change of "gen" is replayed as an acquisition restart).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import quest_pose_transport as pt
import quest_pose_v1 as qp
from teleop_adapter import Recorder, TeleopAdapter, load_config
from teleop_backend import FakeRobotBackend
from teleop_input import ChannelReceiverState, InputChannel

MS = 1_000_000


def replay(path: Path, cfg: Any, send_period_ms: float = 1000 / 30, heartbeat_ms: float = 200, ack_at_s: tuple = (),
           tail_s: float = 1.0) -> dict[str, Any]:  # fmt: skip
    with path.open(encoding="utf-8") as f:
        rows = [json.loads(x) for x in f]
    rows = [r for r in rows if "host_ns" in r]
    t0 = rows[0]["host_ns"]
    clock = [t0]
    channel, rec = InputChannel(), Recorder()
    backend = FakeRobotBackend(clock=lambda: clock[0])
    adapter = TeleopAdapter(cfg, channel, backend, rec)
    state = ChannelReceiverState(channel)
    tr = pt.ContinuityTracker(expect_build=None, stale_ns=500 * MS)
    tr.new_generation(t0, "replay")
    tr.set_source(True, "replay", t0)
    state.open_connection(t0)
    state.process({"v": 2, "type": "hello", "session": "replay", "sender_run": "replay", "protocol": pt.PROTOCOL,
                   "source_format": pt.SOURCE_FORMAT, "source_schema": pt.SOURCE_SCHEMA, "frame": pt.FRAME}, t0)  # fmt: skip
    seq, latest, last_send, last_gen = [0], [None], [t0 - 10**12], [rows[0].get("gen")]
    rejected = [0]
    acks = sorted(int(t0 + a * 1e9) for a in ack_at_s)

    def send(pkt: dict[str, Any], now: int) -> None:
        seq[0] += 1
        pkt.update(v=2, session="replay", seq=seq[0], send_mono_ns=now)
        reply = state.process(json.loads(json.dumps(pkt)), now)
        if reply is None or reply.get("status") != "ok":
            rejected[0] += 1

    def sender_tick(now: int) -> None:
        tr.watchdog(now)
        sample, latest[0] = latest[0], None
        if sample is not None:
            r, raw, recv = sample
            send(pt.pose_packet("replay", tr.acq_gen, tr.session_gen, r, raw, recv, "0.0", tr.continuity_fields(), 0,
                                tr.source_up, tr.stale), now)  # fmt: skip
        elif tr.pending or now - last_send[0] >= heartbeat_ms * MS:
            send({"type": "status", "acq_gen": tr.acq_gen, "source_up": tr.source_up, "source_note": tr.source_note or "-",
                  "stale": tr.stale, "continuity": tr.continuity_fields()}, now)  # fmt: skip
        else:
            return
        last_send[0] = now

    i, n = 0, len(rows)
    end = rows[-1]["host_ns"] + int(tail_s * 1e9)
    next_send = t0
    now = t0
    while now <= end:
        while i < n and rows[i]["host_ns"] <= now:  # every record is processed before decimation
            r = rows[i]
            i += 1
            if r.get("gen") != last_gen[0]:
                last_gen[0] = r.get("gen")
                latest[0] = None
                tr.new_generation(r["host_ns"], "replayed acquisition restart")
            if "raw" not in r:
                tr.on_parse_error(str(r.get("error", "?")), r["host_ns"])
                latest[0] = None
                continue
            try:
                obj = qp.decode_record(r["raw"])
            except qp.RecordError as e:
                tr.on_parse_error(str(e), r["host_ns"])
                latest[0] = None
                continue
            kind = tr.on_record(obj, r["host_ns"])
            if kind == "pose":
                latest[0] = (obj, json.loads(r["raw"]), r["host_ns"])
            elif kind == "event":
                send({"type": "source_event", "acq_gen": tr.acq_gen, "event": json.loads(r["raw"]),
                      "source_recv_mono_ns": r["host_ns"]}, now)  # fmt: skip
            elif kind == "overflow":
                latest[0] = None
        if now >= next_send:
            sender_tick(now)
            next_send += int(send_period_ms * MS)
        while acks and acks[0] <= now:
            acks.pop(0)
            adapter.request_acknowledge()
        clock[0] = now
        adapter.tick(now)
        now += cfg.period_ns
    return {
        "evidence": "REPLAY of a saved recording through a fake backend (not live Quest data, not a robot test)",
        "file": path.name, "source_records": n, "duration_s": round((rows[-1]["host_ns"] - t0) / 1e9, 2),
        "packets_rejected": rejected[0], "engagements": adapter.eng.engagements, "final_state": adapter.eng.state,
        "final_latch": list(adapter.eng.latch_reasons), "synthetic_target_requests": backend.command_count,
        "stop_requests_recorded": backend.stop_count, "gripper_previews": backend.preview_count,
        "transitions": [{"t_s": round((x["t"] - t0) / 1e9, 3), "old": x["old"], "new": x["new"], "reason": x["reason"]} for x in rec.transitions],
        "faults": [{"t_s": round((x["t"] - t0) / 1e9, 3), "reason": x["reason"]} for x in rec.faults],
        "commands": list(backend.commands), "stops": list(backend.stops), "t0": t0,
    }  # fmt: skip


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("recording", type=Path)
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--ack-at", type=float, action="append", default=[], help="operator acknowledgement at this replay time (s)")
    args = ap.parse_args(argv)
    out = replay(args.recording, load_config(args.config), ack_at_s=tuple(args.ack_at))
    out["commands"], out["stops"] = len(out["commands"]), [[round((t - out["t0"]) / 1e9, 3), r] for t, r in out["stops"]]
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
