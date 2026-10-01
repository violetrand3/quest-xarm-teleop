#!/usr/bin/env python3
"""Teleop adapter DRY RUN. NO ROBOT OR GRIPPER CONNECTED.

Listens on 127.0.0.1 for the protocol-v2 Quest pose link (the same loopback port the diagnostic
receiver uses), runs the teleop adapter against a SYNTHETIC FakeRobotBackend and records what
it would have requested. There is no backend option and no flag that selects hardware: the only
backend this program can construct is the fake one, and the only configuration it accepts is
one marked not approved for live motion.

  scheduler thread  fixed period on this host's monotonic clock, independent of network reads
  monitor thread    detects a stuck scheduler and requests a stop without waiting for it
  SIGUSR1           operator acknowledgement of a latched fault (only applies while disengaged)
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any, TextIO

import quest_pose_transport as pt
from quest_pose_stream_receiver import DEFAULT_PORT, log_fields, summarize
from quest_stream_receiver import DiagnosticServer
from quest_transport import LOOPBACK
from teleop_adapter import Recorder, TeleopAdapter, load_config
from teleop_backend import DRY_RUN_BANNER, FakeRobotBackend
from teleop_input import ChannelReceiverState, InputChannel
from teleop_mapping import ConfigError

PRINT_INTERVAL_S = 0.5


def format_status(t: dict[str, Any], backend: FakeRobotBackend, faults: int) -> str:
    f = lambda p: "-" if p is None else "(%+7.1f,%+7.1f,%+7.1f)" % tuple(p)  # noqa: E731
    n = lambda v: "-" if v is None else f"{v:.2f}"  # noqa: E731
    src = "-" if not t.get("src") else f"acq{t['src'][0]}/{str(t['src'][1])[:8]}"
    inp = "DISCONNECTED" if not t.get("connected") else "AVAILABLE" if t.get("avail") else "UNAVAILABLE"
    why = f" [{t['blocked'][:70]}]" if t.get("blocked") else ""
    gp = t.get("gripper", "-") + ("" if t.get("gripper_pct") is None else f" {t['gripper_pct']:.0f}%closed")
    return (f"{DRY_RUN_BANNER} | {t['state']}{why} | input={inp} gen={t.get('gen')} src={src} grip={n(t.get('grip'))} "
            f"trig={n(t.get('trig'))} | synthetic mm: req={f(t.get('req_mm'))} lim={f(t.get('lim_mm'))} "
            f"fb={f(t.get('fb_mm'))} ori={t.get('ori') or '-'} | gripper preview: {gp} | "
            f"cmds={backend.command_count} stops={backend.stop_count} faults={faults}")  # fmt: skip


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", type=Path, required=True, help="dry-run configuration (must be marked not approved for live)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="loopback port (host is always 127.0.0.1)")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--log", type=Path, help="adapter JSONL recording (must not exist)")
    ap.add_argument("--link-log", type=Path, help="per-packet link log (must not exist)")
    ap.add_argument("--summary", type=Path, help="summary JSON (must not exist)")
    args = ap.parse_args(argv)
    for p in (args.log, args.link_log, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists; refusing to overwrite", file=sys.stderr)
            return 2
    try:
        cfg = load_config(args.config)
    except (OSError, ValueError, ConfigError) as e:
        print(f"error: bad configuration: {e}", file=sys.stderr)
        return 2
    if cfg.approved_for_live or cfg.mapping_verified:
        print("error: this program is a dry run; it refuses a configuration marked approved_for_live or with a "
              "verified mapping (it has no hardware backend to use it with)", file=sys.stderr)  # fmt: skip
        return 2

    streams: list[TextIO] = []

    def opened(p: Path | None) -> TextIO | None:
        if p is None:
            return None
        p.parent.mkdir(parents=True, exist_ok=True)
        f = p.open("x", encoding="utf-8")
        streams.append(f)
        return f

    rec = Recorder(opened(args.log))
    channel = InputChannel()
    backend = FakeRobotBackend()  # the only backend this program can construct
    adapter = TeleopAdapter(cfg, channel, backend, rec)
    state = ChannelReceiverState(channel)
    try:
        server = DiagnosticServer(state, args.port, opened(args.link_log), decode_fn=pt.decode,
                                  proto_version=pt.PROTOCOL_VERSION, log_fields=log_fields)  # fmt: skip
    except OSError as e:
        print(f"error: cannot bind {LOOPBACK}:{args.port}: {e}", file=sys.stderr)
        return 2

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGUSR1, lambda *_: adapter.request_acknowledge())
    start = time.monotonic_ns()
    overruns = [0]

    def scheduler() -> None:
        nxt = time.monotonic_ns()
        while not stop.is_set():
            try:
                adapter.tick(time.monotonic_ns())
            except Exception:  # a dead scheduler is left for the monitor to detect and stop
                traceback.print_exc()
                return
            nxt += cfg.period_ns
            delay = (nxt - time.monotonic_ns()) / 1e9
            if delay < 0:  # late: do not try to catch up with a burst of ticks
                overruns[0] += 1
                nxt = time.monotonic_ns()
            else:
                stop.wait(delay)

    def monitor() -> None:
        while not stop.wait(cfg.monitor_period_ns / 1e9):
            adapter.monitor(time.monotonic_ns())

    print(f"{DRY_RUN_BANNER}. Teleop adapter dry run on {LOOPBACK}:{server.port} (loopback only). Backend: {backend.name}, "
          f"all poses and feedback SYNTHETIC. Config: {cfg.label} Mapping: {cfg.mapping_label} "
          f"Scale {cfg.mapping.scale}. SIGUSR1 = operator acknowledgement.", flush=True)  # fmt: skip
    server.start()
    threads = [threading.Thread(target=scheduler, name="scheduler", daemon=True),
               threading.Thread(target=monitor, name="monitor", daemon=True)]  # fmt: skip
    for th in threads:
        th.start()
    try:
        while not stop.wait(PRINT_INTERVAL_S):
            now = time.monotonic_ns()
            print(f"{(now - start) / 1e9:7.1f}s {format_status(adapter.last, backend, rec.counts.get('fault', 0))}", flush=True)
            if args.duration is not None and now - start >= args.duration * 1e9:
                break
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=2.0)
        server.stop()
        end = time.monotonic_ns()
    summary = {
        "banner": DRY_RUN_BANNER, "role": "teleop_dryrun", "robot_connection_attempted": False,
        "gripper_connection_attempted": False, "backend": backend.describe(), "config_label": cfg.label,
        "mapping_label": cfg.mapping_label, "elapsed_s": round((end - start) / 1e9, 3), "ticks": adapter.ticks,
        "scheduler_late_ticks": overruns[0], "final_state": adapter.eng.state, "final_latch": list(adapter.eng.latch_reasons),
        "engagements": adapter.eng.engagements, "synthetic_target_requests": backend.command_count,
        "stop_requests_recorded": backend.stop_count, "gripper_previews": backend.preview_count,
        "gripper_commands_executed": 0, "record_counts": rec.counts,
        "transitions": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.transitions],
        "faults": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.faults],
        "stops": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.stops],
        "acks": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.acks],
        "start_mono_ns": start, "link": summarize(state, start, end),
        "notes": ["Stop requests are recorded by a fake backend; this does not show physical stopping.",
                  "Input available is a statement about the Quest stream, not robot readiness."],
    }  # fmt: skip
    for f in streams:
        f.close()
    text = json.dumps(summary, indent=1)
    print(text)
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
