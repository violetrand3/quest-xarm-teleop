#!/usr/bin/env python3
"""LIVE GRIPPER TEST: the Quest index trigger drives the REAL custom gripper. THE ARM IS FAKE.

Same link, clutch and scheduler as the dry run. The arm backend is the synthetic
FakeRobotBackend (no arm connection exists in this program). The gripper is the lab's gripctl
service on loopback, commanded with POST /api/do/grip only.

  * The jaws follow the index trigger only while the clutch (side grip, middle finger) is engaged.
  * Following starts only after the trigger-derived value comes within the pickup tolerance of
    the jaws' real position, and is rate limited.
  * Trigger 0..1 maps to --open-pct..--closed-pct, which keeps the jaws off both end stops.
  * Clutch release, input loss or any fault: nothing more is sent. The jaws stay where they are.
  * Gripper state lost (USB dropout): following stops; a new pickup is required.
  * At exit, with --restore, one command returns the jaws to where they were at start.

Requires --moves-real-gripper and a --human-confirmation text; without them nothing is sent.
SIGUSR1 = operator acknowledgement of a latched fault (while disengaged).
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
from gripctl_client import GripctlClient
from quest_pose_stream_receiver import DEFAULT_PORT, log_fields, summarize
from quest_stream_receiver import DiagnosticServer
from quest_transport import LOOPBACK
from teleop_adapter import Recorder, TeleopAdapter, load_config
from teleop_backend import FakeRobotBackend, GripperFeedback
from teleop_input import ChannelReceiverState, InputChannel
from teleop_mapping import ConfigError, GripperFollower

BANNER = "GRIPPER LIVE (REAL JAWS MOVE) / ARM FAKE (NO ARM CONNECTION)"
PRINT_INTERVAL_S = 0.5


class FakeArmRealGripperBackend(FakeRobotBackend):
    """Arm: synthetic, as in the dry run. Gripper feedback: the real gripctl state."""

    name = "FakeRobotBackend (arm, synthetic) + gripctl (gripper, REAL)"
    banner = BANNER

    def __init__(self, gripper: GripctlClient):
        super().__init__()
        self._gripper = gripper

    def read_gripper(self) -> GripperFeedback:
        return self._gripper.read()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", type=Path, required=True, help="dry-run configuration (drives the FAKE arm and the clutch)")
    ap.add_argument("--moves-real-gripper", action="store_true", help="required: acknowledges that the real jaws will move")
    ap.add_argument("--human-confirmation", default="", help="who confirmed the on-site checks (recorded)")
    ap.add_argument("--gripctl", default="http://127.0.0.1:8088")
    ap.add_argument("--open-pct", type=float, default=15.0, help="percent closed at trigger 0")
    ap.add_argument("--closed-pct", type=float, default=85.0, help="percent closed at trigger 1")
    ap.add_argument("--pickup-tolerance-pct", type=float, default=8.0)
    ap.add_argument("--max-rate-pct-s", type=float, default=50.0)
    ap.add_argument("--no-restore", action="store_true", help="do not return the jaws to their starting position at exit")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--duration", type=float)
    ap.add_argument("--log", type=Path)
    ap.add_argument("--link-log", type=Path)
    ap.add_argument("--summary", type=Path)
    args = ap.parse_args(argv)
    if not args.moves_real_gripper or len(args.human_confirmation.strip()) < 10:
        print("error: needs --moves-real-gripper and --human-confirmation; nothing was sent", file=sys.stderr)
        return 2
    for p in (args.log, args.link_log, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists; refusing to overwrite", file=sys.stderr)
            return 2
    try:
        cfg = load_config(args.config)
        follower = GripperFollower(args.pickup_tolerance_pct, args.max_rate_pct_s, args.open_pct, args.closed_pct)
    except (OSError, ValueError, ConfigError) as e:
        print(f"error: bad configuration: {e}", file=sys.stderr)
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
    gripper = GripctlClient(args.gripctl, on_event=lambda kind, r: rec.write(kind, time.monotonic_ns(), **r))
    try:
        cal = gripper.load_calibration()
    except Exception as e:
        print(f"error: {e}; nothing was sent", file=sys.stderr)
        return 3
    start_state = gripper.refresh_state()
    if not start_state or not start_state.get("connected"):
        print(f"error: gripper state not available ({(start_state or {}).get('error')}); nothing was sent", file=sys.stderr)
        return 3
    rec.write("gripper_start", time.monotonic_ns(), **{k: start_state.get(k) for k in ("ticks", "pct", "goal", "torque", "load", "age_s")},
              range_pct=[args.open_pct, args.closed_pct], pickup_tolerance_pct=args.pickup_tolerance_pct, calibration=cal,
              max_rate_pct_s=args.max_rate_pct_s, human_confirmation=args.human_confirmation)  # fmt: skip
    channel = InputChannel()
    backend = FakeArmRealGripperBackend(gripper)
    adapter = TeleopAdapter(cfg, channel, backend, rec, gripper_executor=gripper)
    adapter.follower = follower
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

    def scheduler() -> None:
        nxt = time.monotonic_ns()
        while not stop.is_set():
            try:
                adapter.tick(time.monotonic_ns())
            except Exception:
                traceback.print_exc()
                return
            nxt += cfg.period_ns
            delay = (nxt - time.monotonic_ns()) / 1e9
            if delay < 0:
                nxt = time.monotonic_ns()
            else:
                stop.wait(delay)

    def monitor() -> None:
        while not stop.wait(cfg.monitor_period_ns / 1e9):
            adapter.monitor(time.monotonic_ns())

    print(f"{BANNER}. Listening on {LOOPBACK}:{server.port}. Jaws at start: {start_state['ticks']} ticks = {start_state['pct']:.1f} % closed. "
          f"Trigger 0..1 -> {args.open_pct:.0f}..{args.closed_pct:.0f} % closed, pickup +-{args.pickup_tolerance_pct:.0f} %, "
          f"max {args.max_rate_pct_s:.0f} %/s. Confirmation: {args.human_confirmation}", flush=True)  # fmt: skip
    gripper.start()
    server.start()
    threads = [threading.Thread(target=scheduler, name="scheduler", daemon=True),
               threading.Thread(target=monitor, name="monitor", daemon=True)]  # fmt: skip
    for th in threads:
        th.start()
    n = lambda v: "-" if v is None else f"{v:.2f}"  # noqa: E731
    try:
        while not stop.wait(PRINT_INTERVAL_S):
            now = time.monotonic_ns()
            t, g = adapter.last, gripper.snapshot()
            why = f" [{t['blocked'][:60]}]" if t.get("blocked") else ""
            inp = "DISCONNECTED" if not t.get("connected") else "AVAILABLE" if t.get("avail") else "UNAVAILABLE"
            jaws = "DISCONNECTED" if not g.get("connected") else f"{g['ticks']} ticks {g['pct']:.1f}%closed load={g.get('load')}"
            print(f"{(now - start) / 1e9:7.1f}s {BANNER} | {t['state']}{why} | input={inp} gen={t.get('gen')} grip={n(t.get('grip'))} "
                  f"trig={n(t.get('trig'))} | follower={t.get('gripper', '-')} proposal={n(t.get('gripper_pct'))} | jaws: {jaws} | "
                  f"sent={gripper.sent} errors={gripper.send_errors} faults={rec.counts.get('fault', 0)}", flush=True)  # fmt: skip
            if args.duration is not None and now - start >= args.duration * 1e9:
                break
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=2.0)
        server.stop()
        gripper.close()  # the worker is stopped; nothing pending can be sent any more
        end_state = gripper.refresh_state() or {}
        restore: dict[str, Any] = {"requested": not args.no_restore, "done": False}
        if not args.no_restore and end_state.get("connected"):
            # Deliberate end-of-test step (not teleop behaviour): put the jaws back where they were.
            pct = round(start_state["pct"], 2)
            try:
                r = gripper._post_grip(pct)
                time.sleep(1.5)
                after = gripper.refresh_state() or {}
                restore.update(done=bool(r.get("ok")), pct=pct, response=r.get("grip") or r.get("error"), ticks_after=after.get("ticks"))
            except Exception as e:
                restore.update(error=f"{type(e).__name__}: {e}")
            rec.write("gripper_restore", time.monotonic_ns(), **restore)
        end = time.monotonic_ns()
    final = gripper.refresh_state() or {}
    ms = sorted(gripper.send_ms)
    summary = {
        "banner": BANNER, "role": "live_gripper_test", "robot_arm_connection_attempted": False, "gripper_commanded": True,
        "human_confirmation": args.human_confirmation, "elapsed_s": round((end - start) / 1e9, 3), "ticks": adapter.ticks,
        "final_state": adapter.eng.state, "final_latch": list(adapter.eng.latch_reasons), "engagements": adapter.eng.engagements,
        "gripper": {"calibration": cal, "range_pct": [args.open_pct, args.closed_pct], "pickup_tolerance_pct": args.pickup_tolerance_pct,
                    "max_rate_pct_s": args.max_rate_pct_s, "start": {k: start_state.get(k) for k in ("ticks", "pct", "goal", "torque")},
                    "end_before_restore": {k: end_state.get(k) for k in ("ticks", "pct", "goal", "connected")},
                    "restore": restore, "final": {k: final.get(k) for k in ("ticks", "pct", "goal", "torque", "load", "connected")},
                    "commands_sent": gripper.sent, "command_errors": gripper.send_errors, "proposals_cancelled": gripper.cancelled,
                    "state_reads": gripper.state_reads, "state_errors": gripper.state_errors,
                    "command_ms": {"median": ms[len(ms) // 2], "max": ms[-1]} if ms else None,
                    "proposals_computed": backend.preview_count},
        "fake_arm_target_requests": backend.command_count, "fake_arm_stop_requests": backend.stop_count,
        "record_counts": rec.counts,
        "transitions": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.transitions],
        "faults": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.faults],
        "acks": [dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.acks],
        "start_mono_ns": start, "link": summarize(state, start, end),
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
