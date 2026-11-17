#!/usr/bin/env python3
"""LIVE Quest -> two SIMULATED YAM arms. NO HARDWARE: the arms are I2RT's MuJoCo models (yam_sim), no CAN link
is opened. Normally started by `scripts/teleop yam-sim` on the operator's PC; runs on the lab host in I2RT's
environment (/home/dev/yam/i2rt/.venv/bin/python).

Receives the Quest stream (protocol v2, the xArm teleop's sender, loopback only, through the operator's SSH
tunnel), runs the bimanual teleop step (yam_teleop) at 50 Hz on the simulated arms, and shows them in a browser
page (viser, loopback only, through a second tunnel). Each hand: hold the side grip (middle finger) to move its
arm, release to stop; the index trigger closes that arm's gripper.

Signals: SIGINT/SIGTERM end it; SIGUSR1 acknowledges latched hand problems (e.g. a headset reference change).
It also ends by itself after --link-lost-exit-s without a connection (the operator's PC gone).
Console: one line every 0.5 s per state; "ENDING: <why>" when it ends by itself.
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
from typing import Any

import quest_pose_transport as pt
from quest_pose_stream_receiver import log_fields
from quest_stream_receiver import DiagnosticServer
from teleop_input import ChannelReceiverState
from yam_control import BimanualControl
from yam_input import BimanualChannel
from yam_setup import ARMS, Limits
from yam_teleop import YamTeleop

BANNER = "YAM SIMULATION (no hardware)"
WORK = [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # both arms start elbow bent, gripper forward and open
PORT, VIEW_PORT = 28766, 28767  # loopback on the lab host; not the xArm teleop's 28765, not 8765


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--view-port", type=int, default=VIEW_PORT)
    ap.add_argument("--scale", type=float, default=0.667)
    ap.add_argument("--operator-yaw", type=float, default=0.0)
    ap.add_argument("--input-fresh-ms", type=float, default=400.0)
    ap.add_argument("--duration", type=float, default=0.0, help="end after this many seconds (0 = until stopped)")
    ap.add_argument("--link-lost-exit-s", type=float, default=60.0, help="end after this long without a connection from the operator's PC")
    ap.add_argument("--log", type=Path, help="every control step (JSON lines)")
    ap.add_argument("--link-log", type=Path, help="the receiver's packet log")
    a = ap.parse_args(argv)
    if not (0.1 <= a.scale <= 1.5 and -180 <= a.operator_yaw <= 180 and 100 <= a.input_fresh_ms <= 500 and a.duration >= 0):
        print("error: argument outside its allowed range", file=sys.stderr)
        return 2

    from yam_sim import SimView, SimYamPair

    lim = Limits()
    pair = SimYamPair(ARMS, lim)
    for n in pair.names:
        pair.place(n, WORK)
    control = BimanualControl(pair, ARMS, lim)
    teleop = YamTeleop(control, a.scale, a.operator_yaw, a.input_fresh_ms)
    channel = BimanualChannel()
    server = DiagnosticServer(ChannelReceiverState(channel), a.port, a.link_log.open("x") if a.link_log else None,
                              decode_fn=pt.decode, proto_version=pt.PROTOCOL_VERSION, log_fields=log_fields)  # fmt: skip
    view = SimView(pair, a.view_port)
    log = a.log.open("x", encoding="utf-8") if a.log else None
    stop, ack, ended = threading.Event(), threading.Event(), [""]
    last: dict[str, Any] = {"reports": None, "connected": False, "steps": 0}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    signal.signal(signal.SIGUSR1, lambda *_: ack.set())
    start = time.monotonic_ns()

    def scheduler() -> None:
        nxt = time.monotonic_ns()
        try:
            while not stop.is_set():
                now = time.monotonic_ns()
                views = channel.hand_views()
                if ack.is_set():
                    ack.clear()
                    print("acknowledged: " + "; ".join(teleop.acknowledge()), flush=True)
                reports = teleop.tick(views, now)
                last.update(reports=reports, connected=views["right"].connected, steps=last["steps"] + 1)
                if log:
                    log.write(json.dumps({"t": round((now - start) / 1e9, 3), **{h: {"clutch": r.clutch, "arm": r.arm, "grip": r.gripper,
                                          "q": [round(v, 4) for v in control.q[h]], "notes": r.notes} for h, r in reports.items()}}) + "\n")  # fmt: skip
                nxt += int(lim.dt * 1e9)
                delay = (nxt - time.monotonic_ns()) / 1e9
                if delay < 0:
                    nxt = time.monotonic_ns()
                else:
                    stop.wait(delay)
        except Exception:
            traceback.print_exc()
            ended[0] = "the control step failed (see the traceback above)"
            stop.set()

    def viewer() -> None:
        while not stop.wait(1 / 15):
            try:
                view.show({n: list(q) for n, q in control.q.items()},
                          {n: (lim_.target if control.engaged[n] else None) for n, lim_ in control.limiter.items()})
            except Exception:
                traceback.print_exc()
                return

    server.start()
    threads = [threading.Thread(target=scheduler, name="scheduler", daemon=True), threading.Thread(target=viewer, name="viewer", daemon=True)]
    for th in threads:
        th.start()
    print(f"{BANNER}. Listening on 127.0.0.1:{server.port}; view on 127.0.0.1:{a.view_port}. Left hand -> left arm, right hand -> right arm.", flush=True)
    last_link = 0.0
    while not stop.wait(0.5):
        r = last["reports"]
        t = (time.monotonic_ns() - start) / 1e9
        if last["connected"]:
            last_link = t
        elif t - last_link >= a.link_lost_exit_s:
            ended[0] = f"no connection from the operator's PC for {a.link_lost_exit_s:g} s"
            break
        if r is not None:
            hands = " | ".join(f"{h}: {x.clutch} {x.arm} grip={x.gripper}" + (f" [{'; '.join(x.notes)[:110]}]" if x.notes else "") for h, x in r.items())
            print(f"{t:7.1f}s {BANNER} | input={'CONNECTED' if last['connected'] else 'DISCONNECTED'} | {hands}", flush=True)
        if a.duration and t >= a.duration:
            ended[0] = "time limit"
            break
    stop.set()
    for th in threads:
        th.join(timeout=2.0)
    server.stop()
    view.close()
    pair.close()
    if log:
        log.close()
    if ended[0]:
        print(f"ENDING: {ended[0]}", flush=True)
    print(json.dumps({"result": "stopped", "steps": last["steps"], "ended_by": ended[0] or "stopped by the operator"}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
