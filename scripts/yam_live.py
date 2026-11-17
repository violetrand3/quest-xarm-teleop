#!/usr/bin/env python3
"""Bimanual YAM teleop on the lab host: the Quest controllers drive two YAM arms, SIMULATED (--arms sim, the
default: no hardware) or REAL (--arms real: THE ARMS MOVE; needs --moves-real-arms and the operator's typed
confirmation, and the bring-up's hardware file, see yam_real.py). Started by scripts/teleop yam-sim / yam-try.

Runs in I2RT's environment (/home/dev/yam/i2rt/.venv/bin/python). Receives the Quest stream (protocol v2, the
xArm teleop's sender, loopback only, through the operator's SSH tunnel), runs the bimanual teleop step (yam_teleop)
at 50 Hz, and shows the arms in a browser page (viser, loopback only, through a second tunnel). Each hand: hold the
side grip (middle finger) to move its arm, release to stop; the index trigger closes that arm's gripper.

Ending: SIGINT/SIGTERM (Ctrl-C or scripts/teleop stop on the operator's PC), the time limit, no connection from the
operator's PC for --link-lost-exit-s, or a failure of the control step. Every one of them first LOWERS BOTH ARMS TO
THEIR REST POSES at --park-speed, under the same clearance rules as teleop, and only then lets the arms go (real
arms: torque off, they go limp, so they must be at rest first). If lowering cannot finish within --park-timeout-s
the arms stay held where they are and the program says so; a second SIGINT then ends it (real arms go limp).
SIGUSR1 acknowledges latched hand problems (e.g. a headset reference change).
Console: one status line every 0.5 s; "ENDING: <why>" when it ends by itself; a JSON result line last.
"""

from __future__ import annotations

import argparse
import dataclasses
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

WORK = [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # simulation start: elbow bent, gripper forward and open
PORT, VIEW_PORT = 28766, 28767  # loopback on the lab host; not the xArm teleop's 28765, not 8765
REAL_FACTOR = 0.5  # real arms: every speed limit halved until the arms have been seen to follow smoothly


def first_run_limits(lim: Limits) -> Limits:
    return dataclasses.replace(lim, tool_speed=lim.tool_speed * REAL_FACTOR, tool_rot_speed=lim.tool_rot_speed * REAL_FACTOR,
                               joint_speed=lim.joint_speed * REAL_FACTOR, grip_speed=lim.grip_speed * REAL_FACTOR)  # fmt: skip


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arms", choices=("sim", "real"), default="sim")
    ap.add_argument("--moves-real-arms", action="store_true", help="with --arms real: the operator knows the arms will move")
    ap.add_argument("--human-confirmation", default="", help="with --arms real: what the operator typed at the go/no-go questions")
    ap.add_argument("--hardware-file", type=Path, help="with --arms real: the bring-up's measurements (default: yam_real.HARDWARE_FILE)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--view-port", type=int, default=VIEW_PORT)
    ap.add_argument("--scale", type=float, default=0.667)
    ap.add_argument("--operator-yaw", type=float, default=0.0)
    ap.add_argument("--input-fresh-ms", type=float, default=400.0)
    ap.add_argument("--duration", type=float, default=0.0, help="end after this many seconds (0 = until stopped)")
    ap.add_argument("--link-lost-exit-s", type=float, default=0.0, help="end after this long without the operator's PC (default: 60 sim, 10 real)")
    ap.add_argument("--park-speed", type=float, default=0.3, help="joint speed while lowering to rest, rad/s")
    ap.add_argument("--park-timeout-s", type=float, default=40.0)
    ap.add_argument("--log", type=Path, help="every control step (JSON lines)")
    ap.add_argument("--link-log", type=Path, help="the receiver's packet log")
    a = ap.parse_args(argv)
    real = a.arms == "real"
    if not a.link_lost_exit_s:
        a.link_lost_exit_s = 10.0 if real else 60.0
    if not (0.1 <= a.scale <= 1.5 and -180 <= a.operator_yaw <= 180 and 100 <= a.input_fresh_ms <= 500 and a.duration >= 0
            and 0.05 <= a.park_speed <= 0.6 and 5 <= a.park_timeout_s <= 120 and 2 <= a.link_lost_exit_s <= 600):  # fmt: skip
        print("error: argument outside its allowed range", file=sys.stderr)
        return 2
    if real and not (a.moves_real_arms and a.human_confirmation.strip()):
        print("error: --arms real needs --moves-real-arms and --human-confirmation (the operator's answers); nothing was opened", file=sys.stderr)
        return 2
    banner = "YAM REAL ARMS (THE ARMS MOVE)" if real else "YAM SIMULATION (no hardware)"

    lim = first_run_limits(Limits()) if real else Limits()
    if real:
        from yam_real import GRIP_REST_MAX, HARDWARE_FILE, GripperStartError, HardwareFileError, RealYamPair

        try:
            pair = RealYamPair(ARMS, lim, a.hardware_file or HARDWARE_FILE)
        except HardwareFileError as e:
            print(f"error: {e}; nothing was opened", file=sys.stderr)
            return 2
        except GripperStartError as e:
            print(f"error: {e}. Both arms were switched off again.", file=sys.stderr)
            return 2
        grip_max = GRIP_REST_MAX
        print(f"{banner}: both arms enabled and holding where they are. Confirmation: {a.human_confirmation.strip()[:300]}", flush=True)
    else:
        from yam_sim import SimYamPair

        pair = SimYamPair(ARMS, lim)
        for n in pair.names:
            pair.place(n, WORK)
        grip_max = None
    rest = {n: pair.rest(n) for n in pair.names}
    control = BimanualControl(pair, ARMS, lim)
    teleop = YamTeleop(control, a.scale, a.operator_yaw, a.input_fresh_ms)
    channel = BimanualChannel()
    server = DiagnosticServer(ChannelReceiverState(channel), a.port, a.link_log.open("x") if a.link_log else None,
                              decode_fn=pt.decode, proto_version=pt.PROTOCOL_VERSION, log_fields=log_fields)  # fmt: skip
    view = None
    try:
        from yam_sim import SimView

        view = SimView(pair, a.view_port)
    except Exception as e:  # the view is a convenience; the arms do not depend on it
        print(f"no browser view: {e}", flush=True)
    log = a.log.open("x", encoding="utf-8") if a.log else None
    closing, ack = threading.Event(), threading.Event()
    stops = [0]
    phase = ["teleop"]  # teleop -> parking -> parked, or -> holding (lowering could not finish)
    last: dict[str, Any] = {"reports": None, "connected": False, "steps": 0, "park": {}}
    ended = [""]

    def on_stop(*_: Any) -> None:
        stops[0] += 1

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_stop)
    signal.signal(signal.SIGUSR1, lambda *_: ack.set())
    start = time.monotonic_ns()

    def scheduler() -> None:
        nxt = time.monotonic_ns()
        while not closing.is_set():
            now = time.monotonic_ns()
            try:
                if phase[0] == "teleop":
                    views = channel.hand_views()
                    if ack.is_set():
                        ack.clear()
                        print("acknowledged: " + "; ".join(teleop.acknowledge()), flush=True)
                    reports = teleop.tick(views, now)
                    last.update(reports=reports, connected=views["right"].connected, steps=last["steps"] + 1)
                    if log:
                        log.write(json.dumps({"t": round((now - start) / 1e9, 3), **{h: {"clutch": r.clutch, "arm": r.arm, "grip": r.gripper,
                                              "q": [round(v, 4) for v in control.q[h]], "notes": r.notes} for h, r in reports.items()}}) + "\n")  # fmt: skip
                elif phase[0] == "parking":
                    done, said = control.park_tick(rest, a.park_speed, grip_max)
                    last["park"] = said
                    if log:
                        log.write(json.dumps({"t": round((now - start) / 1e9, 3), "parking": said, "q": control.q}) + "\n")
                    if done:
                        phase[0] = "parked"
                else:  # parked or holding: the arms keep their last command
                    for n in pair.names:
                        pair.command(n, control.q[n])
            except Exception:
                traceback.print_exc()
                if phase[0] == "teleop":
                    ended[0] = "the control step failed (see the traceback above)"
                    phase[0] = "parking"  # lowering uses only the joint step and the clearance check
                else:
                    phase[0] = "holding"
            nxt += int(lim.dt * 1e9)
            delay = (nxt - time.monotonic_ns()) / 1e9
            if delay < 0:
                nxt = time.monotonic_ns()
            else:
                closing.wait(delay)

    def viewer() -> None:
        while view is not None and not closing.wait(1 / 15):
            try:
                view.show({n: list(q) for n, q in control.q.items()},
                          {n: (lm.target if control.engaged[n] else None) for n, lm in control.limiter.items()})
            except Exception:
                traceback.print_exc()
                return

    server.start()
    threads = [threading.Thread(target=scheduler, name="scheduler", daemon=True), threading.Thread(target=viewer, name="viewer", daemon=True)]
    for th in threads:
        th.start()
    print(f"{banner}. Listening on 127.0.0.1:{server.port}; view on 127.0.0.1:{a.view_port}. Left hand -> left arm, right hand -> right arm.", flush=True)
    last_link, park_t0, warned = 0.0, None, 0
    while True:
        closing.wait(0.5)
        t = (time.monotonic_ns() - start) / 1e9
        if last["connected"]:
            last_link = t
        if phase[0] == "teleop":
            why = ("stopped by the operator" if stops[0] else ended[0] or
                   ("time limit" if a.duration and t >= a.duration else
                    f"no connection from the operator's PC for {a.link_lost_exit_s:g} s" if t - last_link >= a.link_lost_exit_s else ""))  # fmt: skip
            if why:
                ended[0] = why
                phase[0] = "parking"
                stops[0] = min(stops[0], 1)
            r = last["reports"]
            if r is not None and phase[0] == "teleop":
                hands = " | ".join(f"{h}: {x.clutch} {x.arm} grip={x.gripper}" + (f" [{'; '.join(x.notes)[:110]}]" if x.notes else "") for h, x in r.items())
                print(f"{t:7.1f}s {banner} | input={'CONNECTED' if last['connected'] else 'DISCONNECTED'} | {hands}", flush=True)
        if phase[0] == "parking":
            if park_t0 is None:
                park_t0 = t
                if ended[0] != "stopped by the operator":
                    print(f"ENDING: {ended[0]}", flush=True)
                print("Lowering both arms to rest ...", flush=True)
            print(f"{t:7.1f}s {banner} | LOWERING | " + " | ".join(f"{n}: {s}" for n, s in last["park"].items()), flush=True)
            if stops[0] > 1 and stops[0] != warned:
                warned = stops[0]
                print("still lowering to rest" + ("; for an emergency use the e-stop" if real else ""), flush=True)
            if t - park_t0 > a.park_timeout_s:
                phase[0] = "holding"
        if phase[0] == "parked":
            print("Both arms are at rest.", flush=True)
            break
        if phase[0] == "holding":
            if warned != -1:
                warned = -1
                stops[0] = 1
                print("COULD NOT LOWER BOTH ARMS TO REST: " + " | ".join(f"{n}: {s}" for n, s in last["park"].items())
                      + (". They are held where they are. Stop again (Ctrl-C) to let them go: THEY WILL DROP." if real else ""), flush=True)  # fmt: skip
            if not real or stops[0] > 1:
                break
    closing.set()
    for th in threads:
        th.join(timeout=2.0)
    server.stop()
    if view is not None:
        view.close()
    pair.close()
    if log:
        log.close()
    print(json.dumps({"result": "stopped", "arms": a.arms, "steps": last["steps"], "ended_by": ended[0], "at_rest": phase[0] == "parked"}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
