#!/usr/bin/env python3
"""LIVE TELEOP: the Quest right controller moves the REAL xArm. THE ARM MOVES.

Hold the side grip (middle finger) and the tool follows your hand, scaled down; release and it
stops. Position only: the tool keeps the orientation it has when you engage.

  * The arm stays inside a small box around where it is at start (--box, millimetres). A hand
    movement that would leave the box is held at its edge. The box must lie inside the controller's
    fence with a margin.
  * The command link refuses, before anything reaches the wire, any target outside that box and any
    speed above --ctrl-speed.
  * Clutch released or input lost: no more targets are sent. A fault (controller error, jump in the
    input, tracking error, stuck scheduler): set_state(4), and nothing moves again until the
    operator sends SIGUSR1 (acknowledge and re-arm) and then releases and presses the clutch.
  * Before arming: the controller must be error-free, idle, real (not simulation), fence on.
    Reduced mode must be on unless --allow-reduced-mode-off records the operator's decision.
  * At exit the arm is left as the lab's code leaves it: mode 0, state 4.

Directions (R below): as if you stood at the robot's base looking outward. Hand forward -> +x
(away from the base), hand right -> -y, hand up -> +z. +z was verified on the rig; the x/y signs
come from the lab's notes and have NOT been verified.

set_state(4) is a software stop, not the physical emergency stop.
Needs --moves-real-arm and --human-confirmation; without them it only prints the plan.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any, TextIO

import quest_pose_transport as pt
import xarm_command_link as cl
import xarm_readonly_preflight as ro
from quest_pose_stream_receiver import DEFAULT_PORT, log_fields, summarize
from quest_stream_receiver import DiagnosticServer
from quest_transport import LOOPBACK
from teleop_adapter import ACTIVE, Recorder, TeleopAdapter, parse_config
from teleop_backend import BackendError
from teleop_input import ChannelReceiverState, InputChannel
from teleop_mapping import GripperFollower
from xarm_live_backend import XArmLiveBackend

BANNER = "ARM LIVE (REAL ARM MOVES)"
FENCE_MARGIN_MM = 10.0
LINK_MARGIN_MM = 2.0
PRINT_INTERVAL_S = 0.5
R_BASE_FROM_LOCAL = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
MAPPING_LABEL = ("LIVE: operator as if at the robot base looking outward. Hand forward -> +x, hand right -> -y, hand up -> +z. "
                 "+z verified 2026-10-12; x/y signs from the lab's notes, unverified.")  # fmt: skip


class Abort(RuntimeError):
    pass


def workspace(pose: tuple, boundary: dict, box: list) -> tuple:
    """Box around the start pose (mm), which must lie inside the fence with a margin."""
    lo = [pose[k] - box[2 * k] for k in range(3)]
    hi = [pose[k] + box[2 * k + 1] for k in range(3)]
    for k, ax in enumerate("xyz"):
        f_lo, f_hi = boundary[f"{ax}_mm"]
        if lo[k] < f_lo + FENCE_MARGIN_MM or hi[k] > f_hi - FENCE_MARGIN_MM:
            raise Abort(f"the box {ax} {lo[k]:.0f}..{hi[k]:.0f} mm comes within {FENCE_MARGIN_MM:g} mm of the fence {f_lo}..{f_hi}; nothing was commanded")
    return [round(v, 2) for v in lo], [round(v, 2) for v in hi]


def build_config(lo_mm: list, hi_mm: list, a: argparse.Namespace) -> dict:
    rot = {} if a.rotation == "off" else {"rotation": {"mode": a.rotation, "scale": a.rot_scale, "max_angle_deg": a.rot_max_deg,
                                                       "w_max_deg_s": a.rot_speed, "max_error_deg": 10.0, "max_step_deg": 90.0}}  # fmt: skip
    return {
        **rot,
        "label": "LIVE teleop, small box around the start pose; values chosen for a first supervised session",
        "approved_for_live": True,
        "engagement": {"grip_released_max": 0.2, "grip_pressed_min": 0.8, "grip_hold_min": 0.5,
                       "input_fresh_ms": a.input_fresh_ms, "feedback_fresh_ms": 100},
        "mapping": {"label": MAPPING_LABEL, "verified": False, "scale": a.scale, "R_backend_from_local": R_BASE_FROM_LOCAL},
        "limits": {"workspace_min_m": [v / 1000.0 for v in lo_mm], "workspace_max_m": [v / 1000.0 for v in hi_mm],
                   "workspace_edge": "clamp", "max_anchor_displacement_m": 1.0, "max_controller_step_m": 0.30,  # a network gap is not a jump; the speed check below still catches teleports
                   "max_controller_speed_m_s": 2.5, "v_max_m_s": a.v_max / 1000.0, "a_max_m_s2": a.a_max / 1000.0,
                   "max_target_feedback_error_m": a.track_err / 1000.0},
        "scheduler": {"period_ms": 20, "max_tick_dt_ms": 100, "monitor_period_ms": 25, "stall_ms": 150},
        "gripper": {"pickup_tolerance_pct": 8, "max_rate_pct_s": 50},
    }  # fmt: skip


def main(argv: list[str] | None = None, sdk: tuple | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--address-from-file", type=Path)
    src.add_argument("--address")
    ap.add_argument("--moves-real-arm", action="store_true")
    ap.add_argument("--human-confirmation", default="")
    ap.add_argument("--allow-reduced-mode-off", action="store_true", help="the operator's explicit decision; recorded")
    ap.add_argument("--box", default="50,50,50,50,10,60", help="mm around the start pose: x-,x+,y-,y+,z-,z+ (each 0..300)")
    ap.add_argument("--scale", type=float, default=0.25, help="tool displacement per hand displacement (0.05..1.0)")
    ap.add_argument("--v-max", type=float, default=30.0, help="target speed limit, mm/s (5..150)")
    ap.add_argument("--a-max", type=float, default=150.0, help="target acceleration limit, mm/s^2")
    ap.add_argument("--ctrl-speed", type=float, default=50.0, help="speed given to the controller for each line target, mm/s (<= 200)")
    ap.add_argument("--ctrl-acc", type=float, default=500.0, help="acceleration given to the controller, mm/s^2 (<= 2000)")
    ap.add_argument("--track-err", type=float, default=20.0, help="fault if the target is further than this from the arm, mm")
    ap.add_argument("--input-fresh-ms", type=float, default=300.0)
    ap.add_argument("--rotation", choices=["off", "yaw", "full"], default="off",
                    help="off: tool orientation frozen at each engagement. yaw: turning the hand about 'up' turns the tool about base z. full: all axes")
    ap.add_argument("--rot-max-deg", type=float, default=30.0, help="limit from the tool orientation at session start (5..90)")
    ap.add_argument("--rot-speed", type=float, default=40.0, help="rotation speed limit, deg/s (5..120)")
    ap.add_argument("--rot-scale", type=float, default=1.0, help="tool rotation per hand rotation (0.2..1.5)")
    ap.add_argument("--with-gripper", action="store_true", help="index trigger also drives the real gripper (0..90 %% closed)")
    ap.add_argument("--gripctl", default="http://127.0.0.1:8088")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--duration", type=float, default=120.0, help="seconds, then park and exit (max 600)")
    ap.add_argument("--log", type=Path)
    ap.add_argument("--link-log", type=Path)
    ap.add_argument("--summary", type=Path)
    a = ap.parse_args(argv)
    try:
        box = [float(x) for x in a.box.split(",")]
        ok = (len(box) == 6 and all(0.0 <= v <= 300.0 for v in box) and 0.05 <= a.scale <= 1.0 and 5.0 <= a.v_max <= 150.0
              and 20.0 <= a.a_max <= 1000.0 and a.v_max <= a.ctrl_speed <= 200.0 and 50.0 <= a.ctrl_acc <= 2000.0
              and 5.0 <= a.track_err <= 30.0 and 100.0 <= a.input_fresh_ms <= 500.0 and 5.0 <= a.duration <= 600.0
              and 5.0 <= a.rot_max_deg <= 90.0 and 5.0 <= a.rot_speed <= 120.0 and 0.2 <= a.rot_scale <= 1.5)  # fmt: skip
    except ValueError:
        ok = False
    if not ok:
        print("error: argument outside its allowed range; nothing was done", file=sys.stderr)
        return 2
    print(f"{BANNER}. Box {a.box} mm around the start pose, scale {a.scale}, target speed <= {a.v_max:g} mm/s, "
          f"controller cap {a.ctrl_speed:g} mm/s, gripper {'ON' if a.with_gripper else 'off'}, wrist rotation {a.rotation}"
          + (f" (max {a.rot_max_deg:g} deg, {a.rot_speed:g} deg/s)." if a.rotation != "off" else "."), flush=True)  # fmt: skip
    if not a.moves_real_arm:
        print("plan only: --moves-real-arm not given; no connection was made", flush=True)
        return 0
    if len(a.human_confirmation.strip()) < 10:
        print("error: --moves-real-arm needs --human-confirmation", file=sys.stderr)
        return 2
    for p in (a.log, a.link_log, a.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists", file=sys.stderr)
            return 2
    address = a.address or ro.address_from_file(a.address_from_file)
    api_class, cmd_class, sdk_ver = sdk or cl.load_sdk()
    out: dict = {"tool": "live_teleop", "banner": BANNER, "sdk": sdk_ver, "result": "started", "host_wall_start": time.time(),
                 "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(a).items() if k != "address"}}  # fmt: skip
    streams: list[TextIO] = []

    def opened(p: Path | None) -> TextIO | None:
        if p is None:
            return None
        p.parent.mkdir(parents=True, exist_ok=True)
        f = p.open("x", encoding="utf-8")
        streams.append(f)
        return f

    link: cl.XArmCommandLink | None = None
    backend: XArmLiveBackend | None = None
    server: DiagnosticServer | None = None
    gripper: Any = None
    adapter: TeleopAdapter | None = None
    rec = Recorder(None)
    state: ChannelReceiverState | None = None
    stop = threading.Event()
    threads: list = []
    start = time.monotonic_ns()
    try:
        # 1. read-only look with a link that cannot move anything
        probe = cl.XArmCommandLink(api_class, cmd_class, address, (0, 0, 0), (1e-3, 1e-3, 1e-3), 1e-3, 1e-3)
        probe.connect()
        try:
            s = probe.read()
            rs = probe.reduced_states()
            boundary = ro.decode_boundary(rs[1])
            sim, t_end = None, time.monotonic() + 2.0
            while sim is None and time.monotonic() < t_end:
                sim = probe.is_simulation()
                time.sleep(0.05)
            out["before"] = {"pose": list(s.pose), "state": s.state, "error": s.error, "warn": s.warn, "cmdnum": s.cmdnum,
                             "is_simulation_robot": sim, "reported_mode": probe.reported_mode(), "version": probe.version,
                             "reduced_mode_on": rs[0], "fence_on": rs[5] if len(rs) > 5 else None, "boundary": boundary}  # fmt: skip
            if sim is not False:
                raise Abort("the controller is in simulation mode or sent no status report; nothing was commanded")
            if s.error or s.warn:
                raise Abort(f"controller error {s.error} / warning {s.warn}; nothing was commanded")
            if s.cmdnum != 0 or s.state not in (2, 4):
                raise Abort(f"arm is not idle (state {s.state}, {s.cmdnum} queued commands); nothing was commanded")
            if not (len(rs) > 5 and rs[5] == 1):
                raise Abort("the fence (safety boundary) is not on; nothing was commanded")
            if rs[0] != 1:
                if not a.allow_reduced_mode_off:
                    raise Abort("reduced mode is not on; nothing was commanded")
                out["before"]["reduced_mode_waived_by_operator"] = True
            s2 = (time.sleep(0.5), probe.read())[1]
            if max(abs(u - v) for u, v in zip(s.pose[:3], s2.pose[:3])) > 0.2:
                raise Abort("the arm is moving; nothing was commanded")
            if a.rotation != "off":
                err = math.degrees(probe.rpy_convention_error_rad())  # read-only: our roll/pitch/yaw reading vs the controller's axis-angle
                out["before"]["rpy_convention_error_deg"] = round(err, 4)
                if err > 0.2:
                    raise Abort(f"roll/pitch/yaw convention check failed ({err:.2f} deg); wrist rotation cannot be trusted; nothing was commanded")
            lo, hi = workspace(s2.pose, boundary, box)
            out["workspace_mm"] = {"min": lo, "max": hi, "start": [round(v, 2) for v in s2.pose[:3]]}
        finally:
            probe.close()
        cfg = parse_config(build_config(lo, hi, a))
        out["config"] = cfg.raw
        print(f"workspace (base frame, mm): x {lo[0]}..{hi[0]}, y {lo[1]}..{hi[1]}, z {lo[2]}..{hi[2]}; start {out['workspace_mm']['start']}", flush=True)

        # 2. the commanding link, boxed to that workspace
        # the link also refuses any target rotated further from the start orientation than the session allows
        link = cl.XArmCommandLink(api_class, cmd_class, address, tuple(v - LINK_MARGIN_MM for v in lo), tuple(v + LINK_MARGIN_MM for v in hi),
                                  a.ctrl_speed, a.ctrl_acc, ref_rpy=tuple(s2.pose[3:6]),
                                  max_rot_rad=math.radians(a.rot_max_deg + 3.0 if a.rotation != "off" else 1.0))  # fmt: skip
        link.connect()
        backend = XArmLiveBackend(link, a.ctrl_speed, a.ctrl_acc)
        rec = Recorder(opened(a.log))
        channel = InputChannel()
        if a.with_gripper:
            from gripctl_client import GripctlClient  # the gripper is a separate device and optional

            gripper = GripctlClient(a.gripctl, on_event=lambda kind, r: rec.write(kind, time.monotonic_ns(), **r))
            out["gripper_calibration"] = gripper.load_calibration()
            g0 = gripper.refresh_state()
            if not g0 or not g0.get("connected"):
                raise Abort("gripper not connected; nothing was commanded")
            out["gripper_start"] = {k: g0.get(k) for k in ("ticks", "pct")}
            backend.gripper = gripper
        adapter = TeleopAdapter(cfg, channel, backend, rec, gripper_executor=gripper)
        if gripper is not None:
            adapter.follower = GripperFollower(8.0, 50.0, 0.0, 90.0)
        state = ChannelReceiverState(channel)
        server = DiagnosticServer(state, a.port, opened(a.link_log), decode_fn=pt.decode, proto_version=pt.PROTOCOL_VERSION,
                                  log_fields=log_fields)  # fmt: skip
        rearm = [False]

        def on_usr1(*_: Any) -> None:
            adapter.request_acknowledge()  # type: ignore[union-attr]
            rearm[0] = True

        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda *_: stop.set())
        signal.signal(signal.SIGUSR1, on_usr1)

        backend.arm_motion()  # enable, mode 7, state 0: the operator's explicit step is having launched this program
        time.sleep(1.0)
        out["armed"] = {"mode": link.reported_mode(), "t_s": round((time.monotonic_ns() - start) / 1e9, 2)}

        def scheduler() -> None:
            nxt = time.monotonic_ns()
            while not stop.is_set():
                try:
                    adapter.tick(time.monotonic_ns())  # type: ignore[union-attr]
                except Exception:  # a dead scheduler is detected by the monitor, which halts the arm
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
                adapter.monitor(time.monotonic_ns())  # type: ignore[union-attr]

        if gripper is not None:
            gripper.start()
        server.start()
        threads = [threading.Thread(target=scheduler, name="scheduler", daemon=True), threading.Thread(target=monitor, name="monitor", daemon=True)]
        for th in threads:
            th.start()
        print(f"{BANNER}. Armed in mode {out['armed']['mode']}. Listening on {LOOPBACK}:{server.port}. SIGUSR1 = acknowledge and re-arm.", flush=True)
        f3 = lambda p: "-" if p is None else "(%+7.1f,%+7.1f,%+7.1f)" % tuple(p)  # noqa: E731
        n2 = lambda v: "-" if v is None else f"{v:.2f}"  # noqa: E731
        while not stop.wait(PRINT_INTERVAL_S):
            now = time.monotonic_ns()
            if rearm[0]:
                rearm[0] = False
                time.sleep(0.1)  # let the scheduler apply the acknowledgement first
                if adapter.eng.state != ACTIVE and link.armed_mode is None:
                    try:
                        backend.arm_motion()
                        print("re-armed by the operator", flush=True)
                    except BackendError as e:
                        print(f"re-arm refused: {e}", flush=True)
            t = adapter.last
            why = f" [{t['blocked'][:60]}]" if t.get("blocked") else ""
            inp = "DISCONNECTED" if not t.get("connected") else "AVAILABLE" if t.get("avail") else "UNAVAILABLE"
            print(f"{(now - start) / 1e9:7.1f}s {BANNER} | {t['state']}{why} | input={inp} gen={t.get('gen')} grip={n2(t.get('grip'))} "
                  f"trig={n2(t.get('trig'))} | mm: req={f3(t.get('req_mm'))} lim={f3(t.get('lim_mm'))} fb={f3(t.get('fb_mm'))} rot={t.get('rot_deg')} | "
                  f"armed={'yes' if link.armed_mode is not None else 'NO'} cmds={backend.command_count} halts={backend.halt_count} "
                  f"faults={rec.counts.get('fault', 0)}", flush=True)  # fmt: skip
            if now - start >= a.duration * 1e9:
                break
        out["result"] = "ok"
    except (Abort, cl.LinkError, BackendError, ValueError, OSError) as e:
        out["result"], out["abort_reason"] = "aborted", str(e)
        print(f"ABORTED: {e}", flush=True)
    except Exception as e:
        out["result"], out["abort_reason"] = "error", f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        stop.set()
        for th in threads:
            th.join(timeout=2.0)
        if server is not None:
            server.stop()
        if gripper is not None:
            gripper.close()  # sends nothing
        if link is not None and link.connected:
            try:
                p = link.park()  # mode 0, state 4: always
                out["after"] = {"pose": list(p.pose), "state": p.state, "error": p.error, "warn": p.warn, "cmdnum": p.cmdnum}
            except Exception as e:
                out["park_error"] = f"{type(e).__name__}: {e}"
                print(f"PARK FAILED: {e}. USE THE PHYSICAL STOP IF THE ARM IS MOVING.", flush=True)
            out["ledger"] = link.ledger.summary()
            link.close()
        end = time.monotonic_ns()
        if adapter is not None and backend is not None:
            out.update(elapsed_s=round((end - start) / 1e9, 2), ticks=adapter.ticks, engagements=adapter.eng.engagements,
                       final_state=adapter.eng.state, final_latch=list(adapter.eng.latch_reasons),
                       targets_sent=backend.command_count, stops=backend.stop_count, halts=backend.halt_count,
                       record_counts=rec.counts,
                       transitions=[dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.transitions],
                       faults=[dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.faults],
                       acks=[dict(x, t_s=round((x["t"] - start) / 1e9, 3)) for x in rec.acks], start_mono_ns=start)  # fmt: skip
            if gripper is not None:
                out["gripper"] = {"commands_sent": gripper.sent, "command_errors": gripper.send_errors, "end": gripper.refresh_state()}
        if state is not None:
            out["link"] = summarize(state, start, end)
        out["human_confirmation"] = a.human_confirmation
        for f in streams:
            f.close()
    text = ro.redact(json.dumps(out, indent=1, default=str), address)
    if a.summary is not None:
        a.summary.parent.mkdir(parents=True, exist_ok=True)
        with a.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    print(json.dumps({k: out.get(k) for k in ("result", "abort_reason", "engagements", "targets_sent", "halts", "final_state", "final_latch", "after")}, default=str), flush=True)
    return 0 if out["result"] == "ok" else 3


if __name__ == "__main__":
    sys.exit(main())
