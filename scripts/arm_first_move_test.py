#!/usr/bin/env python3
"""FIRST SUPERVISED ARM MOVEMENT TEST. THIS MOVES THE REAL ARM (a few millimetres, slowly).

Purpose: establish, on the real controller, the three things source reading could not:
  A. a single small line move works through our command link, and which way it goes;
  B. whether a new target sent during a move REPLACES the old one or is QUEUED behind it;
  C. what set_state(4) does to a move in progress (time and distance to stop).

Everything stays within `--distance` (default 10 mm) of where the arm is, along one base axis,
toward the inside of the fence, at `--speed` (default 10 mm/s). The orientation read from the
controller is sent back unchanged. The command link refuses, before anything reaches the wire,
any target outside a box of that size around the start and any speed above the limit.

Without --moves-real-arm it does not connect at all and only prints what it would do. With it, a
--human-confirmation text is required.

It never clears an error or warning. On any anomaly it requests set_state(4) and stops testing.
At the end it leaves the controller as the lab's code does: mode 0, state 4.
set_state(4) is a software stop. It is not the physical emergency stop: keep that within reach.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

import xarm_command_link as cl
import xarm_readonly_preflight as ro
from teleop_rotation import q_conj, q_from_rotvec, q_mul, q_to_rotvec, q_to_rpy, rpy_to_q

FENCE_MARGIN_MM = 5.0
POLL_S = 0.02
ARM_SETTLE_S = 1.0  # pause after arming before the first target, as the vendor's examples do


class Abort(RuntimeError):
    pass


def plan(pose: tuple, boundary: dict, axis: str, distance: float) -> dict:
    """Direction along `axis` that leads away from the nearest fence wall; hard box around the start."""
    k = "xyz".index(axis)
    lo, hi = boundary[f"{axis}_mm"]
    sign = 1.0 if (pose[k] - lo) <= (hi - pose[k]) else -1.0
    end = pose[k] + sign * distance
    if not (lo + FENCE_MARGIN_MM <= end <= hi - FENCE_MARGIN_MM):
        raise Abort(f"a {distance:g} mm move along {axis} would end {end:.1f} mm, within {FENCE_MARGIN_MM:g} mm of the fence [{lo}, {hi}]")
    box_min, box_max = [pose[i] - 1.5 for i in range(3)], [pose[i] + 1.5 for i in range(3)]
    box_min[k], box_max[k] = min(pose[k], end) - 1.5, max(pose[k], end) + 1.5
    return {"axis": axis, "k": k, "sign": sign, "start_mm": [round(v, 3) for v in pose[:3]], "end_coordinate_mm": round(end, 3),
            "box_min_mm": [round(v, 3) for v in box_min], "box_max_mm": [round(v, 3) for v in box_max]}  # fmt: skip


class FirstMove:
    def __init__(self, link: cl.XArmCommandLink, pl: dict, distance: float, speed: float, acc: float, mode: int, log: list):
        self.link, self.pl, self.d, self.speed, self.acc, self.mode, self.log = link, pl, distance, speed, acc, mode, log
        self.k, self.sign = pl["k"], pl["sign"]
        self.start: tuple | None = None
        self.stop_requested = False

    def note(self, kind: str, **kw: Any) -> None:
        self.log.append(dict(k=kind, t=time.monotonic_ns(), **kw))

    def along(self, pose: tuple) -> float:
        """Signed travel from the start along the test direction, mm."""
        return (pose[self.k] - self.start[self.k]) * self.sign  # type: ignore[index]

    def target(self, travel: float) -> tuple:
        p = list(self.start)  # type: ignore[arg-type]
        p[self.k] += self.sign * travel
        return tuple(p)

    def check(self, s: cl.ArmState) -> None:
        if self.stop_requested:
            raise Abort("stop requested by the operator")
        if s.error or s.warn:
            raise Abort(f"controller error {s.error} / warning {s.warn}")
        off = max(abs(s.pose[i] - self.start[i]) for i in range(3) if i != self.k)  # type: ignore[index]
        if off > 2.0:
            raise Abort(f"moved {off:.2f} mm off the test axis")
        if not (-2.0 <= self.along(s.pose) <= self.d + 2.0):
            raise Abort(f"position {self.along(s.pose):.2f} mm is outside the planned 0..{self.d:g} mm")
        rot = max(abs(math.remainder(s.pose[i] - self.start[i], math.tau)) for i in range(3, 6))  # type: ignore[index]  # wrap-safe near +-180 deg
        if rot > math.radians(1.0):
            raise Abort(f"orientation changed by {math.degrees(rot):.2f} deg")

    def watch(self, seconds: float, label: str, until: Any = None) -> list:
        """Poll the pose; returns [(t_s, travel_mm, state, cmdnum)]. Every sample is checked."""
        t0, t0_ns, tr = time.monotonic(), time.monotonic_ns(), []
        while time.monotonic() - t0 < seconds:
            s = self.link.read()
            tr.append((round(time.monotonic() - t0, 3), round(self.along(s.pose), 3), s.state, s.cmdnum))
            self.check(s)
            if until is not None and until(tr):
                break
            time.sleep(POLL_S)
        self.note("trajectory", label=label, t0_mono_ns=t0_ns, samples=tr)
        return tr

    @staticmethod
    def settled(tr: list, at: float, tol: float = 0.3, n: int = 10) -> bool:
        return len(tr) >= n and all(abs(x[1] - at) <= tol for x in tr[-n:])

    def run(self) -> dict:
        res: dict = {}
        s0 = self.link.read()
        self.start = s0.pose
        self.note("start", pose=list(s0.pose), state=s0.state, cmdnum=s0.cmdnum)
        T = self.d / self.speed
        armed = self.link.arm_motion(self.mode)
        self.note("armed", mode=self.mode, state=armed.state, reported_mode=self.link.reported_mode())
        time.sleep(ARM_SETTLE_S)

        # A. one move out, one move back
        self.link.move_line(self.target(self.d), self.speed, self.acc)
        tr = self.watch(T + 3.0, "A out", until=lambda t: self.settled(t, self.d))
        first = next((x[0] for x in tr if x[1] > 0.2), None)
        res["A_single_move"] = {"reached_mm": tr[-1][1], "target_mm": self.d, "reached": self.settled(tr, self.d),
                                "first_motion_after_s": first, "duration_s": tr[-1][0],
                                "peak_speed_mm_s": round(max(((b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(tr, tr[5:]) if b[0] > a[0]), default=0.0), 1),
                                "states_seen": sorted({x[2] for x in tr}), "max_cmdnum": max(x[3] for x in tr)}  # fmt: skip
        if not res["A_single_move"]["reached"]:
            raise Abort("test A: the arm did not reach the target")
        self.link.move_line(self.target(0.0), self.speed, self.acc)
        tr = self.watch(T + 3.0, "A back", until=lambda t: self.settled(t, 0.0))
        if not self.settled(tr, 0.0):
            raise Abort("test A: the arm did not return to the start")

        # B. second target while the first is in progress: replaced or queued?
        t_switch = min(0.4 * T, 0.6)
        self.link.move_line(self.target(self.d), self.speed, self.acc)
        self.watch(t_switch, "B first target")
        self.link.move_line(self.target(0.0), self.speed, self.acc)
        tr = self.watch(2 * T + 3.0, "B after second target", until=lambda t: self.settled(t, 0.0))
        peak = max(x[1] for x in tr)
        res["B_second_target"] = {"second_target_sent_after_s": round(t_switch, 2), "peak_travel_mm": peak, "first_target_mm": self.d,
                                  "verdict": "QUEUED (went to the first target before returning)" if peak >= self.d - 0.5
                                  else "REPLACED (turned back before reaching the first target)",
                                  "returned_to_start": self.settled(tr, 0.0), "max_cmdnum": max(x[3] for x in tr)}  # fmt: skip
        if not self.settled(tr, 0.0):
            raise Abort("test B: the arm did not return to the start")

        # C. set_state(4) during a move
        self.link.move_line(self.target(self.d), self.speed, self.acc)
        pre = self.watch(min(0.5 * T, 0.6), "C before stop")
        t_req, t_req_ns = time.monotonic(), time.monotonic_ns()
        after_halt = self.link.halt()
        t_ack = time.monotonic()
        tr, t0 = [], time.monotonic()
        while time.monotonic() - t0 < 1.5:  # read directly: check() would abort on state 4 semantics only if off-plan
            s = self.link.read()
            tr.append((round(time.monotonic() - t_req, 3), round(self.along(s.pose), 3), s.state, s.cmdnum))
            self.check(s)
            time.sleep(POLL_S)
        self.note("trajectory", label="C after stop", t0_mono_ns=t_req_ns, samples=tr)
        at_req, final = pre[-1][1], tr[-1][1]
        still = next((x[0] for i, x in enumerate(tr) if all(abs(y[1] - final) <= 0.05 for y in tr[i:])), None)
        res["C_stop_during_move"] = {"travel_when_requested_mm": at_req, "final_travel_mm": final,
                                     "distance_after_request_mm": round(final - at_req, 3), "stop_call_took_ms": round((t_ack - t_req) * 1000, 1),
                                     "stationary_after_s": still, "state_after": after_halt.state, "cmdnum_after": tr[-1][3],
                                     "resumed_by_itself": final >= self.d - 0.5}  # fmt: skip
        # return to the start: needs an explicit re-arm after the stop
        self.link.arm_motion(self.mode)
        time.sleep(ARM_SETTLE_S)
        self.link.move_line(self.target(0.0), self.speed, self.acc)
        tr = self.watch(T + 3.0, "C back", until=lambda t: self.settled(t, 0.0))
        res["returned_to_start"] = self.settled(tr, 0.0)
        return res


class RotationCheck:
    """A small pure rotation of the tool about one base axis and back (the flange position is not moved).
    Tells us that an orientation target works through our link, which way it turns, and how fast."""

    def __init__(self, link: cl.XArmCommandLink, axis: str, angle_deg: float, speed: float, acc: float, mode: int, log: list):
        self.link, self.axis, self.angle, self.speed, self.acc, self.mode, self.log = link, axis, math.radians(angle_deg), speed, acc, mode, log
        self.u = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}[axis]
        self.start: tuple | None = None
        self.q0: tuple | None = None
        self.stop_requested = False

    def turned(self, pose: tuple) -> tuple:
        """(rotation about the test axis, rotation about anything else), radians, since the start."""
        r = q_to_rotvec(q_mul(rpy_to_q(*pose[3:6]), q_conj(self.q0)))  # type: ignore[arg-type]
        on = sum(r[k] * self.u[k] for k in range(3))
        return on, math.sqrt(max(0.0, sum(v * v for v in r) - on * on))

    def check(self, s: cl.ArmState) -> None:
        if self.stop_requested:
            raise Abort("stop requested by the operator")
        if s.error or s.warn:
            raise Abort(f"controller error {s.error} / warning {s.warn}")
        if math.dist(s.pose[:3], self.start[:3]) > 2.0:  # type: ignore[index]
            raise Abort(f"the flange moved {math.dist(s.pose[:3], self.start[:3]):.2f} mm during a pure rotation")  # type: ignore[index]
        on, other = self.turned(s.pose)
        if not (-math.radians(1.5) <= on <= self.angle + math.radians(1.5)) or other > math.radians(1.5):
            raise Abort(f"rotation {math.degrees(on):.2f} deg about {self.axis}, {math.degrees(other):.2f} deg off-axis: outside the plan")

    def watch(self, seconds: float, label: str, goal_rad: float) -> list:
        t0, t0_ns, tr = time.monotonic(), time.monotonic_ns(), []
        while time.monotonic() - t0 < seconds:
            s = self.link.read()
            tr.append((round(time.monotonic() - t0, 3), round(math.degrees(self.turned(s.pose)[0]), 3), s.state))
            self.check(s)
            if len(tr) >= 10 and all(abs(x[1] - math.degrees(goal_rad)) <= 0.15 for x in tr[-10:]):
                break
            time.sleep(POLL_S)
        self.log.append(dict(k="trajectory", t=time.monotonic_ns(), label=label, t0_mono_ns=t0_ns, unit="deg about " + self.axis, samples=tr))
        return tr

    def run(self) -> dict:
        s0 = self.link.read()
        self.start, self.q0 = s0.pose, rpy_to_q(*s0.pose[3:6])
        err = math.degrees(self.link.rpy_convention_error_rad())
        res: dict = {"rpy_convention_error_deg": round(err, 4)}
        if err > 0.2:
            raise Abort(f"roll/pitch/yaw convention check failed: {err:.2f} deg between our reading and the controller's axis-angle; nothing was commanded")
        self.link.arm_motion(self.mode)
        time.sleep(ARM_SETTLE_S)
        q_t = q_mul(q_from_rotvec(tuple(self.angle * v for v in self.u)), self.q0)
        target = tuple(self.start[:3]) + tuple(q_to_rpy(q_t))
        self.link.move_line(target, self.speed, self.acc)
        tr = self.watch(8.0, "R out", self.angle)
        reached = len(tr) >= 10 and all(abs(x[1] - math.degrees(self.angle)) <= 0.15 for x in tr[-10:])
        at = self.link.read()
        res["R_rotation"] = {
            "axis": "base " + self.axis, "target_deg": round(math.degrees(self.angle), 2), "turned_deg": tr[-1][1], "reached": reached,
            "first_motion_after_s": next((x[0] for x in tr if abs(x[1]) > 0.1), None), "duration_s": tr[-1][0],
            "peak_deg_s": round(max(((b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(tr, tr[5:]) if b[0] > a[0]), default=0.0), 1),
            "speed_argument": self.speed, "start_rpy_deg": [round(math.degrees(v), 3) for v in self.start[3:6]],
            "target_rpy_deg": [round(math.degrees(v), 3) for v in target[3:6]], "reported_rpy_deg": [round(math.degrees(v), 3) for v in at.pose[3:6]],
            "flange_shift_mm": round(math.dist(at.pose[:3], self.start[:3]), 3)}  # fmt: skip
        if not reached:
            raise Abort("rotation check: the tool did not reach the target orientation")
        self.link.move_line(self.start, self.speed, self.acc)
        tr = self.watch(8.0, "R back", 0.0)
        res["returned_to_start"] = len(tr) >= 10 and all(abs(x[1]) <= 0.15 for x in tr[-10:])
        return res


def main(argv: list[str] | None = None, sdk: tuple | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--address-from-file", type=Path)
    src.add_argument("--address")
    ap.add_argument("--moves-real-arm", action="store_true", help="required to connect and move")
    ap.add_argument("--human-confirmation", default="")
    ap.add_argument("--axis", choices=["x", "y", "z"], default="y")
    ap.add_argument("--distance", type=float, default=10.0, help="mm (max 20)")
    ap.add_argument("--speed", type=float, default=10.0, help="mm/s (max 20)")
    ap.add_argument("--acc", type=float, default=100.0, help="mm/s^2 (max 500)")
    ap.add_argument("--mode", type=int, choices=[0, 7], default=7)
    ap.add_argument("--allow-reduced-mode-off", action="store_true",
                    help="run although the controller's reduced mode is off. The operator's explicit decision, recorded; "
                         "this program still refuses anything above --speed or outside its small box")
    ap.add_argument("--rotate-deg", type=float, default=0.0,
                    help="instead of the translation tests: turn the tool by this angle (2..15) about a base axis and back, without moving the flange")
    ap.add_argument("--rot-axis", choices=["x", "y", "z"], default="z")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args(argv)
    if args.rotate_deg and not (2.0 <= args.rotate_deg <= 15.0):
        print("error: --rotate-deg must be 2..15", file=sys.stderr)
        return 2
    if not (2.0 <= args.distance <= 20.0 and 2.0 <= args.speed <= 20.0 and 20.0 <= args.acc <= 500.0):
        print("error: distance 2..20 mm, speed 2..20 mm/s, acc 20..500 mm/s^2", file=sys.stderr)
        return 2
    if args.rotate_deg:
        print(f"ARM ROTATION CHECK: {args.rotate_deg:g} deg about base {args.rot_axis} and back, flange position fixed, mode {args.mode}.", flush=True)
    else:
        print(f"FIRST ARM MOVEMENT TEST: {args.distance:g} mm along base {args.axis} at {args.speed:g} mm/s, mode {args.mode}. "
              "Tests: A single move, B second target during a move, C set_state(4) during a move.", flush=True)  # fmt: skip
    if not args.moves_real_arm:
        print("plan only: --moves-real-arm not given; no connection was made", flush=True)
        return 0
    if len(args.human_confirmation.strip()) < 10:
        print("error: --moves-real-arm needs --human-confirmation", file=sys.stderr)
        return 2
    if args.out is not None and args.out.exists():
        print(f"error: {args.out} already exists", file=sys.stderr)
        return 2
    address = args.address or ro.address_from_file(args.address_from_file)
    api_class, cmd_class, sdk_ver = sdk or cl.load_sdk()
    log: list = []
    out: dict = {"tool": "arm_first_move_test", "sdk": sdk_ver, "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items() if k != "address"},
                 "result": "started", "host_wall_start": time.time()}  # fmt: skip
    link: cl.XArmCommandLink | None = None
    test: Any = None
    try:
        # 1. read-only look first, with a link that cannot move anything (degenerate box around nothing)
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
                             "is_simulation_robot": sim, "reported_mode": probe.reported_mode(),
                             "version": probe.version, "reduced_mode_on": rs[0], "fence_on": rs[5] if len(rs) > 5 else None, "boundary": boundary}  # fmt: skip
            if sim is not False:
                raise Abort(("the controller is in SIMULATION mode: the real arm would not move" if sim else
                             "no status report from the controller, so real vs simulation is unknown") + "; nothing was commanded")
            if s.error or s.warn:
                raise Abort(f"controller error {s.error} / warning {s.warn}; nothing was commanded")
            if s.cmdnum != 0 or s.state not in (2, 4):
                raise Abort(f"arm is not idle (state {s.state}, {s.cmdnum} queued commands); nothing was commanded")
            if not (len(rs) > 5 and rs[5] == 1):
                raise Abort("the fence (safety boundary) is not on; nothing was commanded")
            if rs[0] != 1:
                if not args.allow_reduced_mode_off:
                    raise Abort("reduced mode is not on; nothing was commanded (the operator can waive this with --allow-reduced-mode-off)")
                out["before"]["reduced_mode_waived_by_operator"] = True
            s2 = (time.sleep(0.5), probe.read())[1]
            if max(abs(a - b) for a, b in zip(s.pose[:3], s2.pose[:3])) > 0.2:
                raise Abort("the arm is moving; nothing was commanded")
            pl = plan(s2.pose, boundary, args.axis, args.distance)
            out["plan"] = pl
        finally:
            out["probe_ledger"] = probe.ledger.summary()
            probe.close()
        print(f"plan: from {pl['start_mm']} move {'+' if pl['sign'] > 0 else '-'}{args.axis} to {args.axis}={pl['end_coordinate_mm']} "
              f"(hard box {pl['box_min_mm']} .. {pl['box_max_mm']})", flush=True)  # fmt: skip
        # 2. the commanding link, boxed to this plan
        if args.rotate_deg:
            box = ([round(v - 1.5, 3) for v in s2.pose[:3]], [round(v + 1.5, 3) for v in s2.pose[:3]])
            out["plan"] = {"rotation_deg": args.rotate_deg, "axis": args.rot_axis, "box_min_mm": box[0], "box_max_mm": box[1]}
            link = cl.XArmCommandLink(api_class, cmd_class, address, tuple(box[0]), tuple(box[1]), args.speed, args.acc,
                                      ref_rpy=tuple(s2.pose[3:6]), max_rot_rad=math.radians(args.rotate_deg + 2.0))  # fmt: skip
            link.connect()
            test = RotationCheck(link, args.rot_axis, args.rotate_deg, args.speed, args.acc, args.mode, log)
        else:
            link = cl.XArmCommandLink(api_class, cmd_class, address, tuple(pl["box_min_mm"]), tuple(pl["box_max_mm"]), args.speed, args.acc)
            link.connect()
            test = FirstMove(link, pl, args.distance, args.speed, args.acc, args.mode, log)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda *_: setattr(test, "stop_requested", True))
        out["tests"] = test.run()
        out["result"] = "ok"
    except (Abort, cl.LinkError) as e:
        out["result"], out["abort_reason"] = "aborted", str(e)
        print(f"ABORTED: {e}", flush=True)
    except Exception as e:
        out["result"], out["abort_reason"] = "error", f"{type(e).__name__}: {e}"
        print(f"ERROR: {type(e).__name__}: {e}", flush=True)
    finally:
        if link is not None and link.connected:
            try:
                p = link.park()  # state 4, mode 0: always, also after an abort
                out["after"] = {"pose": list(p.pose), "state": p.state, "error": p.error, "warn": p.warn, "cmdnum": p.cmdnum}
                if test is not None and test.start is not None:
                    out["after"]["distance_from_start_mm"] = round(math.dist(p.pose[:3], test.start[:3]), 3)
            except Exception as e:
                out["park_error"] = f"{type(e).__name__}: {e}"
                print(f"PARK FAILED: {e}. USE THE PHYSICAL STOP IF THE ARM IS MOVING.", flush=True)
            out["ledger"] = link.ledger.summary()
            link.close()
        out["log"] = log
        out["human_confirmation"] = args.human_confirmation
    text = ro.redact(json.dumps(out, indent=1, default=str), address)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    print(json.dumps({k: out.get(k) for k in ("result", "abort_reason", "tests", "after")}, indent=1, default=str), flush=True)
    return 0 if out["result"] == "ok" else 3


if __name__ == "__main__":
    sys.exit(main())
