#!/usr/bin/env python3
"""READ-ONLY observer of the xArm controller configuration. NOT a robot controller.

It opens one dedicated SDK connection, reads, and closes that connection. It never enables,
stops, moves, clears or configures anything, and it does not act as a safety function: if the
arm moves unexpectedly, the on-site person uses the site's stopping procedure.

Why three layers (installed SDK 1.18.4, read as source before this was written):
  * XArmAPI.connect() itself calls clean_warn() when a warning is present
    (x3/base.py _check_version, "if self.warn_code != 0: self.clean_warn()").
  * The SDK's report thread reconnects the report socket by itself.
  * By default the SDK switches the control connection to heartbeat protocol 3.
So:
  1. API guard   every public SDK method except a short allowlist is replaced, on this one
                 instance, by a stub that records the attempt and does nothing. The
                 constructor is called with do_not_open=True and the guard is installed
                 BEFORE connect().
  2. Wire guard  UxbusCmdTcp.send_modbus_request is wrapped in this process so that only
                 payload-free read registers (READ_REGISTERS) can reach the controller.
                 Anything else is refused locally and recorded.
  3. Lifecycle   no timed keep-alive thread, the switch to heartbeat protocol 3 is refused
                 (plain protocol 2 only), report reconnect disabled, bounded connect,
                 cleanup = close this connection only.

Every SDK operation used, every frame sent and every refusal is in the ledger of the snapshot.
Report-backed values are only taken from reports received on this connection, never from the
SDK's initialisation defaults. Times are this host's clock; the controller supplies no
timestamps, and separately read values are not synchronised with each other.
"""

from __future__ import annotations

import argparse
import inspect
import io
import json
import math
import re
import signal
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

TOOL = "xarm_readonly_preflight"
BANNER = "READ-ONLY OBSERVER: no enable, no mode/state change, no stop, no motion, no configuration write"

# Payload-free read registers of the private Modbus protocol (core/config/x_config.py UxbusReg).
READ_REGISTERS = {1: "GET_VERSION", 2: "GET_ROBOT_SN", 13: "GET_STATE", 14: "GET_CMDNUM", 15: "GET_ERROR",
                  41: "GET_TCP_POSE", 42: "GET_JOINT_POS", 49: "GET_REDUCED_MODE", 53: "GET_REDUCED_STATE",
                  91: "GET_TCP_POSE_AA"}  # fmt: skip
# Public SDK methods that stay callable on the guarded instance. Everything else is refused.
ALLOWED_API = frozenset({
    "connect", "disconnect",  # connection management
    "get_version", "get_robot_sn", "get_state", "get_cmdnum", "get_err_warn_code", "get_position",
    "get_servo_angle", "get_reduced_mode", "get_reduced_states",  # reads (registers above)
    "version_is_ge", "set_timeout", "log_api_info",  # local only: no frame is sent (set_timeout sets the client wait)
    "is_alive", "run", "start", "join",  # not SDK operations; present only if a base class is a Thread
})  # fmt: skip
NEVER = ("motion_enable", "set_mode", "set_state", "emergency_stop", "clean_error", "clean_warn", "reset", "move_gohome",
         "set_position", "set_servo_angle", "set_servo_angle_j", "set_servo_cartesian", "set_tcp_offset",
         "set_world_offset", "set_tcp_load", "set_reduced_mode", "set_reduced_tcp_boundary", "set_fence_mode",
         "set_collision_sensitivity", "set_servo_attach", "set_servo_detach", "set_gripper_position",
         "set_tgpio_digital", "set_cgpio_digital")  # fmt: skip
STATE_NAMES = {0: "motion (ready/moving)", 1: "in motion", 2: "sleeping/ready", 3: "suspended", 4: "stopped", 5: "state changed (reserved)", 6: "decelerating to stop"}
MODE_NAMES = {0: "position", 1: "servo", 2: "joint teaching", 3: "cartesian teaching (invalid)", 4: "joint velocity",
              5: "cartesian velocity", 6: "joint online trajectory planning", 7: "cartesian online trajectory planning"}  # fmt: skip
MOVE_EPS_MM, MOVE_EPS_DEG = 0.5, 0.1  # larger change during the stationary window is flagged


class Ledger:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.events: list = []

    def add(self, kind: str, **kw: Any) -> None:
        with self._lock:
            self.events.append(dict(t_mono_ns=time.monotonic_ns(), kind=kind, **kw))

    def summary(self) -> dict:
        with self._lock:
            ev = list(self.events)
        count = lambda kind, key: {str(k): sum(1 for e in ev if e["kind"] == kind and e[key] == k)  # noqa: E731
                                   for k in sorted({e[key] for e in ev if e["kind"] == kind}, key=str)}  # fmt: skip
        return {"sdk_api_calls": count("api_call", "name"), "wire_frames_sent_by_register": count("wire_sent", "register"),
                "api_calls_refused": count("api_refused", "name"),
                "wire_frames_refused": [{k: e[k] for k in ("register", "protocol", "payload_len")} for e in ev if e["kind"] == "wire_refused"],
                "notes": [e["note"] for e in ev if e["kind"] == "note"]}  # fmt: skip


def install_wire_guard(cmd_tcp_class: Any, ledger: Ledger, write_rules: dict | None = None) -> Callable[[], None]:
    """Allow only payload-free reads of READ_REGISTERS on the private protocol. Returns an undo.

    write_rules (None for the read-only inspector) lets a separate, explicitly commanding program
    admit specific writes: {register: (name, validator(payload bytes) -> bool)}. A write passes only
    if its register is listed AND its validator accepts the exact payload."""
    original = cmd_tcp_class.send_modbus_request
    write_rules = write_rules or {}

    def guarded(self, unit_id, pdu_data, pdu_len, prot_id=-1, t_id=None, debug=False):  # type: ignore[no-untyped-def]
        protocol = getattr(self, "_protocol_identifier", None) if prot_id < 0 else prot_id
        if protocol == 2 and unit_id in READ_REGISTERS and pdu_len == 0:
            ledger.add("wire_sent", register=f"{unit_id} {READ_REGISTERS[unit_id]}")
            return original(self, unit_id, pdu_data, pdu_len, prot_id=prot_id, t_id=t_id, debug=debug)
        rule = write_rules.get(unit_id) if protocol == 2 and pdu_len > 0 else None
        if rule is not None:
            try:
                ok = bool(rule[1](bytes(pdu_data[i] for i in range(pdu_len))))
            except Exception:
                ok = False
            if ok:
                ledger.add("wire_sent", register=f"{unit_id} {rule[0]}")
                return original(self, unit_id, pdu_data, pdu_len, prot_id=prot_id, t_id=t_id, debug=debug)
        ledger.add("wire_refused", register=unit_id, protocol=protocol, payload_len=pdu_len)
        return -1  # the SDK treats this as "not sent"

    cmd_tcp_class.send_modbus_request = guarded
    original_set = getattr(cmd_tcp_class, "set_protocol_identifier", None)
    noted: list = []

    def keep_protocol_2(self, protocol_identifier):  # type: ignore[no-untyped-def]
        # connect() re-arms the SDK's heartbeat switch (its _init() resets _keep_heart), and the report
        # thread then asks for protocol 3 on every pass. Refusing keeps this observer on plain protocol 2.
        if protocol_identifier != 2:
            if not noted:
                noted.append(1)
                ledger.add("note", note=f"SDK request to switch to heartbeat protocol {protocol_identifier} refused; staying on protocol 2")
            return -1
        return original_set(self, protocol_identifier)

    if original_set is not None:
        cmd_tcp_class.set_protocol_identifier = keep_protocol_2

    def undo() -> None:
        cmd_tcp_class.send_modbus_request = original
        if original_set is not None:
            cmd_tcp_class.set_protocol_identifier = original_set

    return undo


def install_api_guard(core: Any, ledger: Ledger, allowed: frozenset = ALLOWED_API) -> list:
    """Replace every public method outside `allowed` (default: the read-only ALLOWED_API) on this ONE
    instance with a recording stub."""
    refused = []
    for name, _fn in inspect.getmembers(type(core), predicate=inspect.isfunction):  # never evaluates properties
        if name.startswith("_") or name in allowed:
            continue

        def stub(*a: Any, _name: str = name, **k: Any) -> int:
            ledger.add("api_refused", name=_name)
            return -99

        setattr(core, name, stub)
        refused.append(name)
    return refused


def decode_rich_report(raw: bytes) -> dict:
    """Decode one rich report (port 30002) with the offsets of the installed SDK's
    x3/base.py __handle_report_normal / __handle_report_rich. Fields beyond the frame length are absent."""
    n = len(raw)
    f = lambda off, cnt: [round(v, 6) for v in struct.unpack_from("<%df" % cnt, raw, off)]  # noqa: E731
    deg = lambda rad: [round(math.degrees(v), 4) for v in rad]  # noqa: E731
    bits = lambda b: [(b >> i) & 1 for i in range(8)]  # noqa: E731
    d: dict = {"frame_length_field": struct.unpack_from(">I", raw, 0)[0], "frame_bytes": n}
    if n < 145:
        d["error"] = "frame too short"
        return d
    pose, off = f(35, 6), f(91, 6)
    load = f(115, 4)
    d.update(state=raw[4] & 0x0F, mode=raw[4] >> 4, cmd_num=struct.unpack_from(">H", raw, 5)[0],
             joint_angles_deg=deg(f(7, 7)), pose_mm_deg=[round(v, 3) for v in pose[:3]] + deg(pose[3:]),
             joint_torque=f(59, 7), motor_brake_bits=bits(raw[87]), motor_enable_bits=bits(raw[88]),
             error_code=raw[89], warn_code=raw[90],
             tcp_offset_mm_deg=[round(v, 3) for v in off[:3]] + deg(off[3:]),
             tcp_load_kg=round(load[0], 3), tcp_load_cog_mm=[round(v, 3) for v in load[1:]],
             collision_sensitivity=raw[131], teach_sensitivity=raw[132], gravity_direction=f(133, 3))  # fmt: skip
    if n >= 151:
        d.update(arm_type=raw[145], arm_axis=raw[146], master_id=raw[147], slave_id=raw[148], motor_tid=raw[149], motor_fid=raw[150])
    if n >= 245:
        tr, pp = f(181, 5), f(201, 5)
        d.update(tcp_jerk=tr[0], tcp_acc_limit_mm_s2=tr[1:3], tcp_speed_limit_mm_s=tr[3:5],
                 joint_jerk_rad=pp[0], joint_acc_limit_rad_s2=pp[1:3], joint_speed_limit_rad_s=pp[3:5],
                 servo_codes=[list(raw[229 + 2 * i:231 + 2 * i]) for i in range(8)])  # fmt: skip
    if n >= 284:
        sp = f(252, 8)
        d.update(realtime_tcp_speed_mm_s=sp[0], realtime_joint_speeds_rad_s=sp[1:])
    if n >= 288:
        d["report_counter"] = struct.unpack_from(">I", raw, 284)[0]
    if n >= 312:
        w = f(288, 6)
        d["world_offset_mm_deg"] = [round(v, 3) for v in w[:3]] + deg(w[3:])
    if n >= 417:
        d.update(is_simulation_robot=bool(raw[314]), collision_detection=raw[315], collision_tool_type=raw[316],
                 collision_tool_params=f(317, 6))  # fmt: skip
    if n >= 495:
        b = raw[494]
        d["flags_byte_494"] = {"reduced_mode": b & 1, "fence_mode": (b >> 1) & 1, "report_current": (b >> 2) & 1,
                               "approx_motion": (b >> 3) & 1, "cart_continuous": (b >> 4) & 1}  # fmt: skip
    if n >= 574:
        lim = f(516, 14)
        d.update(reduced_mode_on=raw[495], reduced_tcp_boundary_raw=list(struct.unpack_from(">6h", raw, 496)),
                 reduced_max_tcp_speed_mm_s=f(508, 1)[0], reduced_max_joint_speed_deg_s=deg(f(512, 1))[0],
                 reduced_joint_limits_deg=[deg(lim[2 * i:2 * i + 2]) for i in range(7)],
                 fence_on=raw[572], collision_rebound_on=raw[573])  # fmt: skip
    return d


def decode_boundary(raw6: Any) -> dict | None:
    """SDK order (set_reduced_tcp_boundary / get_reduced_states docs): [x_max, x_min, y_max, y_min, z_max, z_min], mm."""
    if not (isinstance(raw6, (list, tuple)) and len(raw6) == 6):
        return None
    return {"order": "x_max, x_min, y_max, y_min, z_max, z_min (mm, SDK documentation)", "raw": list(raw6),
            "x_mm": [raw6[1], raw6[0]], "y_mm": [raw6[3], raw6[2]], "z_mm": [raw6[5], raw6[4]]}  # fmt: skip


def address_from_file(path: Path) -> str:
    found = sorted(set(re.findall(r'XArmAPI\(\s*"((?:\d{1,3}\.){3}\d{1,3})"', path.read_text(encoding="utf-8", errors="replace"))))
    if len(found) != 1:
        raise SystemExit(f"error: expected exactly one XArmAPI(\"<ipv4>\") address in {path}, found {len(found)}")
    return found[0]


class Inspector:
    def __init__(self, api_class: Any, cmd_tcp_class: Any, address: str, connect_timeout_s: float = 8.0,
                 report_wait_s: float = 4.0, samples: int = 5, sample_interval_s: float = 0.5, min_reports: int = 5):  # fmt: skip
        if not re.fullmatch(r"(?:\d{1,3}\.){3}\d{1,3}", address):
            raise ValueError("address must be a dotted IPv4 address (anything else would select the SDK's serial path)")
        self.api_class, self.cmd_tcp_class, self.address = api_class, cmd_tcp_class, address
        self.connect_timeout_s, self.report_wait_s = connect_timeout_s, report_wait_s
        self.samples, self.sample_interval_s, self.min_reports = samples, sample_interval_s, min_reports
        self.ledger = Ledger()
        self.reports: list = []  # (host_mono_ns, raw bytes)
        self._rlock = threading.Lock()
        self.api: Any = None
        self.core: Any = None
        self.abort = threading.Event()
        self._undo_wire: Callable[[], None] | None = None

    # -- SDK access: every call goes through here and is logged ---------------------------------
    def _call(self, name: str, *a: Any, **k: Any) -> Any:
        if name not in ALLOWED_API:
            raise RuntimeError(f"inspector bug: {name} is not an allowed SDK operation")
        self.ledger.add("api_call", name=name)
        return getattr(self.api, name)(*a, **k)

    def _read(self, name: str, *a: Any, **k: Any) -> dict:
        """One read. The value is kept only if the SDK return code is 0."""
        t0, wall = time.monotonic_ns(), time.time()
        try:
            ret = self._call(name, *a, **k)
            code, value = (ret[0], ret[1]) if isinstance(ret, (tuple, list)) and len(ret) == 2 else (-100, None)
            err = None
        except Exception as e:  # a failed read never triggers anything on the robot
            code, value, err = -101, None, f"{type(e).__name__}: {e}"
        t1 = time.monotonic_ns()
        ok = code == 0
        return {"source": f"SDK {name}() (explicit query)", "code": code, "ok": ok, "value": value if ok else None,
                "error": err, "host_query_start_mono_ns": t0, "host_query_end_mono_ns": t1,
                "host_query_ms": round((t1 - t0) / 1e6, 2), "host_wall_time": wall}  # fmt: skip

    def _on_report(self, original: Callable[[Any], Any], data: Any) -> Any:
        with self._rlock:
            self.reports.append((time.monotonic_ns(), bytes(data)))
            if len(self.reports) > 400:
                del self.reports[1:-200]  # keep the first and the newest
        return original(data)

    def close(self) -> None:
        """Close only this observer's connection. No robot stop function is ever called here."""
        if self.api is not None:
            try:
                self._call("disconnect")
            except Exception as e:
                self.ledger.add("note", note=f"disconnect raised {type(e).__name__}: {e}")
        if self._undo_wire is not None:
            self._undo_wire()
            self._undo_wire = None

    def run(self) -> dict:
        snap: dict = {"tool": TOOL, "banner": BANNER, "result": "started", "host_wall_start": time.time(),
                      "host_mono_start_ns": time.monotonic_ns(), "address": "<xarm-ip> (redacted)",
                      "robot_changing_calls_made": 0}  # fmt: skip
        try:
            self._undo_wire = install_wire_guard(self.cmd_tcp_class, self.ledger)
            # Construction is separate from connection (do_not_open=True). No keep-alive thread,
            # no heartbeat, no tool-modbus baud check, no serial-number check, TCP only.
            self.ledger.add("api_call", name="XArmAPI.__init__(do_not_open=True)")
            self.api = self.api_class(self.address, is_radian=False, do_not_open=True, enable_report=True,
                                      report_type="rich", timed_comm=False, enable_heartbeat=False,
                                      baud_checkset=False, check_robot_sn=False, forbid_uds=True)  # fmt: skip
            self.core = self.api._arm
            if self.core.connected:
                raise RuntimeError("SDK connected during construction although do_not_open=True")
            snap["api_methods_refused_count"] = len(install_api_guard(self.core, self.ledger))
            for name in NEVER:  # the guard must cover every forbidden operation this SDK has
                if hasattr(type(self.core), name) and getattr(self.core, name).__name__ != "stub":
                    raise RuntimeError(f"guard missing for {name}")
            original = self.core._handle_report_data
            self.core._handle_report_data = lambda data: self._on_report(original, data)

            done: list = []

            def connect() -> None:
                try:
                    self._call("connect")
                    done.append(None)
                except Exception as e:
                    done.append(f"{type(e).__name__}: {e}")

            t_c0 = time.monotonic_ns()
            th = threading.Thread(target=connect, name="connect", daemon=True)
            th.start()
            th.join(self.connect_timeout_s)
            snap["connect_ms"] = round((time.monotonic_ns() - t_c0) / 1e6, 1)
            if th.is_alive() or done[0] is not None or not self.core.connected:
                snap["result"] = "connect_failed"
                snap["connect_error"] = "timeout" if th.is_alive() else done[0] or "not connected"
                return snap
            self.core._connect_report = lambda: self.ledger.add("note", note="SDK report reconnect attempt refused")  # no auto reconnect
            snap["report_socket_connected"] = bool(self.core.reported)

            pol: dict = {}
            for name in ("get_version", "get_robot_sn", "get_reduced_mode", "get_reduced_states"):
                pol[name] = self._read(name)
            series = []
            for i in range(self.samples):
                if self.abort.is_set():
                    break
                series.append({n: self._read(n) for n in ("get_state", "get_cmdnum", "get_err_warn_code", "get_position", "get_servo_angle")})
                if i + 1 < self.samples:
                    self.abort.wait(self.sample_interval_s)
            deadline = time.monotonic() + self.report_wait_s
            while time.monotonic() < deadline and not self.abort.is_set():
                with self._rlock:
                    if len(self.reports) >= self.min_reports:
                        break
                time.sleep(0.05)
            snap["polled_once"], snap["polled_series"] = pol, series
            snap.update(self._summarise(pol, series))
            snap["result"] = "aborted" if self.abort.is_set() else "ok"
            return snap
        except Exception as e:
            snap["result"], snap["exception"] = "error", f"{type(e).__name__}: {e}"
            return snap
        finally:
            self.close()
            led = self.ledger.summary()
            snap["ledger"] = led
            snap["robot_changing_calls_made"] = 0  # by construction; attempts are listed as refused
            snap["refused_attempts_total"] = sum(led["api_calls_refused"].values()) + len(led["wire_frames_refused"])
            snap["host_mono_end_ns"] = time.monotonic_ns()

    # -- interpretation (no SDK calls) -------------------------------------------------------------
    def _summarise(self, pol: dict, series: list) -> dict:
        out: dict = {}
        ver = pol["get_version"]["value"]
        out["firmware_version_string"] = ver
        nums = re.findall(r"(\d+)\.(\d+)\.(\d+)", ver or "")
        lead = re.match(r"\D*(\d+),(\d+),", ver or "")
        if nums:
            out["firmware"] = {"version": ".".join(nums[-1]), "axis": int(lead.group(1)) if lead else None,
                               "device_type": int(lead.group(2)) if lead else None,
                               "source": "get_version() string (register 1); last x.y.z group, leading 'axis,type,' if present"}  # fmt: skip
        with self._rlock:
            reps = list(self.reports)
        rep: dict = {"source": "rich report socket (port 30002) on this connection; host receive times only",
                     "received": len(reps), "verified_fresh": False}  # fmt: skip
        if reps:
            dec = [decode_rich_report(r) for _, r in reps]
            first, last = dec[0], dec[-1]
            gaps = [(b[0] - a[0]) / 1e6 for a, b in zip(reps, reps[1:])]
            counters = [d.get("report_counter") for d in dec]
            rep.update(first_host_mono_ns=reps[0][0], last_host_mono_ns=reps[-1][0],
                       span_ms=round((reps[-1][0] - reps[0][0]) / 1e6, 1),
                       interval_ms={"median": round(sorted(gaps)[len(gaps) // 2], 1), "max": round(max(gaps), 1)} if gaps else None,
                       counter_first=counters[0], counter_last=counters[-1], frame_bytes=sorted({len(r) for _, r in reps}),
                       last=last, last_raw_hex=reps[-1][1].hex())  # fmt: skip
            advancing = counters[0] is not None and counters[-1] is not None and counters[-1] != counters[0]
            rep["verified_fresh"] = len(reps) >= self.min_reports and "error" not in last and (advancing or counters[0] is None)
            rep["freshness_basis"] = (f"{len(reps)} reports received after connecting"
                                      + (", report counter advanced" if advancing else ", counter not available or not advancing"))  # fmt: skip
            if len(dec) > 1 and "error" not in last and "error" not in first:
                dj = max(max(abs(a - b) for a, b in zip(d["joint_angles_deg"], first["joint_angles_deg"])) for d in dec if "error" not in d)
                dp = max(math.dist(d["pose_mm_deg"][:3], first["pose_mm_deg"][:3]) for d in dec if "error" not in d)
                rep["stationary_check"] = {"max_joint_change_deg": round(dj, 4), "max_tcp_change_mm": round(dp, 3),
                                           "thresholds": [MOVE_EPS_DEG, MOVE_EPS_MM],
                                           "movement_observed": dj > MOVE_EPS_DEG or dp > MOVE_EPS_MM,
                                           "states_seen": sorted({d["state"] for d in dec if "error" not in d}),
                                           "max_reported_tcp_speed_mm_s": max((abs(d.get("realtime_tcp_speed_mm_s", 0.0)) for d in dec if "error" not in d), default=None)}  # fmt: skip
            # SDK cached properties after these reports, as a cross-check of the decoder
            try:
                c = self.core
                rep["sdk_properties_cross_check"] = {
                    "tcp_offset": list(c.position_offset), "world_offset": list(c.world_offset), "tcp_load": list(c.tcp_load),
                    "mode": c.mode, "collision_sensitivity": c.collision_sensitivity, "first_report_over": bool(c._first_report_over)}  # fmt: skip
            except Exception as e:
                rep["sdk_properties_cross_check"] = f"unavailable: {type(e).__name__}: {e}"
        out["report"] = rep
        if not rep["verified_fresh"]:
            rep["note"] = "report-backed fields are NOT accepted: no verified fresh report (SDK defaults are never reported as measurements)"
        rs = pol["get_reduced_states"]
        if rs["ok"] and isinstance(rs["value"], (list, tuple)) and len(rs["value"]) >= 2:
            out["reduced_states_decoded"] = {"reduced_mode_on": rs["value"][0], "tcp_boundary": decode_boundary(rs["value"][1]),
                                             "raw_structure": rs["value"], "source": "get_reduced_states() register 53"}  # fmt: skip
        poses = [s["get_position"]["value"] for s in series if s["get_position"]["ok"]]
        if len(poses) > 1:
            out["polled_pose_spread_mm"] = round(max(math.dist(p[:3], poses[0][:3]) for p in poses), 4)
        return out


def load_installed_sdk() -> tuple:
    from xarm import version as sdk_version  # the installed SDK, used directly
    from xarm.core.wrapper.uxbus_cmd_tcp import UxbusCmdTcp
    from xarm.wrapper import XArmAPI

    return XArmAPI, UxbusCmdTcp, getattr(sdk_version, "__version__", "?")


def redact(text: str, address: str) -> str:
    return re.sub(r"(?:\d{1,3}\.){3}\d{1,3}", "<ip>", text.replace(address, "<xarm-ip>"))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--address-from-file", type=Path, help='take the one XArmAPI("<ipv4>") address from this existing source file')
    src.add_argument("--address")
    ap.add_argument("--connect", action="store_true", help="actually connect; without it only the plan is printed")
    ap.add_argument("--human-confirmation", default="", help="who confirmed the on-site checks (recorded); required with --connect")
    ap.add_argument("--out", type=Path, help="snapshot JSON (must not exist)")
    ap.add_argument("--samples", type=int, default=5)
    args = ap.parse_args(argv)
    address = args.address or address_from_file(args.address_from_file)
    if args.out is not None and args.out.exists():
        print(f"error: {args.out} already exists; refusing to overwrite", file=sys.stderr)
        return 2
    print(BANNER, flush=True)
    print(f"read registers allowed on the wire: {sorted(READ_REGISTERS.values())}", flush=True)
    if not args.connect:
        print("plan only: --connect not given, no connection was made", flush=True)
        return 0
    if len(args.human_confirmation.strip()) < 10:
        print("error: --connect needs --human-confirmation describing the on-site checks", file=sys.stderr)
        return 2
    api_class, cmd_class, sdk_ver = load_installed_sdk()
    insp = Inspector(api_class, cmd_class, address, samples=args.samples)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: insp.abort.set())  # ends the read loop; cleanup closes the connection only
    real_out, captured = sys.stdout, io.StringIO()
    sys.stdout = captured  # the SDK prints the address and serial numbers
    try:
        snap = insp.run()
    finally:
        sys.stdout = real_out
    snap["sdk_version"] = sdk_ver
    snap["address_source"] = str(args.address_from_file) if args.address_from_file else "--address"
    snap["human_confirmation"] = args.human_confirmation
    snap["sdk_stdout"] = redact(captured.getvalue(), address).splitlines()
    text = redact(json.dumps(snap, indent=1, default=str), address)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    moved = (snap.get("report", {}).get("stationary_check") or {}).get("movement_observed")
    brief = {k: snap.get(k) for k in ("result", "connect_error", "exception", "connect_ms", "refused_attempts_total")}
    print(json.dumps(brief), flush=True)
    if moved:
        print("ALERT: MOVEMENT WAS OBSERVED DURING THE STATIONARY WINDOW. Tell the on-site person. This program does not stop the robot.", flush=True)
        return 4
    return 0 if snap["result"] == "ok" else 3


if __name__ == "__main__":
    sys.exit(main())
