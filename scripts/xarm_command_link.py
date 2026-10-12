"""COMMANDING link to the xArm controller. THIS CAN MOVE THE ARM.

Imported only by programs that say so in their name and require an explicit flag
(arm_first_move_test.py, the future live teleop entry point). Never by the dry run or the
read-only inspector.

It uses the installed SDK for transport, with the same three protections as the inspector
(xarm_readonly_preflight.py), widened by exactly four writes:

  register 11  MOTION_EN   payload [8, 1]            enable all joints (never disable)
  register 19  SET_MODE    payload [0] or [7]        position mode / cartesian online planning
  register 12  SET_STATE   payload [0] or [4]        ready / stop
  register 21  MOVE_LINE   absolute base-frame line move, target inside the HARD BOX given at
                           construction, speed and acceleration not above the given maxima

Every other write is refused before it reaches the wire: clean_error, clean_warn, joint moves,
servo mode, TCP/world/payload/collision/reduced-mode/fence setters, gripper, GPIO. An error or
warning on the controller is reported and makes arming and moving fail; it is never cleared here.

Nothing is automatic: arm_motion() (enable + mode + state 0) and halt() (state 4) happen only when
the calling program asks, no reconnect, no retry. Timeouts are short so that a stalled reply cannot
hold the stop request for long. halt() is the SDK-level stop, set_state(4); it is NOT a hardware
emergency stop and its physical effect has to be verified on the rig.

Units at this boundary: millimetres, mm/s, mm/s^2; orientation in RADIANS exactly as the controller
reports it (so it can be passed back unchanged).
"""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

import xarm_readonly_preflight as ro
from teleop_rotation import q_angle, q_from_rotvec, rpy_to_q

MODE_POSITION, MODE_ONLINE = 0, 7
MOVE_LINE_BYTES = bytes([0, 0, 0])  # coord = base, not axis-angle, execute (not check-only)
# check_is_simulation_robot is a local flag test the SDK makes inside motion_enable/set_state (no frame).
ALLOWED_API = frozenset(ro.ALLOWED_API | {"motion_enable", "set_mode", "set_state", "check_is_simulation_robot", "get_position_aa"})


class LinkError(RuntimeError):
    """A read or command did not succeed. Nothing is retried."""


@dataclass(frozen=True)
class ArmState:
    pose: tuple  # x, y, z in mm; roll, pitch, yaw in rad (as reported)
    state: int
    error: int
    warn: int
    cmdnum: int
    t_mono_ns: int  # host clock when the last of the reads completed
    read_ms: float


def make_write_rules(box_min_mm: tuple, box_max_mm: tuple, max_speed_mm_s: float, max_acc_mm_s2: float,
                     ref_rpy: tuple | None = None, max_rot_rad: float | None = None) -> dict:
    """ref_rpy + max_rot_rad (optional): a line target whose orientation is further than max_rot_rad from the
    reference orientation is refused too."""
    q_ref = None if ref_rpy is None else rpy_to_q(*ref_rpy)

    def move_ok(p: bytes) -> bool:
        if len(p) != 43 or p[40:] != MOVE_LINE_BYTES:
            return False
        v = struct.unpack("<10f", p[:40])
        if not all(math.isfinite(x) for x in v):
            return False
        if not all(box_min_mm[k] <= v[k] <= box_max_mm[k] for k in range(3)):
            return False
        if q_ref is not None and q_angle(q_ref, rpy_to_q(v[3], v[4], v[5])) > max_rot_rad:
            return False
        return 0.0 < v[6] <= max_speed_mm_s + 1e-6 and 0.0 < v[7] <= max_acc_mm_s2 + 1e-6 and v[8] == 0.0 and v[9] == -1.0

    return {11: ("MOTION_EN(enable all)", lambda p: p == bytes([8, 1])),
            19: ("SET_MODE(0|7)", lambda p: p in (bytes([0]), bytes([7]), bytes([0, 0]), bytes([7, 0]))),
            12: ("SET_STATE(0|4)", lambda p: p in (bytes([0]), bytes([4]))),
            21: ("MOVE_LINE(in box)", move_ok)}  # fmt: skip


class XArmCommandLink:
    def __init__(self, api_class: Any, cmd_tcp_class: Any, address: str, box_min_mm: tuple, box_max_mm: tuple,
                 max_speed_mm_s: float, max_acc_mm_s2: float, io_timeout_s: float = 0.5,
                 connect_timeout_s: float = 8.0, clock: Callable[[], int] = time.monotonic_ns,
                 ref_rpy: tuple | None = None, max_rot_rad: float | None = None):  # fmt: skip
        if (ref_rpy is None) != (max_rot_rad is None):
            raise ValueError("ref_rpy and max_rot_rad go together")
        self.ref_rpy, self.max_rot_rad = ref_rpy, max_rot_rad
        import re
        if not re.fullmatch(r"(?:\d{1,3}\.){3}\d{1,3}", address):
            raise ValueError("address must be a dotted IPv4 address")
        if not all(box_min_mm[k] < box_max_mm[k] for k in range(3)) or max_speed_mm_s <= 0 or max_acc_mm_s2 <= 0:
            raise ValueError("bad hard box or limits")
        self.api_class, self.cmd_tcp_class, self.address, self.clock = api_class, cmd_tcp_class, address, clock
        self.box_min, self.box_max = tuple(map(float, box_min_mm)), tuple(map(float, box_max_mm))
        self.max_speed, self.max_acc = float(max_speed_mm_s), float(max_acc_mm_s2)
        self.io_timeout_s, self.connect_timeout_s = io_timeout_s, connect_timeout_s
        self.ledger = ro.Ledger()
        self.api: Any = None
        self.core: Any = None
        self._undo: Callable[[], None] | None = None
        self.connected = False
        self.armed_mode: int | None = None  # set by arm_motion(), cleared by halt()/any failed command
        self.version: str | None = None

    # -- logged SDK access ----------------------------------------------------------------------
    def _call(self, name: str, *a: Any, **k: Any) -> Any:
        if name not in ALLOWED_API:
            raise RuntimeError(f"link bug: {name} is not an allowed SDK operation")
        self.ledger.add("api_call", name=name)
        return getattr(self.api, name)(*a, **k)

    def connect(self) -> None:
        self._undo = ro.install_wire_guard(self.cmd_tcp_class, self.ledger,
                                           make_write_rules(self.box_min, self.box_max, self.max_speed, self.max_acc,
                                                            self.ref_rpy, self.max_rot_rad))  # fmt: skip
        self.ledger.add("api_call", name="XArmAPI.__init__(do_not_open=True)")
        self.api = self.api_class(self.address, is_radian=True, do_not_open=True, enable_report=True, report_type="rich",
                                  timed_comm=False, enable_heartbeat=False, baud_checkset=False, check_robot_sn=False,
                                  forbid_uds=True)  # fmt: skip
        self.core = self.api._arm
        if self.core.connected:
            raise LinkError("SDK connected during construction although do_not_open=True")
        ro.install_api_guard(self.core, self.ledger, ALLOWED_API)
        for name in ("clean_error", "clean_warn", "emergency_stop", "set_servo_angle", "set_servo_cartesian", "set_tcp_offset",
                     "set_reduced_mode", "set_fence_mode", "set_gripper_position", "reset", "move_gohome"):  # fmt: skip
            if hasattr(type(self.core), name) and getattr(self.core, name).__name__ != "stub":
                raise LinkError(f"guard missing for {name}")
        done: list = []

        def run() -> None:
            try:
                self._call("connect")
                done.append(None)
            except Exception as e:
                done.append(f"{type(e).__name__}: {e}")

        th = threading.Thread(target=run, name="connect", daemon=True)
        th.start()
        th.join(self.connect_timeout_s)
        if th.is_alive() or done[0] is not None or not self.core.connected:
            err = "timeout" if th.is_alive() else done[0] or "not connected"
            self.close()
            raise LinkError(f"connect failed: {err}")
        self.core._connect_report = lambda: self.ledger.add("note", note="SDK report reconnect attempt refused")
        self._call("set_timeout", self.io_timeout_s)  # local wait time only; no frame is sent
        self.connected = True
        code, ver = self._call("get_version")
        self.version = ver if code == 0 else None

    def close(self) -> None:
        """Closes this connection only. Does NOT stop the arm: call halt() first if that is wanted."""
        self.connected, self.armed_mode = False, None
        if self.api is not None:
            try:
                self._call("disconnect")
            except Exception as e:
                self.ledger.add("note", note=f"disconnect raised {type(e).__name__}: {e}")
        if self._undo is not None:
            self._undo()
            self._undo = None

    # -- reads -----------------------------------------------------------------------------------
    def _get(self, name: str) -> Any:
        code, value = self._call(name)
        if code != 0:
            raise LinkError(f"{name} failed (code {code})")
        return value

    def read(self) -> ArmState:
        t0 = self.clock()
        pose = self._get("get_position")
        state = self._get("get_state")
        err, warn = self._get("get_err_warn_code")
        cmdnum = self._get("get_cmdnum")
        t1 = self.clock()
        return ArmState(tuple(pose[:6]), int(state), int(err), int(warn), int(cmdnum), t1, round((t1 - t0) / 1e6, 2))

    def read_aa(self) -> tuple:
        """Pose with the orientation as an axis-angle vector (x, y, z mm; rx, ry, rz rad), from the controller."""
        return tuple(self._get("get_position_aa")[:6])

    def rpy_convention_error_rad(self) -> float:
        """Angle between the orientation built from the reported roll/pitch/yaw with OUR convention
        (R = Rz*Ry*Rx) and the controller's own axis-angle readout of the same pose. Read-only."""
        rpy, aa = self.read().pose[3:6], self.read_aa()[3:6]
        return q_angle(rpy_to_q(*rpy), q_from_rotvec(aa))

    def reported_mode(self) -> int | None:
        """Mode from the rich report (the controller has no mode query). None if no report arrived yet."""
        return int(self.core.mode) if getattr(self.core, "_first_report_over", False) else None

    def is_simulation(self) -> bool | None:
        """Simulation flag from the rich report. In simulation mode the controller reports a moving pose
        while the real arm stays still. None if no report arrived yet."""
        return bool(self.core.is_simulation_robot) if getattr(self.core, "_first_report_over", False) else None

    def reduced_states(self) -> Any:
        return self._get("get_reduced_states")

    # -- commands (each one explicit, none retried) ------------------------------------------------
    def arm_motion(self, mode: int) -> ArmState:
        """Enable, select the mode, set state 0. Refuses if the controller has an error or warning."""
        if mode not in (MODE_POSITION, MODE_ONLINE):
            raise LinkError("only mode 0 (position) or 7 (cartesian online planning) can be selected")
        s = self.read()
        if s.error or s.warn:
            raise LinkError(f"controller reports error {s.error} / warning {s.warn}; not arming (errors are never cleared here)")
        for name, arg in (("motion_enable", True), ("set_mode", mode), ("set_state", 0)):
            code = self._call(name, arg)
            if code != 0:
                self.armed_mode = None
                raise LinkError(f"{name}({arg}) failed (code {code})")
            if name == "set_mode":
                time.sleep(0.1)
        t_end = time.monotonic() + 1.0
        while True:  # the state may take a moment to leave "stopped" after set_state(0)
            s = self.read()
            if s.error or s.warn:
                raise LinkError(f"after arming: error {s.error}, warning {s.warn}")
            if s.state not in (4, 5):
                break
            if time.monotonic() > t_end:
                raise LinkError(f"after arming: controller still in state {s.state}")
            time.sleep(0.05)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and self.reported_mode() != mode:
            time.sleep(0.05)
        if self.reported_mode() is not None and self.reported_mode() != mode:
            raise LinkError(f"controller reports mode {self.reported_mode()}, expected {mode}")
        self.armed_mode = mode
        return s

    def move_line(self, pose: tuple, speed_mm_s: float, acc_mm_s2: float) -> None:
        """One absolute line target (x, y, z mm; roll, pitch, yaw rad). Returns when the controller has
        ACCEPTED it, not when the motion is finished. Raises if it was refused."""
        if self.armed_mode is None:
            raise LinkError("not armed")
        tx = [float(v) for v in pose[:6]] + [float(speed_mm_s), float(acc_mm_s2), 0.0, -1.0]
        self.ledger.add("api_call", name="arm_cmd.set_nfp32_with_bytes(MOVE_LINE)")
        ret = self.core.arm_cmd.set_nfp32_with_bytes(21, tx, 10, MOVE_LINE_BYTES, 3, timeout=self.io_timeout_s)
        code = ret[0]
        if code in (0, 2) and self.core.arm_cmd.state_is_ready:
            return
        self.armed_mode = None
        why = {8: "refused locally by the wire guard (outside the hard box, rotated too far or over the limits) or not sent",
               1: "controller has an error", 3: "no reply in time", 9: "controller not ready"}.get(code, "")  # fmt: skip
        if code in (0, 2):
            why = "controller not ready to move (state 4/5 or motors not enabled)"
        raise LinkError(f"move refused (code {code}) {why}".strip())

    def halt(self) -> ArmState:
        """SDK-level stop: set_state(4). Always attempted, also when not armed."""
        self.armed_mode = None
        code = self._call("set_state", 4)
        if code != 0:
            raise LinkError(f"set_state(4) failed (code {code})")
        return self.read()

    def park(self) -> ArmState:
        """Leave the controller as the lab's own code leaves it: mode 0, state 4."""
        self.armed_mode = None
        errors = []
        for name, arg in (("set_state", 4), ("set_mode", 0), ("set_state", 4)):
            try:
                code = self._call(name, arg)
                if code != 0:
                    errors.append(f"{name}({arg}) code {code}")
            except Exception as e:
                errors.append(f"{name}({arg}) {type(e).__name__}: {e}")
        if errors:
            raise LinkError("park incomplete: " + "; ".join(errors))
        return self.read()


def load_sdk() -> tuple:
    return ro.load_installed_sdk()
