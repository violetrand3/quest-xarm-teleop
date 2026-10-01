"""PREPARED, NOT CONNECTED: adapters for the existing xArm and custom-gripper interfaces.

This module never imports the xArm SDK, a Dynamixel SDK or an HTTP client, and it has no entry
point. The arm object and the gripper HTTP callables must be INJECTED by the caller; the tests
inject fakes. Nothing in the dry run constructs these classes.

Only what the source inventory (reports/robot_backend_source_inventory_20261001.md) states
unambiguously is implemented:

  feedback   arm.get_position() -> (code, [x, y, z, roll, pitch, yaw]) in mm / degrees
             (is_radian=False; stepper.py:78, 93), arm.get_err_warn_code() -> (code, [err, warn])
  stop       arm.emergency_stop() then arm.set_state(4), as Robot.stop() (stepper.py:215-220)
  target     conversion of an internal target to set_position keyword arguments
             (x, y, z in mm; roll, pitch, yaw passed back exactly as reported)
  gripper    GET /api/state -> values.position (ticks), status.connected (stepper.py:55-61)

Everything the inventory cannot establish raises LiveIntegrationBlocked and stays blocked until
it has been verified on the rig under supervision. Nothing here enables motion, changes mode,
clears errors, homes, resets or reconnects.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from teleop_backend import (BackendError, GripperFeedback, GripperProposal, LiveIntegrationBlocked, RobotBackend,
                            ToolFeedback, Vec3)  # fmt: skip
from teleop_mapping import GRIPPER_OPEN_TICKS, GRIPPER_SPAN_TICKS

COMMAND_BLOCKERS = (
    "arm command path is blocked: (1) the only motion call in the existing code is a BLOCKING "
    "set_position(..., wait=True) in mode 0 (stepper.py:192); whether a non-blocking set_position queues or "
    "replaces targets is not established; (2) motion_enable/set_mode/set_state(0) must be a supervised, "
    "explicit operator step and is never issued by this adapter; (3) what emergency_stop() + set_state(4) "
    "do to motion in progress is unverified; (4) the controller TCP offset, reduced mode and fence are "
    "unverified; (5) exclusive command ownership versus stepper.py is not arranged"
)
GRIPPER_BLOCKERS = (
    "gripper execution is blocked: POST /api/do/grip has no stall cancel and enables torque (gripctl.py:1566-1610); "
    "holding behaviour, effort caps, calibration validity and the 1-3 s USB dropouts must be checked on the rig first"
)


class XArmPreparedBackend(RobotBackend):
    name = "XArmPreparedBackend (injected arm object; command path blocked)"
    synthetic = False
    frame = "xArm base frame as reported by get_position(); TCP offset on the controller UNVERIFIED"
    ORIENTATION_REPR = "xarm_roll_pitch_yaw_deg_as_reported"

    def __init__(self, arm: Any, *, is_radian: bool, clock: Callable[[], int] = time.monotonic_ns):
        if is_radian is not False:
            raise LiveIntegrationBlocked("only the documented is_radian=False (mm / degrees) interface is prepared")
        self._arm, self.clock = arm, clock

    def read_feedback(self) -> ToolFeedback:
        code, pose = self._arm.get_position()
        if code != 0 or not isinstance(pose, (list, tuple)) or len(pose) < 6:
            raise BackendError(f"get_position failed (code {code})")
        ecode, ew = self._arm.get_err_warn_code()
        if ecode != 0 or list(ew) != [0, 0]:
            raise BackendError(f"arm reports error/warning {list(ew)} (code {ecode}); it is never cleared automatically")
        p_m = (pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0)  # the one mm -> m conversion
        return ToolFeedback(p_m, (pose[3], pose[4], pose[5]), self.ORIENTATION_REPR, self.frame, self.clock(), False, "xarm.get_position")

    @staticmethod
    def native_target(p_m: Vec3, orientation: tuple) -> dict:
        """Keyword arguments for set_position: the one m -> mm conversion; orientation passed back verbatim."""
        if len(orientation) != 3:
            raise BackendError("orientation must be the (roll, pitch, yaw) triple captured from feedback")
        return {"x": p_m[0] * 1000.0, "y": p_m[1] * 1000.0, "z": p_m[2] * 1000.0,
                "roll": orientation[0], "pitch": orientation[1], "yaw": orientation[2]}  # fmt: skip

    def command_target(self, p_m: Vec3, orientation: tuple) -> None:
        self.native_target(p_m, orientation)  # validated, then refused: no motion call is made
        raise LiveIntegrationBlocked(COMMAND_BLOCKERS)

    def stop(self, reason: str) -> None:
        """The rig's own stop sequence. Its physical effect is unverified; it is not a hardware E-stop."""
        errors = []
        for call in (lambda: self._arm.emergency_stop(), lambda: self._arm.set_state(4)):
            try:
                call()
            except Exception as e:  # attempt both calls regardless
                errors.append(f"{type(e).__name__}: {e}")
        if errors:
            raise BackendError("stop sequence raised: " + "; ".join(errors))

    def preview_gripper(self, proposal: GripperProposal) -> None:
        return None  # previews are recorded by the adapter; nothing is sent


class GripctlPreparedGripper:
    """Custom gripper (gripctl HTTP service). Read-only state via an injected getter; commands blocked."""

    def __init__(self, http_get_json: Callable[[str], dict], clock: Callable[[], int] = time.monotonic_ns):
        self._get, self.clock = http_get_json, clock

    def read(self) -> GripperFeedback:
        try:
            d = self._get("/api/state")
            ticks = (d.get("values") or {}).get("position")
            connected = bool((d.get("status") or {}).get("connected"))
        except Exception as e:
            raise BackendError(f"gripper state read failed: {type(e).__name__}: {e}") from None
        pct = None if not connected or not isinstance(ticks, (int, float)) else (GRIPPER_OPEN_TICKS - ticks) / GRIPPER_SPAN_TICKS * 100.0
        return GripperFeedback(pct, connected, self.clock(), False)

    @staticmethod
    def native_command(proposal: GripperProposal) -> tuple:
        return "/api/do/grip", {"value": proposal.pct_closed}  # percent closed, 0 = open

    def command(self, proposal: GripperProposal) -> None:
        raise LiveIntegrationBlocked(GRIPPER_BLOCKERS)
