"""REAL arm backend for the teleop adapter. THIS CAN MOVE THE ARM. Not imported by the dry run.

Targets go to the controller as absolute line moves through XArmCommandLink (mode 7, "cartesian
online planning"), one per scheduler tick. The adapter already limits how fast its target moves, so
each new target is only a small step ahead of the arm; the speed and acceleration given to the
controller are a second, firmware-side cap.

  read_feedback()   pose from the controller (mm -> m here, the one conversion); raises if the
                    controller reports an error or warning (never cleared)
  motion_ready()    False until arm_motion() was called by the operator-facing program
  command_target()  one MOVE_LINE; the orientation captured at engagement is passed back unchanged
  stop(latched)     latched=False (clutch release, input loss): nothing more is sent; the arm ends
                    at its last target, which the adapter keeps within its tracking-error limit of
                    the arm. latched=True (a fault): set_state(4), and the backend is disarmed
                    until arm_motion() is called again
  close()           mode 0, state 4, as the lab's own code leaves the arm

Whether mode 7 really replaces targets, and what set_state(4) does physically, are assumptions
until the supervised first-movement test has measured them.
"""

from __future__ import annotations

from typing import Any

import xarm_command_link as cl
from teleop_backend import BackendError, GripperProposal, RobotBackend, ToolFeedback, Vec3
from teleop_rotation import q_to_rpy, rpy_to_q


class XArmLiveBackend(RobotBackend):
    name = "XArmLiveBackend (REAL arm, mode 7 line targets)"
    synthetic = False
    banner = "ARM LIVE (REAL ARM MOVES)"
    frame = "xArm base frame as reported by the controller; TCP offset zero (flange)"
    ORIENTATION_REPR = "xarm_roll_pitch_yaw_rad_as_reported"

    def __init__(self, link: cl.XArmCommandLink, speed_mm_s: float, acc_mm_s2: float):
        self.link, self.speed, self.acc = link, float(speed_mm_s), float(acc_mm_s2)
        self.command_count = self.stop_count = self.halt_count = 0
        self.gripper: Any = None  # optional GripctlClient (separate device); None = gripper not used
        self.last_state: cl.ArmState | None = None

    def arm_motion(self) -> None:
        """Explicit operator step: enable, mode 7, state 0. Refused if the controller has an error."""
        try:
            self.link.arm_motion(cl.MODE_ONLINE)
        except cl.LinkError as e:
            raise BackendError(str(e)) from None

    def motion_ready(self) -> tuple:
        if self.link.armed_mode != cl.MODE_ONLINE:
            return False, "arm not armed (operator must arm it)"
        return True, ""

    def read_feedback(self) -> ToolFeedback:
        try:
            s = self.link.read()
        except cl.LinkError as e:
            raise BackendError(str(e)) from None
        self.last_state = s
        if s.error or s.warn:
            raise BackendError(f"controller error {s.error} / warning {s.warn} (never cleared automatically)")
        return ToolFeedback((s.pose[0] / 1000.0, s.pose[1] / 1000.0, s.pose[2] / 1000.0), tuple(s.pose[3:6]),
                            self.ORIENTATION_REPR, self.frame, s.t_mono_ns, False, "xarm get_position",
                            rpy_to_q(s.pose[3], s.pose[4], s.pose[5]))  # fmt: skip

    def orientation_from_q(self, q_xyzw: tuple) -> tuple:
        return q_to_rpy(q_xyzw)  # roll, pitch, yaw in rad; the convention is checked against the controller at session start

    def command_target(self, p_m: Vec3, orientation: tuple) -> None:
        pose = (p_m[0] * 1000.0, p_m[1] * 1000.0, p_m[2] * 1000.0) + tuple(orientation)
        try:
            self.link.move_line(pose, self.speed, self.acc)
        except cl.LinkError as e:
            raise BackendError(str(e)) from None
        self.command_count += 1

    def stop(self, reason: str, latched: bool = False) -> None:
        self.stop_count += 1
        if not latched:
            return  # no further target is sent; see the module docstring
        self.halt_count += 1
        try:
            self.link.halt()
        except cl.LinkError as e:
            raise BackendError(f"halt failed: {e}") from None

    def read_gripper(self) -> Any:
        return None if self.gripper is None else self.gripper.read()

    def preview_gripper(self, proposal: GripperProposal) -> None:
        return None

    def close(self) -> None:
        try:
            self.link.park()
        except cl.LinkError as e:
            raise BackendError(f"park failed: {e}") from None

    def describe(self) -> dict:
        return {"backend": self.name, "synthetic": False, "frame": self.frame, "orientation_repr": self.ORIENTATION_REPR,
                "controller_speed_cap_mm_s": self.speed, "controller_acc_cap_mm_s2": self.acc,
                "hard_box_mm": [list(self.link.box_min), list(self.link.box_max)], "version": self.link.version}  # fmt: skip
