"""Backend interface of the teleop adapter, and the SYNTHETIC backend used by the dry run.

The adapter talks to exactly four operations:

  read_feedback()            fresh tool pose, in the backend's own Cartesian frame
  command_target(p, ori)     REPLACE the single outstanding target (never a queue)
  stop(reason)               cancel outstanding motion; called on every disengagement
  preview_gripper(proposal)  record a proposed gripper command; never executes it

Internal units are SI: positions in metres. Orientation is carried as the backend's own native
tuple and is opaque to the adapter: it is captured at engagement and passed back verbatim, so
no Euler convention is assumed anywhere in the core. Each backend converts units exactly once,
at this boundary.

FakeRobotBackend is the only backend the dry run can construct. Everything it reports is
invented: its frame, start pose, workspace and gripper state are NOT lab data, and a recorded
stop request proves nothing about physical stopping.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

DRY_RUN_BANNER = "DRY RUN: NO ROBOT OR GRIPPER CONNECTED"
Vec3 = tuple  # (x, y, z)


class BackendError(RuntimeError):
    """Feedback, command or stop failed. The adapter latches a fault; nothing is retried."""


class LiveIntegrationBlocked(BackendError):
    """A hardware behaviour the source inventory cannot establish. Not guessed, not worked around."""


@dataclass(frozen=True)
class ToolFeedback:
    p_m: Vec3  # metres, backend Cartesian frame
    orientation: tuple  # backend-native, opaque; passed back verbatim
    orientation_repr: str
    frame: str
    read_mono_ns: int  # this host's monotonic clock when the read completed (not a robot timestamp)
    synthetic: bool
    source: str
    q_xyzw: tuple | None = None  # tool rotation in the backend frame as a unit quaternion; None = not provided


@dataclass(frozen=True)
class GripperFeedback:
    pct_closed: float | None  # 0 = open, 100 = closed; None = unknown
    connected: bool
    read_mono_ns: int
    synthetic: bool


@dataclass(frozen=True)
class GripperProposal:
    pct_closed: float  # value for POST /api/do/grip {"value": pct_closed}
    ticks_nominal: int  # 3033 - pct/100*1852 (rig formula; the driver's calibration decides)
    trigger: float


class RobotBackend:
    name = "abstract"
    synthetic = False
    frame = "unspecified"

    def read_feedback(self) -> ToolFeedback:
        raise NotImplementedError

    def command_target(self, p_m: Vec3, orientation: tuple) -> None:
        raise NotImplementedError

    def stop(self, reason: str, latched: bool = False) -> None:
        """Called on EVERY disengagement. latched=False: ordinary release or input loss (no further
        targets will come). latched=True: a fault; the backend must do its strongest software stop and
        must not accept targets again until it is explicitly re-armed."""
        raise NotImplementedError

    def motion_ready(self) -> tuple:
        """(ready, reason). The adapter refuses to engage while this is False."""
        return True, ""

    def orientation_from_q(self, q_xyzw: tuple) -> tuple:
        """The backend-native orientation for a rotation given as a quaternion (only used with wrist rotation)."""
        raise BackendError("this backend cannot take a rotation target")

    def read_gripper(self) -> GripperFeedback | None:
        return None

    def preview_gripper(self, proposal: GripperProposal) -> None:
        raise NotImplementedError


class FakeRobotBackend(RobotBackend):
    """SYNTHETIC. No sockets, no serial port, no SDK. Tracks the commanded target perfectly."""

    name = "FakeRobotBackend"
    synthetic = True
    frame = "SYNTHETIC_FRAME (invented for the dry run; not the lab, not the xArm base)"
    START_P_M: Vec3 = (0.0, 0.0, 0.0)  # invented
    START_ORIENTATION = (10.0, 20.0, 30.0)  # invented, opaque
    ORIENTATION_REPR = "synthetic_opaque_triplet"
    KEEP = 50_000

    def __init__(self, clock: Callable[[], int] = time.monotonic_ns, start_p_m: Vec3 = START_P_M,
                 gripper_pct_closed: float | None = 0.0, rotation: bool = False):  # fmt: skip
        self.clock = clock
        self._q = (0.0, 0.0, 0.0, 1.0) if rotation else None  # synthetic tool rotation; None = rotation not offered
        self._lock = threading.Lock()
        self._p = tuple(float(v) for v in start_p_m)
        self._orientation = self.START_ORIENTATION
        self._gripper_pct = gripper_pct_closed  # never changes: gripper execution does not exist here
        self.commands: deque = deque(maxlen=self.KEEP)  # (t_ns, p_m, orientation)
        self.stops: deque = deque(maxlen=self.KEEP)  # (t_ns, reason)
        self.gripper_previews: deque = deque(maxlen=self.KEEP)  # (t_ns, GripperProposal)
        self.command_count = self.stop_count = self.preview_count = self.feedback_count = self.latched_stop_count = 0
        # fault injection, used by tests only
        self.fail_feedback: str | None = None
        self.fail_command: str | None = None
        self.fail_stop: str | None = None
        self.frozen_read_ns: int | None = None  # report this (old) read time
        self.tracking = True
        self.command_gate: threading.Event | None = None  # command_target blocks until set

    def read_feedback(self) -> ToolFeedback:
        with self._lock:
            self.feedback_count += 1
            if self.fail_feedback:
                raise BackendError(f"synthetic feedback failure: {self.fail_feedback}")
            t = self.clock() if self.frozen_read_ns is None else self.frozen_read_ns
            return ToolFeedback(self._p, self._orientation, self.ORIENTATION_REPR, self.frame, t, True, self.name, self._q)

    def orientation_from_q(self, q_xyzw: tuple) -> tuple:
        if self._q is None:
            raise BackendError("synthetic backend created without rotation")
        return ("q",) + tuple(q_xyzw)

    def command_target(self, p_m: Vec3, orientation: tuple) -> None:
        with self._lock:
            self.command_count += 1
            self.commands.append((self.clock(), tuple(p_m), tuple(orientation)))
            fail, gate = self.fail_command, self.command_gate
        if gate is not None:
            gate.wait()  # simulates a blocking backend call
        if fail:
            raise BackendError(f"synthetic command failure: {fail}")
        if self.tracking:
            with self._lock:
                self._p, self._orientation = tuple(p_m), tuple(orientation)
                if orientation and orientation[0] == "q":
                    self._q = tuple(orientation[1:])

    def stop(self, reason: str, latched: bool = False) -> None:
        with self._lock:
            self.stop_count += 1
            self.latched_stop_count += int(latched)
            self.stops.append((self.clock(), reason))
            if self.fail_stop:
                raise BackendError(f"synthetic stop failure: {self.fail_stop}")

    def read_gripper(self) -> GripperFeedback | None:
        return GripperFeedback(self._gripper_pct, True, self.clock(), True)

    def preview_gripper(self, proposal: GripperProposal) -> None:
        with self._lock:
            self.preview_count += 1
            self.gripper_previews.append((self.clock(), proposal))

    def describe(self) -> dict:
        return {"backend": self.name, "synthetic": True, "frame": self.frame, "start_p_m": list(self.START_P_M),
                "start_orientation": list(self.START_ORIENTATION), "orientation_repr": self.ORIENTATION_REPR,
                "tracking_model": "target adopted instantly (invented)", "gripper_execution": "does not exist",
                "note": "all poses and feedback are synthetic; stop requests are only recorded"}  # fmt: skip
