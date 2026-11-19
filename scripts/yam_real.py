"""Bimanual YAM: the two REAL arms behind the interface yam_control drives. OPENING THEM ENABLES THE MOTORS.

Runs on the lab host in I2RT's environment. Used only by yam_live.py --arms real, which needs the operator's typed
confirmation; nothing else imports it.

What I2RT does with real arms, and what this layer does about it (read from i2rt 234d895, 2026-11-17):
  - get_yam_robot() enables the motors. With zero_gravity_mode=False it starts by holding every joint where it
    is (PD on the measured position), and refuses to start when a joint reads outside its limits (a sign of a
    wrong zero position). This layer always starts that way: nothing moves at start-up.
  - The LINEAR_4310 gripper is calibrated at start-up by DRIVING it to its ends, unless its limits are given.
    This layer gives them, from the hardware file (below), and refuses to start without them.
  - I2RT's own control thread keeps sending the commands, so the motors' watchdog (TIMEOUT 8000 on every motor:
    without commands a motor goes limp) is fed while this program runs. If the program dies, the arms go limp.
  - close() sets every torque to zero: the arm goes limp. yam_live.py lowers both arms to their rest poses first.

The hardware file (HARDWARE_FILE on the lab host, written after the bring-up, outside the synced copy):
    {"measured": "<who, when, how>",
     "arms": {"left":  {"gripper_limits": [closed, open], "rest_q": [6 joint angles, rad]},
              "right": {"gripper_limits": [closed, open], "rest_q": [...]}}}
gripper_limits are motor positions in rad, as I2RT's calibration reports them; rest_q is where the arm rests
safely when it goes limp (confirmed by looking at the arm).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np
from i2rt.robots.get_robot import get_yam_robot

from yam_limits import joint_velocity
from yam_setup import ArmSetup, HardwareFileError, Limits, load_hardware
from yam_sim import N_ARM, SimYamPair

HARDWARE_FILE = Path("/home/dev/yam/yam_hardware.json")
GRIP_START_TOL = 0.05  # how far outside 0..1 a gripper may read at start-up (calibration noise)
GRIP_REST_MAX = 0.4  # lowering to rest closes a gripper to at most this opening, so the next start-up is valid


class GripperStartError(RuntimeError):
    pass


class RealYamPair(SimYamPair):
    sim = False

    def __init__(self, arms: Sequence[ArmSetup], lim: Limits, hardware_file: Path = HARDWARE_FILE,
                 velocity_ff: bool = True, **kw: Any):  # fmt: skip
        self.hw = load_hardware(hardware_file, [a.name for a in arms])
        self.robots = {}
        self.velocity_ff = velocity_ff
        self._last_cmd: dict[str, list[float]] = {}
        try:
            super().__init__(arms, lim, **kw)
        except BaseException:
            self.close()  # an arm already opened must not stay enabled when the other failed
            raise
        load_hardware(hardware_file, self.names, self.ranges)  # now that the joint ranges are known

    def _make_robot(self, a: ArmSetup):  # noqa: ANN202
        r = get_yam_robot(channel=a.channel, arm_type=self.arm_type, gripper_type=self.gripper_type, sim=False,
                          zero_gravity_mode=False,  # hold where it is: nothing moves at start-up
                          gripper_limits_override=np.array(self.hw[a.name]["gripper_limits"]),  # no calibration motion
                          enable_auto_recovery=False)  # fmt: skip
        self.robots[a.name] = r  # so close() reaches it even if the next arm fails
        # I2RT shifts a motor's zero by a full turn at start-up when it reads outside (-pi, pi] (wrap_around_offset).
        # The gripper's stroke is ~5 rad (closed ~0, open ~-5.07 on the right arm), so jaws more than ~62% open at
        # start-up come back 2*pi off and the measured limits no longer apply: the first gripper command would drive
        # the motor a full turn past an end stop. Nothing has been commanded yet: check the opening and refuse.
        g = float(r.get_joint_pos()[N_ARM])
        if not -GRIP_START_TOL <= g <= 1.0 + GRIP_START_TOL:
            raise GripperStartError(
                f"the {a.name} gripper reads {g:+.2f} of its stroke at start-up (0 = closed, 1 = open): it is probably more "
                "than about 60% open, so I2RT shifted its zero by a full turn and the measured limits do not apply. Nothing "
                "was commanded. Close that gripper below half open (or recalibrate it in the bring-up) and start again")
        return r

    def rest(self, name: str) -> list[float]:
        return list(self.hw[name]["rest_q"])

    def command(self, name: str, q7: Sequence[float]) -> None:
        """The position command, with the velocity that goes with it (yam_limits.joint_velocity): I2RT's motor control
        is kp * (position error) + kd * (velocity error), so with the velocity left at 0 (command_joint_pos) the kd term
        brakes every movement and the joint trails its command (up to 78% of the tracking allowance on the right wrist
        at level 30, 2026-11-19). The gains are not changed."""
        q = [float(v) for v in q7]
        vel = joint_velocity(self._last_cmd.get(name), q, self.lim) if self.velocity_ff else [0.0] * len(q)
        self._last_cmd[name] = q
        if not any(vel):
            self.robots[name].command_joint_pos(np.asarray(q, dtype=float))
            return
        _command_pos_vel(self.robots[name], np.asarray(q, dtype=float), np.asarray(vel, dtype=float))

    def place(self, name: str, q7: Sequence[float]) -> None:  # noqa: D102
        raise RuntimeError("place() is for the simulation only")

    def close(self) -> None:
        """Torque off (I2RT's close): the arms go limp. Lower them to rest first."""
        for r in list(self.robots.values()):
            try:
                r.close()
            except Exception as e:  # close the other arm whatever happens to this one
                print(f"closing an arm failed: {e}", flush=True)


def _command_pos_vel(robot: Any, pos: np.ndarray, vel: np.ndarray) -> None:
    """command_joint_pos (i2rt motor_chain_robot, read 2026-11-19) plus a velocity, all under the command lock.
    Not I2RT's command_joint_state: it replaces the commands with an all-zero set (kp = kd = 0, no position hold) BEFORE
    taking the lock, so the motor thread can pick that up for a cycle. Falls back to command_joint_pos (no velocity)
    when this I2RT version does not have the parts used here."""
    try:
        clip, lock = robot._clip_robot_joint_pos_command, robot._command_lock
        to_pos, to_vel, remap = robot._joint_to_motor_pos, robot._joint_to_motor_vel, robot.remapper
        kp, kd, n = robot._kp, robot._kd, len(robot.motor_chain)
        from i2rt.robots.motor_chain_robot import JointCommands
    except (AttributeError, ImportError):
        robot.command_joint_pos(pos)
        return
    pos = clip(pos)
    with lock:
        c = JointCommands.init_all_zero(n)
        c.pos = to_pos(remap.to_robot_joint_pos_space(pos))
        c.vel = to_vel(remap.to_robot_joint_vel_space(vel))
        c.kp, c.kd = kp, kd
        robot._commands = c
