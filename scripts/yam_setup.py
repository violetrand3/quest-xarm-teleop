"""Bimanual YAM: where the two arms stand, and the limits every target passes. Pure data; no hardware here.

World frame: origin on the table top midway between the two arm bases; x forward (the way both arms face,
away from the operator), y to the operator's left, z up. Metres and radians throughout the YAM code.

ASSUMED until measured on the rig (2026-11-17): the bases stand on the table 0.60 m apart, both facing +x;
the arms are the standard YAM with the LINEAR_4310 gripper (I2RT's defaults). The operator's left hand
drives the left arm (can_left), the right hand the right arm (can_right).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ArmSetup:
    name: str  # "left" / "right"
    channel: str  # its CAN link on the lab host (not opened in simulation)
    base_xyz: tuple[float, float, float]  # base origin in the world frame, m
    base_yaw: float  # turn of the base about z, rad (0 = facing +x)
    box: tuple[float, float, float, float, float, float]  # grasp point box, world frame: x_min,x_max,y_min,y_max,z_min,z_max


@dataclass(frozen=True)
class Limits:
    dt: float = 0.02  # control period, s (50 Hz)
    tool_speed: float = 0.15  # m/s: how fast a target may move
    tool_rot_speed: float = math.radians(45)  # rad/s
    jump: float = 0.08  # m: the hand target moving this far in one step is a glitch, not a movement
    jump_rot: float = math.radians(40)
    lag: float = 0.04  # m: the target never runs further than this ahead of the arm
    lag_rot: float = math.radians(20)
    joint_speed: float = 1.5  # rad/s per joint, after the IK
    joint_flip: float = 0.35  # rad in one step: the IK switched to another arm posture -> refused
    joint_margin: float = 0.05  # rad kept from each joint's end stop
    reach_tol: float = 0.005  # m: the IK answer this far from the target = out of reach (reported, followed partly)
    reach_tol_rot: float = math.radians(3)
    clearance: float = 0.01  # m kept between the two arms, and between an arm and the table
    grip_speed: float = 2.0  # gripper opening (0..1) per second
    track_err: float = 0.15  # rad: a joint measured this far from its command stops both arms (real arms; 0 in sim)


LEFT = ArmSetup("left", "can_left", (0.0, 0.30, 0.0), 0.0, (0.10, 0.55, -0.15, 0.65, 0.03, 0.55))
RIGHT = ArmSetup("right", "can_right", (0.0, -0.30, 0.0), 0.0, (0.10, 0.55, -0.65, 0.15, 0.03, 0.55))
ARMS = (LEFT, RIGHT)
