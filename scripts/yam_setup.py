"""Bimanual YAM: where the two arms stand, and the limits every target passes. Pure data; no hardware here.

World frame: origin on the table top midway between the two arm bases; x forward (the way both arms face,
away from the operator), y to the operator's left, z up. Metres and radians throughout the YAM code.

Placement (2026-11-17, from the operator): both bases stand on the table, both facing away from the operator, who
stands behind them; 0.50 m apart centre to centre (measured by the operator 2026-11-17). Assumed: the arms
are the standard YAM (I2RT's default). The gripper, LINEAR_4310, was confirmed by the operator on
2026-11-17. The operator's left hand drives the left arm (can_left), the right hand the right arm (can_right).
"""

from __future__ import annotations

import dataclasses
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


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
    tool_accel: float = 1.0  # m/s^2: the target speeds up and slows down this gently (full speed in 0.15 s)
    tool_rot_accel: float = math.radians(300)  # rad/s^2
    jump: float = 0.08  # m: the hand target moving this far in one step is a glitch, not a movement
    jump_rot: float = math.radians(40)
    lag: float = 0.04  # m: the target never runs further than this ahead of the arm
    lag_rot: float = math.radians(20)
    joint_speed: float = 1.5  # rad/s per joint, after the IK
    joint_flip: float = 0.35  # rad in one step: the IK switched to another arm posture -> refused
    joint_margin: float = 0.05  # rad kept from each joint's end stop
    wrist_give: float = 0.05  # orientation weight (position = 1) when the full pose is out of the wrist's reach: the
    # tool still goes where the hand goes and only its angle stops short (the wrist sat on an end stop 26% of the time
    # at speed level 15, 2026-11-18, holding the whole arm back)
    reach_tol: float = 0.005  # m: the IK answer this far from the target = out of reach (reported, followed partly)
    reach_tol_rot: float = math.radians(3)
    clearance: float = 0.01  # m kept between the two arms, and between an arm and the table
    tip_gap: float = 0.01  # m kept between one arm's fingertips and the other's, at every speed level: two hands on one
    # object (the arms otherwise keep `clearance`, 2-3 cm at the faster levels; 5% of the time at level 30, 2026-11-19)
    grip_speed: float = 2.0  # gripper opening (0..1) per second
    track_err: float = 0.10  # rad: a joint measured this far from its command stops both arms (real arms; 0 in sim) ...
    track_lag_s: float = 0.25  # ... plus this many seconds of the joint's fastest commanded speed in the last 0.5 s: a
    # moving real joint trails its command. Measured 2026-11-17 (run yam_20261117-180314, 86 s, half speed): still
    # within 0.044 rad, lag 0.04-0.12 s per joint, at worst 51% of this allowance (right joint 5). Was 0.15 + 0.3 s.


# Speed levels, named by the tool's top speed in cm/s: (tool m/s, tool turn deg/s, joint rad/s, gripper opening/s,
# clearance m). The accelerations follow from the speeds: full speed in 0.15 s. 7.5 is what the real arms ran at from 2026-11-17 (half of 15, the simulation's); the faster ones
# keep a wider gap between the arms, because a real arm trails its command (0.04-0.12 s measured at 7.5, i.e. up to
# 5 cm at 50 cm/s) and the clearance is checked on the commands. A human hand reaching for something moves at
# roughly 0.5-1 m/s; the operator's hand reached 0.21 m/s (95th percentile) and 0.43 m/s (99th) while engaged, held
# back by the 7.5 cm/s limit 37% of the time (5 demos, 2026-11-17). Each level must be tried in the simulation and
# then on the arms, one level at a time, before the next.
RAMP_S = 0.15  # s from standstill to full speed (and back): smooth starts and stops
SPEEDS = {7.5: (0.075, 22.5, 0.75, 1.0, 0.01), 15: (0.15, 45.0, 1.5, 2.0, 0.01),
          30: (0.30, 90.0, 2.5, 3.0, 0.02), 50: (0.50, 150.0, 3.5, 4.0, 0.03)}  # fmt: skip


def limits_at(speed_cm_s: float, base: Limits | None = None) -> Limits:
    """The limits for one of the SPEEDS levels; every other limit stays as in Limits."""
    if speed_cm_s not in SPEEDS:
        raise ValueError(f"speed {speed_cm_s:g} cm/s is not a level; the levels are {', '.join(f'{s:g}' for s in SPEEDS)}")
    v, w, j, g, c = SPEEDS[speed_cm_s]
    return dataclasses.replace(base or Limits(), tool_speed=v, tool_rot_speed=math.radians(w), joint_speed=j,
                               grip_speed=g, clearance=c, tool_accel=v / RAMP_S, tool_rot_accel=math.radians(w) / RAMP_S)  # fmt: skip


BASE_SPACING = 0.50  # m, centre to centre (measured by the operator 2026-11-17; was the estimate 0.35)
# Each box reaches 0.10 m past the middle into the other arm's side; the clearance check keeps the arms apart there.
# The box floor is the table top (z 0): the table rule is the floor, fingertips may touch the table but not press into
# it, every other part keeps the clearance above it. Was 3 cm, which kept the fingertips ~3 cm up, too high to grasp a
# bag lying flat (2026-11-18, the operator).
BOX_FLOOR = 0.0
# The box edge nearest the operator, m in front of the bases (x). Was 0.10; the left arm sat on it 11% of the time at
# level 30 (2026-11-19). The clearance model keeps an arm off its own base and links.
BOX_NEAR = 0.03
LEFT = ArmSetup("left", "can_left", (0.0, BASE_SPACING / 2, 0.0), 0.0, (BOX_NEAR, 0.55, -0.10, BASE_SPACING / 2 + 0.45, BOX_FLOOR, 0.55))
RIGHT = ArmSetup("right", "can_right", (0.0, -BASE_SPACING / 2, 0.0), 0.0, (BOX_NEAR, 0.55, -BASE_SPACING / 2 - 0.45, 0.10, BOX_FLOOR, 0.55))
ARMS = (LEFT, RIGHT)


# --- the bring-up's measurements of the real arms (see yam_real.py) ---------------------------------------------
class HardwareFileError(ValueError):
    pass


def load_hardware(path: Path, names: Sequence[str], ranges: Sequence[Sequence[float]] | None = None) -> dict[str, Any]:
    """The bring-up's measurements, checked: every arm, finite numbers, rest poses inside the joint ranges."""
    try:
        hw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise HardwareFileError(f"{path} does not exist: the bring-up (gripper limits, rest poses) has not been recorded") from None
    except (OSError, ValueError) as e:
        raise HardwareFileError(f"{path}: {e}") from None
    if not isinstance(hw, dict) or not str(hw.get("measured", "")).strip():
        raise HardwareFileError(f"{path}: needs a 'measured' note (who, when, how)")
    arms = hw.get("arms")
    if not isinstance(arms, dict):
        raise HardwareFileError(f"{path}: needs 'arms'")
    out = {}
    for n in names:
        a = arms.get(n)
        if not isinstance(a, dict):
            raise HardwareFileError(f"{path}: no entry for the {n} arm")
        g, r = a.get("gripper_limits"), a.get("rest_q")
        if not (isinstance(g, list) and len(g) == 2 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in g) and g[0] != g[1]):
            raise HardwareFileError(f"{path}: {n} gripper_limits must be [closed, open], two different numbers")
        if not (isinstance(r, list) and len(r) == 6 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in r)):
            raise HardwareFileError(f"{path}: {n} rest_q must be {6} joint angles")
        if ranges is not None and any(not lo <= v <= hi for v, (lo, hi) in zip(r, ranges)):
            raise HardwareFileError(f"{path}: {n} rest_q is outside the joint ranges")
        out[n] = {"gripper_limits": [float(v) for v in g], "rest_q": [float(v) for v in r]}
    return out
