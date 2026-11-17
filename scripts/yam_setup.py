"""Bimanual YAM: where the two arms stand, and the limits every target passes. Pure data; no hardware here.

World frame: origin on the table top midway between the two arm bases; x forward (the way both arms face,
away from the operator), y to the operator's left, z up. Metres and radians throughout the YAM code.

Placement (2026-11-17, from the operator): both bases stand on the table, both facing away from the operator, who
stands behind them; about 0.35 m apart centre to centre (the operator's ESTIMATE: measure it). Assumed: the arms
are the standard YAM (I2RT's default). The gripper, LINEAR_4310, was confirmed by the operator on
2026-11-17. The operator's left hand drives the left arm (can_left), the right hand the right arm (can_right).
"""

from __future__ import annotations

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


BASE_SPACING = 0.35  # m, centre to centre (estimate, 2026-11-17)
# Each box reaches 0.10 m past the middle into the other arm's side; the clearance check keeps the arms apart there.
LEFT = ArmSetup("left", "can_left", (0.0, BASE_SPACING / 2, 0.0), 0.0, (0.10, 0.55, -0.10, BASE_SPACING / 2 + 0.45, 0.03, 0.55))
RIGHT = ArmSetup("right", "can_right", (0.0, -BASE_SPACING / 2, 0.0), 0.0, (0.10, 0.55, -BASE_SPACING / 2 - 0.45, 0.10, 0.03, 0.55))
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
