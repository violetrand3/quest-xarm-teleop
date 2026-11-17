"""Bimanual YAM episode recording: the xArm teleop's EpisodeRecorder (folders, A/B buttons, cameras, disk and
length limits, save/discard) with a row for two arms. NO HARDWARE ACCESS: it is handed one record per control step.

Operator controls (right controller, thumb), as on the xArm: A starts a demo and, pressed again, saves it;
B discards the one in progress. The demo closes itself after max_episode_s, or when the session ends.

The record yam_live.py hands it every step (k = "tick"):
    {"k": "tick", "t": ns, "dt_ms": ..., "btn_a": bool|None, "btn_b": bool|None,
     "arms": {"left": {...}, "right": {...}}}          one entry per arm, the fields of ARM_FIELDS
and events (k = "transition" with "arm", "old", "new"; k = "fault" with "reason").
"""

from __future__ import annotations

import math
from typing import Any

from episode_recorder import EpisodeRecorder

FORMAT = "quest-yam-bimanual-episode-v1"
ARM_FIELDS = {
    "clutch": "this hand's clutch state; ACTIVE = the arm is following the hand",
    "arm": "the arm's state in the control step (idle, following, held, let go, fault)",
    "q_cmd": "joint command sent this step: joints 1-6 (rad) and the gripper opening (0 = closed, 1 = open)",
    "q_meas": "measured joints 1-6 (rad) and gripper opening, read at the start of this step (simulation: the command read back)",
    "tcp_p": "tool (grasp point) position from the measured joints, metres, world frame (yam_setup.py: origin on the table midway between the bases, x away from the operator, y left, z up)",
    "tcp_q": "tool orientation from the measured joints, quaternion x, y, z, w, world frame",
    "target_p": "the tool target this step after the box, speed and lag limits, metres, world frame; null when not following",
    "target_q": "its orientation, quaternion x, y, z, w",
    "grip_cmd": "gripper opening commanded this step (0 = closed, 1 = open)",
    "trig": "index trigger 0..1 (closes the gripper while the side grip is held)",
    "squeeze": "side grip 0..1 (held = arm follows)",
    "hand_p": "this controller's position in the headset's LOCAL space, metres",
    "hand_q": "its orientation, quaternion x, y, z, w, LOCAL space",
    "input_age_ms": "age of the newest controller sample at this step",
    "notes": "what limited or refused the arm this step (box edge, speed limit, too close, end stop, ...)",
}
STEP_FIELDS = {"t": "host monotonic clock, ns, at the start of the control step; the same clock as the camera frame times",
               "dt_ms": "time since the previous step",
               "left": "the left arm (left controller): " + "; ".join(f"{k}: {v}" for k, v in ARM_FIELDS.items()),
               "right": "the right arm (right controller), the same fields"}  # fmt: skip


class YamEpisodeRecorder(EpisodeRecorder):
    FORMAT, STEP_FIELDS = FORMAT, STEP_FIELDS

    @staticmethod
    def _row(rec: dict, extra: dict | None) -> dict:
        arms = rec.get("arms") or {}
        return {"t": rec["t"], "dt_ms": rec.get("dt_ms"), **{n: {k: a.get(k) for k in ARM_FIELDS} for n, a in arms.items()}}

    @staticmethod
    def _count(ep: dict, row: dict, rec: dict) -> None:
        """meta.json's statistics for two arms: following time while either arm follows, tool travel of both
        arms together (mm), the gripper range of both (percent closed, as the xArm's)."""
        arms = {n: v for n, v in row.items() if isinstance(v, dict)}
        if any(a.get("clutch") == "ACTIVE" for a in arms.values()):
            ep["active_ns"] += int((rec.get("dt_ms") or 0.0) * 1e6)
        last: dict[str, Any] = ep["last_p"] or {}
        for n, a in arms.items():
            p = a.get("tcp_p")
            if p is None:
                ep["rows_without_pose"] += 1
                continue
            if n in last:
                ep["travel_mm"] += math.dist(p, last[n]) * 1000.0
            last[n] = p
            q = a.get("q_meas")
            if q is not None and len(q) > 6 and q[6] is not None:
                closed = round((1.0 - q[6]) * 100.0, 2)
                ep["grip_min"], ep["grip_max"] = min(ep["grip_min"], closed), max(ep["grip_max"], closed)
        ep["last_p"] = last
