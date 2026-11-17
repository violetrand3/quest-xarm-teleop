"""Bimanual YAM: both Quest controllers from the protocol-v2 stream, one clutch per hand. Quest side only.

The left controller drives the left arm, the right controller the right arm. Each hand gets exactly the rules
the xArm's right hand has (teleop_input.Engagement): a valid release, then a valid press, in one unbroken
identity and continuity generation; any input problem disengages; reference-space changes latch until
acknowledged. A hand's input is usable only when the record says so for THAT hand (the same test as the
xArm's right hand, quest_pose_transport.right_input_reasons, applied to its own controller).

BimanualChannel extends the xArm's InputChannel without changing it: it keeps everything the channel keeps
(connection, continuity, notices, rejects) and adds a sample per hand from the same accepted packet.
"""

from __future__ import annotations

import dataclasses
from typing import Any

import quest_pose_transport as pt
import quest_pose_v1 as qp
from teleop_input import InputChannel, InputSample, InputView

HANDS = {"left": ("grip_l", "in_l"), "right": ("grip_r", "in_r")}
GRIP_RELEASED_MAX, GRIP_PRESSED_MIN, GRIP_HOLD_MIN = 0.2, 0.8, 0.5  # side grip (middle finger), as on the xArm


def hand_reasons(rec: qp.PoseRecord, hand: str) -> list[str]:
    """Every reason this hand's input is unusable in this record ([] = usable)."""
    pose_key, in_key = HANDS[hand]
    g = rec.poses[pose_key]
    out: list[str] = []
    if not rec.focused or rec.session_state != "FOCUSED":
        out.append("unfocused")
    if not g.queried or g.result < 0:
        out.append("locate_failed")
    if g.active is not True:
        out.append("pose_action_inactive")
    if not g.pos_valid or g.p is None:
        out.append("pos_invalid")
    if not g.ori_valid or g.q is None:
        out.append("ori_invalid")
    if g.pos_valid and not g.pos_tracked:
        out.append("pos_untracked")
    if g.ori_valid and not g.ori_tracked:
        out.append("ori_untracked")
    if rec.inputs[in_key]["squeeze"] is None:
        out.append("grip_action_inactive")
    return out


def hand_sample(rec: qp.PoseRecord, hand: str, arrival_ns: int, source_seq: int, identity: tuple, continuity_gen: int) -> InputSample:
    pose_key, in_key = HANDS[hand]
    g, i = rec.poses[pose_key], rec.inputs[in_key]
    reasons = hand_reasons(rec, hand)
    return InputSample(arrival_ns, source_seq, rec.t_query_ns, identity, continuity_gen, not reasons, tuple(reasons),
                       None if g.p is None else tuple(g.p), None if g.q is None else tuple(g.q), i.get("squeeze"),
                       i.get("trigger"), i.get("a" if hand == "right" else "x"), i.get("b" if hand == "right" else "y"))  # fmt: skip


class BimanualChannel(InputChannel):
    def __init__(self) -> None:
        super().__init__()
        self._hands: dict[str, InputSample] = {}

    def on_packet(self, pkt: dict[str, Any], accepted: bool, now_ns: int) -> None:
        super().on_packet(pkt, accepted, now_ns)
        if accepted and pkt.get("type") == "pose":
            v = pt.validate_pose(pkt)
            hands = {h: hand_sample(v.record, h, now_ns, v.source_seq, v.identity(), v.continuity_gen) for h in HANDS}
            with self._lock:
                self._hands = hands

    def hand_views(self, consume: bool = True) -> dict[str, InputView]:
        """One view per hand; connection, continuity, notices and rejects are shared (and consumed once)."""
        v = self.view(consume)
        with self._lock:
            hands = dict(self._hands)
        # The channel drops its sample whenever the input stops being current (close, reject, source down or
        # stale): both hands follow it.
        return {h: dataclasses.replace(v, sample=None if v.sample is None else hands.get(h)) for h in HANDS}
