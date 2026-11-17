#!/usr/bin/env python3
"""Bimanual YAM: both hands from the Quest stream, the clutch per hand, the hand-to-tool mapping, the gripper pickup.
Runs anywhere (the arms are the stand-in from test_yam_control)."""

from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import quest_pose_v1 as qp  # noqa: E402
from teleop_input import ACTIVE, ARMED, WAIT_RELEASE, InputSample, InputView  # noqa: E402
from teleop_rotation import mat_vec, q_angle, q_from_rotvec, q_mul  # noqa: E402
from test_controller_wait import SEEN  # noqa: E402  a real pose record: right controller seen, left inactive
from test_yam_control import FakePair  # noqa: E402
from yam_control import BimanualControl  # noqa: E402
from yam_input import hand_reasons  # noqa: E402
from yam_setup import ARMS, Limits  # noqa: E402
from yam_teleop import HandMap, YamTeleop, world_from_local  # noqa: E402

LIM = Limits()
UP = (0.0, 0.0, 0.0, 1.0)
IDENT = ("run", 1, "build", "sid", 1, 0, 0)


class Hand:
    """One controller as the stream would deliver it."""

    def __init__(self, p=(0.0, 1.0, -0.3), q=UP):
        self.p, self.q, self.grip, self.trigger, self.ok, self.seq = p, q, 0.0, 0.0, True, 0

    def view(self, now_ns):
        self.seq += 1
        s = InputSample(now_ns, self.seq, now_ns, IDENT, 1, self.ok, () if self.ok else ("pos_untracked",), self.p, self.q,
                        self.grip, self.trigger)  # fmt: skip
        return InputView(True, True, False, 1, s, (), 0)


class Rig:
    def __init__(self, yaw=0.0, scale=0.5):
        self.pair = FakePair()
        self.control = BimanualControl(self.pair, ARMS, LIM)
        self.t = YamTeleop(self.control, scale, yaw, 400)
        self.hands = {"left": Hand(), "right": Hand()}
        self.now = 0

    def step(self, n=1):
        for _ in range(n):
            self.now += 20_000_000
            r = self.t.tick({h: x.view(self.now) for h, x in self.hands.items()}, self.now)
        return r

    def engage(self, h):
        self.hands[h].grip = 0.0
        self.step()
        self.hands[h].grip = 1.0
        return self.step()


class FrameTests(unittest.TestCase):
    def test_operator_axes(self):
        R = world_from_local(0)
        for local, world in (((0, 0, -1), (1, 0, 0)), ((-1, 0, 0), (0, 1, 0)), ((0, 1, 0), (0, 0, 1))):
            self.assertEqual(tuple(round(v, 9) for v in mat_vec(R, local)), world)
        self.assertEqual(tuple(round(v, 9) for v in mat_vec(world_from_local(180), (0, 0, -1))), (-1, 0, 0))

    def test_hand_map(self):
        m = HandMap(0.5, world_from_local(0))
        m.anchor((0, 1, 0), UP, ((0.3, 0.3, 0.3), UP))
        p, q = m.target((0, 1, -0.2), q_from_rotvec((0, math.radians(30), 0)))  # 20 cm forward, turned 30 deg left
        self.assertEqual(tuple(round(v, 9) for v in p), (0.4, 0.3, 0.3))
        self.assertAlmostEqual(q_angle(q, q_from_rotvec((0, 0, math.radians(30)))), 0.0, places=9)  # about world up


class HandReasonTests(unittest.TestCase):
    def test_each_hand_judged_on_its_own_controller(self):
        rec = qp.decode_record(json.dumps(SEEN))
        self.assertEqual(hand_reasons(rec, "right"), [])
        self.assertIn("pose_action_inactive", hand_reasons(rec, "left"))


class ClutchTests(unittest.TestCase):
    def test_release_then_press_engages_and_follows(self):
        r = Rig()
        self.assertEqual(r.step()["left"].clutch, ARMED)  # released grip seen
        rep = r.engage("left")
        self.assertEqual(rep["left"].clutch, ACTIVE)
        x0 = r.pair.cmd["left"][0]
        r.hands["left"].p = (0.0, 1.0, -0.36)  # 6 cm forward
        r.step(40)
        self.assertAlmostEqual(r.pair.cmd["left"][0] - x0, 0.03, places=6)  # scale 0.5
        self.assertEqual(r.pair.cmd["right"][0], 0.3)  # the other arm did not move

    def test_grip_held_at_start_never_engages(self):
        r = Rig()
        r.hands["left"].grip = 1.0
        self.assertEqual(r.step(5)["left"].clutch, WAIT_RELEASE)

    def test_releasing_stops_the_arm(self):
        r = Rig()
        r.engage("left")
        r.hands["left"].grip = 0.3
        rep = r.step()
        self.assertNotEqual(rep["left"].clutch, ACTIVE)
        x = r.pair.cmd["left"][0]
        r.hands["left"].p = (0.0, 1.0, -0.5)
        r.step(10)
        self.assertEqual(r.pair.cmd["left"][0], x)

    def test_one_hand_lost_the_other_continues(self):
        r = Rig()
        r.engage("left"), r.engage("right")
        r.hands["left"].ok = False
        r.hands["right"].p = (0.0, 1.0, -0.32)
        rep = r.step(10)
        self.assertNotEqual(rep["left"].clutch, ACTIVE)
        self.assertEqual(rep["right"].clutch, ACTIVE)
        self.assertAlmostEqual(r.pair.cmd["right"][0], 0.31, places=6)

    def test_a_jump_needs_release_and_press_again(self):
        r = Rig()
        r.engage("left")
        r.hands["left"].p = (0.0, 1.0, -0.6)  # 30 cm in one step (15 cm at the tool)
        rep = r.step()
        self.assertNotEqual(rep["left"].clutch, ACTIVE)
        self.assertTrue(any("jumped" in n for n in rep["left"].notes))
        self.assertNotEqual(r.step(3)["left"].clutch, ACTIVE)  # grip still held: no re-engagement
        self.assertEqual(r.engage("left")["left"].clutch, ACTIVE)


class GripperTests(unittest.TestCase):
    def test_pickup_then_follow(self):
        r = Rig()  # the gripper starts open (1.0)
        r.hands["left"].trigger = 1.0  # would close
        self.assertEqual(r.step(5)["left"].gripper, "waiting for the trigger to match")
        self.assertEqual(r.pair.cmd["left"][6], 1.0)
        r.hands["left"].trigger = 0.05  # matches open
        self.assertEqual(r.step()["left"].gripper, "following")
        r.hands["left"].trigger = 1.0
        r.step(40)
        self.assertEqual(r.pair.cmd["left"][6], 0.0)

    def test_losing_the_hand_drops_the_pickup(self):
        r = Rig()
        r.hands["left"].trigger = 0.0
        r.step()
        r.hands["left"].ok = False
        r.step()
        r.hands["left"].ok, r.hands["left"].trigger = True, 1.0
        self.assertEqual(r.step()["left"].gripper, "waiting for the trigger to match")


if __name__ == "__main__":
    unittest.main()
