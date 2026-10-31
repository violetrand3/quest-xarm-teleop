"""Tests for the in-session wrist guard (teleop_limits.WristGuard) and its use in the adapter.

ALL DATA HERE IS SYNTHETIC: the joint angles are invented functions of the tool position, standing in for
an arm whose wrist approaches its turning point. Nothing here shows how the real arm's joints move.
"""
from __future__ import annotations

import dataclasses
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import teleop_adapter as ta  # noqa: E402
from teleop_limits import WristGuard  # noqa: E402
from test_teleop_adapter import CONFIG, P0, Rig, off  # noqa: E402

FAR = (0.0, 0.0, 0.0, 0.5, 55.0, -6.0)  # joints well away from trouble


def joints(j4: float = 0.5, j5: float = 55.0, j6: float = -6.0) -> tuple:
    return (0.0, 0.0, 0.0, j4, j5, j6)


class WristGuardTests(unittest.TestCase):
    def test_outside_the_zone_everything_is_sent(self) -> None:
        g = WristGuard(15, 45, 15)
        for x in (0.30, 0.31, 0.29):
            self.assertEqual(g.check(FAR, ((x, 0.0, 0.3), None), (x - 0.001, 0.0, 0.3), None), (None, True))
        self.assertEqual(g.safe, ((0.29, 0.0, 0.3), None))  # where the arm was measured, not the target

    def test_in_the_zone_only_targets_leading_back_are_sent(self) -> None:
        g = WristGuard(15, 45, 15)
        g.check(joints(j5=20), ((0.29, 0.0, 0.3), None), (0.28, 0.0, 0.3), None)  # outside the zone (20 > 15): sent; safe = 0.29
        here = ((0.28, 0.0, 0.3), None)  # the arm reached 0.28 and is now inside the zone
        self.assertEqual(g.check(joints(j5=12), here, (0.27, 0.0, 0.3), None), (None, False))  # further in: held
        self.assertTrue(g.note.startswith("held: joint 5 at 12deg"), g.note)
        self.assertEqual(g.check(joints(j5=12), here, (0.28, 0.0, 0.3), None), (None, False))  # where it already is: nothing to send
        self.assertEqual(g.check(joints(j5=12), here, (0.285, 0.0, 0.3), None), (None, True))  # back toward 0.29
        self.assertTrue(g.note.startswith("leading back"), g.note)
        self.assertEqual(g.check(joints(j5=13), ((0.285, 0.0, 0.3), None), (0.28, 0.0, 0.3), None), (None, False))
        self.assertEqual(g.check(joints(j5=16), ((0.29, 0.0, 0.3), None), (0.27, 0.0, 0.3), None), (None, True))  # out: free again
        self.assertEqual(g.note, "")

    def test_a_turn_of_the_tool_counts_too(self) -> None:
        g = WristGuard(15, 45, 15)
        down, turned = (1.0, 0.0, 0.0, 0.0), (0.9961947, 0.0, 0.0871557, 0.0)  # straight down, and 10 deg turned about z
        p = (0.3, 0.0, 0.3)
        g.check(FAR, (p, turned), p, down)  # sent while outside; safe = turned
        self.assertEqual(g.check(joints(j5=10), (p, down), p, down), (None, False))
        self.assertEqual(g.check(joints(j5=10), (p, down), p, (0.9990482, 0.0, 0.0436194, 0.0)), (None, True))  # turning back

    def test_joints_4_and_6_stop_before_their_limit(self) -> None:
        g = WristGuard(15, 45, 15)
        here = ((0.3, 0.0, 0.3), None)
        self.assertEqual(g.check(joints(j4=340), here, here[0], None)[1], True)  # 20 deg left: in the zone, nothing to lead back to
        why, send = g.check(joints(j6=-346), here, here[0], None)
        self.assertEqual(send, False)
        self.assertEqual(why, "wrist_limit:joint 6 at -346.0deg, within 15deg of its limit")
        self.assertEqual(g.check(None, here, here[0], None), ("wrist_guard:no joint angles from the backend", False))


def wrist_cfg(**w):  # type: ignore[no-untyped-def]
    d = json.loads(CONFIG.read_text())
    d["wrist"] = dict({"j5_zone_deg": 15, "j46_zone_deg": 45, "j46_stop_deg": 15}, **w)
    return ta.parse_config(d)


class AdapterWristTests(unittest.TestCase):
    def rig(self, j_of_dx) -> Rig:  # type: ignore[no-untyped-def]
        r = Rig(wrist_cfg())
        read, x0 = r.backend.read_feedback, r.backend.read_feedback().p_m[0]
        r.backend.read_feedback = lambda: (lambda f: dataclasses.replace(f, joints_deg=j_of_dx((f.p_m[0] - x0) * 1000.0)))(read())
        return r

    def test_config(self) -> None:
        self.assertEqual(wrist_cfg().wrist, (15.0, 45.0, 15.0))
        for bad in ({"j5_zone_deg": 0}, {"j46_stop_deg": 50}, {"extra": 1}):
            with self.assertRaises(ta.ConfigError):
                wrist_cfg(**bad)

    def test_the_arm_is_held_at_the_turning_point_and_follows_back_out(self) -> None:
        # joint 5 falls 1 deg per mm as the tool moves toward the base (-x); the zone starts 15 mm in
        r = self.rig(lambda dx: joints(j5=30.0 + dx))
        r.engage()
        out = {}
        for i in range(1, 101):  # hand along LOCAL +z: the tool toward -x, 0.2 * 300 mm = 60 mm asked for
            out = r.step(off(P0, (0.0, 0.0, 0.003 * i)), 1.0)
        for _ in range(100):
            out = r.step(off(P0, (0.0, 0.0, 0.3)), 1.0)
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertAlmostEqual(out["fb_mm"][0] - r.backend.START_P_M[0] * 1000.0, -15.5, delta=1.5)  # held just inside the zone
        self.assertTrue(out["wrist"].startswith("held: joint 5"), out["wrist"])
        for i in range(100, -1, -1):  # the hand goes back
            out = r.step(off(P0, (0.0, 0.0, 0.003 * i)), 1.0)
        for _ in range(100):
            out = r.step(P0, 1.0)
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertAlmostEqual(out["fb_mm"][0], r.backend.START_P_M[0] * 1000.0, delta=0.5)  # followed all the way back
        self.assertEqual(out["wrist"], "")
        self.assertEqual(r.rec.faults, [])

    def test_joint_4_near_its_limit_is_a_fault_before_the_controllers(self) -> None:
        r = self.rig(lambda dx: joints(j4=330.0 - dx))  # joint 4 rises 1 deg per mm toward the base, from 330
        r.engage()
        out = {}
        for i in range(1, 101):
            out = r.step(off(P0, (0.0, 0.0, 0.003 * i)), 1.0)
            if out["state"] != ta.ACTIVE:
                break
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("wrist_limit:joint 4 at 345"), r.rec.faults[-1])

    def test_no_joint_angles_is_a_fault(self) -> None:
        r = Rig(wrist_cfg())
        r.engage()
        out = r.step(off(P0, (0.0, 0.0, 0.003)), 1.0)
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.rec.faults[-1]["reason"], "wrist_guard:no joint angles from the backend")


if __name__ == "__main__":
    unittest.main()
