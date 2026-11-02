"""Tests for the arm geometry (arm_kinematics: the xArm6 geometry), the wrist guard (teleop_limits.WristGuard) and its use in the adapter.

REAL DATA: the (joint angles, tool pose) pairs in RECORDED were read from the lab's arm at the start of sessions
(summary files, 2026-10-31 to 2026-11-02); STUCK is the pose where the guard of 2026-10-31 held the arm for nine grip
presses on 2026-11-02. Everything else is synthetic: poses made from joint angles with the same geometry, and joint
angles that follow the fake backend's tool by that geometry. Nothing here shows how the real arm moves.
"""
from __future__ import annotations

import dataclasses
import json
import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import teleop_adapter as ta  # noqa: E402
import arm_kinematics as kin  # noqa: E402
from teleop_limits import WristGuard, _q_turn  # noqa: E402
from teleop_rotation import q_angle, rpy_to_q  # noqa: E402
from test_teleop_adapter import CONFIG, P0, Rig, off  # noqa: E402

# joints [deg], pose [mm, mm, mm, roll, pitch, yaw rad] as the controller reported them
RECORDED = [([50.17, -12.65, -23.53, -88.09, 33.16, 93.92], [199.2, 259.8, 226.941, 2.632, -0.666, 0.761]),
            ([9.59, -62.87, 4.63, -77.85, 11.6, 81.84], [189.188, 88.195, 281.879, 3.047, -0.984, 0.042]),
            ([24.54, 4.42, -43.72, -7.36, 24.64, 90.2], [406.748, 191.152, 256.53, -2.889, -0.079, -1.045]),
            ([-13.3, -56.28, 5.59, -67.17, 22.48, 56.88], [207.42, -16.23, 247.51, 2.663, -0.664, -0.05]),
            ([10.04, -50.36, -6.6, -126.99, 16.14, 131.07], [212.699, 77.93, 281.925, 3.033, -1.175, 0.051]),
            ([8.81, -30.9, -23.7, -74.32, 10.05, 76.5], [307.004, 106.358, 353.602, 3.045, -0.913, 0.064])]  # fmt: skip
STUCK = ([52.29, -38.97, -26.07, 0.54, 15.0, 12.62], [199.413, 259.448, 443.296, -2.876, -0.843, 0.568])


def pose_fb(pose: list) -> tuple:
    return (pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0), rpy_to_q(*pose[3:])


def tool_axis(q: tuple) -> tuple:
    r = kin.q_to_matrix(q)
    return (r[0][2], r[1][2], r[2][2])


def tilted(q: tuple, deg: float) -> tuple:
    """q tilted by deg toward straight down (negative: away from it)."""
    z = tool_axis(q)
    n = math.hypot(z[0], z[1])
    a = math.radians(deg)
    return _q_turn(q, (-z[1] / n * a, z[0] / n * a, 0.0))


def moved(p: tuple, d_mm: tuple) -> tuple:
    return tuple(p[k] + d_mm[k] / 1000.0 for k in range(3))


def matrix_q(r: list) -> tuple:
    t = r[0][0] + r[1][1] + r[2][2]
    s = math.sqrt(t + 1.0) * 2.0  # all the poses used here are far from a half turn
    return ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, s / 4.0)


class KinematicsTests(unittest.TestCase):
    def test_the_geometry_matches_the_arm(self) -> None:
        for j, pose in RECORDED + [STUCK]:
            p, r, axes = kin.fk(j)
            self.assertLess(math.dist([v * 1000.0 for v in p], pose[:3]), 4.0)
            self.assertLess(math.degrees(q_angle(matrix_q(r), rpy_to_q(*pose[3:]))), 1.0)
            z4, z6 = axes[3][1], tool_axis(rpy_to_q(*pose[3:]))  # joint 5 is the angle between the axes of joints 4 and 6
            self.assertAlmostEqual(math.degrees(math.acos(sum(a * b for a, b in zip(z4, z6)))), abs(j[4]), delta=0.1)

    def test_predicted_joints_for_moves_of_a_few_degrees(self) -> None:
        rnd = random.Random(7)
        for j, _ in RECORDED:
            for _ in range(20):
                j1 = [a + rnd.uniform(-3.0, 3.0) for a in j]
                p0, r0, _ = kin.fk(j)
                p1, r1, _ = kin.fk(j1)
                pred = kin.predict(j, [p1[k] - p0[k] for k in range(3)], matrix_q(r0), matrix_q(r1))
                self.assertAlmostEqual(abs(pred[4]), abs(j1[4]), delta=0.1)


class WristGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.g = WristGuard(15, 45, 15)

    def at(self, j: list, pose: list) -> tuple:
        fb = pose_fb(pose)
        self.g.engage(fb)
        return j, fb

    def test_away_from_the_zone_everything_is_sent(self) -> None:
        j, fb = self.at(*RECORDED[2])  # joint 5 at 25 deg
        for d in ((10, 0, 0), (0, -20, 5), (-5, 5, -5)):
            t = (moved(fb[0], d), tilted(fb[1], -5))
            self.assertEqual(self.g.check(j, fb, *t), (None, t))
        self.assertEqual(self.g.note, "")

    def test_at_the_stuck_pose_of_2026_11_02_helpful_moves_are_followed(self) -> None:
        j, fb = self.at(STUCK[0][:4] + [14.0] + STUCK[0][5:], STUCK[1])  # just inside the zone
        back = (fb[0], tilted(fb[1], 3))  # toward straight down
        self.assertEqual(self.g.check(j, fb, *back), (None, back))
        self.assertTrue(self.g.note.startswith("following: joint 5 at 14deg"), self.g.note)
        self.g.engage(fb)
        aside = (moved(fb[0], (0, -15, 0)), fb[1])  # what the operator tried that day (85 mm in -y): refused then
        self.assertEqual(self.g.check(j, fb, *aside), (None, aside))
        self.g.engage(fb)
        self.assertEqual(self.g.check(j, fb, fb[0], tilted(fb[1], -3)), (None, None))  # further from straight down
        self.assertEqual(self.g.note, "held: joint 5 at 14deg, near the wrist's turning point; try: tilt the gripper back toward straight down")

    def test_the_helpful_part_of_a_move_is_followed(self) -> None:
        j, fb = self.at(STUCK[0][:4] + [14.0] + STUCK[0][5:], STUCK[1])
        p, q = moved(fb[0], (0, -10, 0)), tilted(fb[1], -4)  # the move aside helps, the tilt away from down does not
        self.assertEqual(self.g.check(j, fb, p, q), (None, (p, fb[1])))
        self.assertTrue(self.g.note.startswith("following part of the move"), self.g.note)
        self.assertEqual(self.g.last, (p, fb[1]))

    def test_the_arm_is_held_at_the_edge_of_the_zone_not_inside(self) -> None:
        j, fb = self.at(STUCK[0][:4] + [16.0] + STUCK[0][5:], STUCK[1])  # just outside
        self.assertEqual(self.g.check(j, fb, fb[0], tilted(fb[1], -5)), (None, None))  # would take joint 5 to about 9
        self.assertTrue(self.g.note.startswith("held: joint 5 at 16deg"), self.g.note)

    def test_near_the_limit_of_joint_6_the_hint_is_a_turn_that_helps(self) -> None:
        j = [20.0, -40.0, -20.0, 10.0, 40.0, 330.0]  # synthetic: joint 6 30 deg from its limit
        p, r, _ = kin.fk(j)
        fb = (p, matrix_q(r))
        self.g.engage(fb)
        z = tool_axis(fb[1])
        worse = _q_turn(fb[1], tuple(v * math.radians(5) for v in z))  # turning about its own axis turns joint 6
        better = _q_turn(fb[1], tuple(-v * math.radians(5) for v in z))
        self.assertGreater(kin.predict(j, (0, 0, 0), fb[1], worse)[5], 334.0)
        self.assertEqual(self.g.check(j, fb, fb[0], worse), (None, None))
        self.assertIn("joint 6 at 330deg, near its limit; try: turn the gripper", self.g.note)
        self.g.engage(fb)
        self.assertEqual(self.g.check(j, fb, fb[0], better), (None, (fb[0], better)))

    def test_joints_4_and_6_stop_before_their_limit(self) -> None:
        j, fb = self.at(*RECORDED[0])
        why, send = self.g.check(j[:5] + [-346.0], fb, *fb)
        self.assertEqual((why, send), ("wrist_limit:joint 6 at -346.0deg, within 15deg of its limit", None))
        self.assertEqual(self.g.check(None, fb, *fb), ("wrist_guard:no joint angles from the backend", None))


def wrist_cfg(**w):  # type: ignore[no-untyped-def]
    d = json.loads(CONFIG.read_text())
    d["wrist"] = dict({"j5_zone_deg": 15, "j46_zone_deg": 45, "j46_stop_deg": 15}, **w)
    return ta.parse_config(d)


class AdapterWristTests(unittest.TestCase):
    def rig(self, j0: list) -> Rig:
        """Joint angles that follow the fake tool by the arm's geometry (the tool keeps its orientation: rotation off)."""
        r = Rig(wrist_cfg())
        read = r.backend.read_feedback
        state = {"p": None, "j": list(j0)}
        self.j5 = []

        def feedback():  # type: ignore[no-untyped-def]
            f = read()
            if state["p"] is not None:
                state["j"] = kin.predict(state["j"], [f.p_m[k] - state["p"][k] for k in range(3)], steps=3)
            state["p"] = f.p_m
            self.j5.append(state["j"][4])
            return dataclasses.replace(f, joints_deg=tuple(state["j"]))

        r.backend.read_feedback = feedback
        return r

    def test_config(self) -> None:
        self.assertEqual(wrist_cfg().wrist, (15.0, 45.0, 15.0))
        for bad in ({"j5_zone_deg": 0}, {"j46_stop_deg": 50}, {"extra": 1}):
            with self.assertRaises(ta.ConfigError):
                wrist_cfg(**bad)

    def test_held_at_the_zone_then_any_helpful_move_is_followed(self) -> None:
        # at the stuck pose's arm shape, moving the tool down takes joint 5 toward 0 and moving it in -y away from 0
        r = self.rig(STUCK[0][:4] + [18.0] + STUCK[0][5:])
        r.engage()
        out = {}
        for i in range(1, 101):  # hand down (LOCAL -y is the backend's -z): 0.2 * 300 mm = 60 mm asked for
            out = r.step(off(P0, (0.0, -0.003 * i, 0.0)), 1.0)
        for _ in range(100):
            out = r.step(off(P0, (0.0, -0.3, 0.0)), 1.0)
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertTrue(out["wrist"].startswith("held: joint 5 at 15deg"), out["wrist"])
        self.assertGreater(min(self.j5), 14.8)  # stopped at the edge of the zone
        z_held = out["fb_mm"][2]
        self.assertLess(z_held, -10.0)  # it did go down until then
        for i in range(1, 101):  # then the hand goes sideways (LOCAL +x is the backend's -y), not back up
            out = r.step(off(P0, (0.003 * i, -0.3, 0.0)), 1.0)
        for _ in range(200):
            out = r.step(off(P0, (0.3, -0.3, 0.0)), 1.0)
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertLess(out["fb_mm"][1], -59.0)  # followed sideways all the way
        self.assertLess(out["fb_mm"][2], z_held + 0.5)  # without having to go back up first
        self.assertGreater(min(self.j5), 14.8)  # and never further into the zone
        self.assertEqual(out["joints"][4], round(self.j5[-1], 2))  # joint angles are in the tick log
        self.assertEqual(r.rec.faults, [])

    def test_joint_4_near_its_limit_is_a_fault_before_the_controllers(self) -> None:
        r = Rig(wrist_cfg())
        read = r.backend.read_feedback
        r.backend.read_feedback = lambda: (lambda f: dataclasses.replace(f, joints_deg=(0.0, 0.0, 0.0, 346.0, 40.0, 0.0)))(read())
        r.engage()
        out = r.step(off(P0, (0.0, 0.0, 0.003)), 1.0)
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("wrist_limit:joint 4 at 346"), r.rec.faults[-1])

    def test_no_joint_angles_is_a_fault(self) -> None:
        r = Rig(wrist_cfg())
        r.engage()
        out = r.step(off(P0, (0.0, 0.0, 0.003)), 1.0)
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.rec.faults[-1]["reason"], "wrist_guard:no joint angles from the backend")


if __name__ == "__main__":
    unittest.main()
