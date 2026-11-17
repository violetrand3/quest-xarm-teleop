#!/usr/bin/env python3
"""Bimanual YAM: limits and the control step, against a stand-in for the arms (no simulator, no numpy)."""

from __future__ import annotations

import ast
import dataclasses
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from teleop_rotation import q_angle, q_from_rotvec, q_to_rotvec  # noqa: E402
from yam_control import BimanualControl, Request  # noqa: E402
from yam_limits import TargetLimiter, clamp_box, dist, joint_step, stretch_box  # noqa: E402
from yam_setup import ARMS, LEFT, Limits  # noqa: E402

LIM = Limits()
V = LIM.tool_speed * LIM.dt
UP = (0.0, 0.0, 0.0, 1.0)


def pose(x, y, z, q=UP):
    return ((x, y, z), q)


class FakePair:
    """Joints ARE the tool pose: q6 = (x, y, z, rotation vector). The IK is exact; parts are clear unless
    `problems` says otherwise; `offset` makes the measured joints differ from the commands."""

    names = ("left", "right")

    def __init__(self, start=None, problems=None):
        self.cmd = {n: list(start[n]) if start else [0.3, 0.3 if n == "left" else -0.3, 0.3, 0, 0, 0, 1.0] for n in self.names}
        self.problems = problems or (lambda q: {})
        self.offset = {n: [0.0] * 7 for n in self.names}
        self.commands = []

    def joint_ranges(self, n):
        return [(-5.0, 5.0)] * 6

    def joints(self, n):
        return [a + b for a, b in zip(self.cmd[n], self.offset[n])]

    def fk(self, n, q6):
        return (tuple(q6[:3]), q_from_rotvec(q6[3:6]))

    def solve(self, n, p, q6_init):
        return list(p[0]) + list(q_to_rotvec(p[1])), 0.0, 0.0

    def clearance_problems(self, q):
        return self.problems(q)

    def command(self, n, q7):
        self.cmd[n] = list(q7)
        self.commands.append((n, list(q7)))


def control(pair=None, arms=ARMS, lim=LIM):
    pair = pair or FakePair()
    c = BimanualControl(pair, arms, lim)
    return c, pair


class BoxTests(unittest.TestCase):
    def test_clamp_and_stretch(self):
        box = (0, 1, 0, 1, 0, 1)
        self.assertEqual(clamp_box((2, 0.5, -1), box), ((1, 0.5, 0), ["x max", "z min"]))
        self.assertEqual(stretch_box(box, (1.5, 0.5, -0.2)), (0, 1.5, 0, 1, -0.2, 1))


class TargetLimiterTests(unittest.TestCase):
    def setUp(self):
        self.t = TargetLimiter(LEFT.box, LIM)
        self.start = pose(0.3, 0.3, 0.3)
        self.t.reset(self.start)

    def test_speed_limit(self):
        s = self.t.step(pose(0.4, 0.3, 0.3), self.start)
        self.assertAlmostEqual(dist(s.target[0], self.start[0]), V)
        self.assertIn("speed limit", s.notes)

    def test_box(self):
        p = self.start
        for _ in range(400):
            s = self.t.step(pose(0.3, 0.3, 0.9), p)
            p = s.target
        self.assertAlmostEqual(p[0][2], LEFT.box[5])
        self.assertTrue(any("box edge (z max)" in n for n in s.notes))

    def test_jump_is_refused_and_forgotten(self):
        self.t.step(pose(0.301, 0.3, 0.3), self.start)
        s = self.t.step(pose(0.40, 0.3, 0.3), self.start)
        self.assertTrue(s.glitch)
        self.assertEqual(s.target, self.t.target)
        self.assertFalse(self.t.step(pose(0.40, 0.3, 0.3), self.start).glitch)  # after a refusal there is no previous request

    def test_never_more_than_lag_ahead_of_the_arm(self):
        for _ in range(100):  # the arm does not move at all
            s = self.t.step(pose(0.5, 0.3, 0.3), self.start)
        self.assertLessEqual(dist(s.target[0], self.start[0]), LIM.lag + V)
        self.assertIn("waiting for the arm", s.notes)

    def test_turn_speed(self):
        q = q_from_rotvec((0, 0, math.radians(90)))
        s = self.t.step(pose(0.3, 0.3, 0.3, q), self.start)
        self.assertAlmostEqual(q_angle(UP, s.target[1]), LIM.tool_rot_speed * LIM.dt, places=6)

    def test_start_outside_the_box_stretches_it(self):
        t = TargetLimiter(LEFT.box, LIM)
        notes = t.reset(pose(0.05, 0.3, 0.3))  # x min is 0.10
        self.assertTrue(notes)
        s = t.step(pose(0.0, 0.3, 0.3), pose(0.05, 0.3, 0.3))
        self.assertAlmostEqual(s.target[0][0], 0.05)  # may stay, not go further out
        s = t.step(pose(0.06, 0.3, 0.3), pose(0.05, 0.3, 0.3))
        self.assertGreater(s.target[0][0], 0.05)  # may come back in


class JointStepTests(unittest.TestCase):
    R = [(-1.0, 1.0)] * 6

    def test_end_stop_margin(self):
        s = joint_step([0.94] * 6, [0.99] + [0.94] * 5, self.R, LIM)
        self.assertAlmostEqual(s.q[0], 1.0 - LIM.joint_margin)
        self.assertIn("joint 1 at its end stop", s.notes)

    def test_parked_inside_the_margin_is_not_moved(self):
        s = joint_step([0.99] + [0.0] * 5, [0.995] + [0.0] * 5, self.R, LIM)
        self.assertAlmostEqual(s.q[0], 0.99)

    def test_posture_switch_refused(self):
        s = joint_step([0.0] * 6, [0.0, 0.5, 0, 0, 0, 0], self.R, LIM)
        self.assertIsNone(s.q)
        self.assertIn("joint 2", s.refused)

    def test_slowed_keeps_direction(self):
        s = joint_step([0.0] * 6, [0.1, 0.05, 0, 0, 0, 0], self.R, LIM)
        self.assertAlmostEqual(s.q[0], LIM.joint_speed * LIM.dt)
        self.assertAlmostEqual(s.q[1] / s.q[0], 0.5)


class ControlTests(unittest.TestCase):
    def test_idle_arms_are_commanded_where_they_are(self):
        c, p = control()
        before = {n: list(p.cmd[n]) for n in p.names}
        for _ in range(5):
            r = c.tick({"left": Request(pose(0.5, 0.3, 0.3))})
        self.assertEqual(r["left"].state, "idle")
        self.assertEqual(p.cmd, before)
        self.assertEqual(len(p.commands), 10)  # both arms every step

    def test_follows_after_engagement_and_stops_on_release(self):
        c, p = control()
        c.engage("left")
        for _ in range(10):
            r = c.tick({"left": Request(pose(0.33, 0.3, 0.3)), "right": Request(pose(0.3, -0.3, 0.3))})
        self.assertEqual(r["left"].state, "following")
        self.assertAlmostEqual(p.cmd["left"][0], 0.33)
        c.release("left")
        c.tick({"left": Request(pose(0.4, 0.3, 0.3))})
        self.assertAlmostEqual(p.cmd["left"][0], 0.33)

    def test_input_lost_lets_go(self):
        c, p = control()
        c.engage("left")
        r = c.tick({})
        self.assertEqual(r["left"].state, "let go")
        self.assertFalse(c.engaged["left"])

    def test_jump_lets_go(self):
        c, p = control()
        c.engage("left")
        c.tick({"left": Request(pose(0.301, 0.3, 0.3))})
        r = c.tick({"left": Request(pose(0.45, 0.3, 0.3))})
        self.assertEqual(r["left"].state, "let go")
        self.assertIn("jumped", r["left"].notes[0])

    def test_a_move_toward_a_problem_is_held_the_other_arm_still_moves(self):
        c, p = control(FakePair(problems=lambda q: {"left gripper and right gripper": 0.005} if q["left"][1] < 0.29 else {}))
        c.engage("left"), c.engage("right")
        for _ in range(20):
            r = c.tick({"left": Request(pose(0.3, 0.2, 0.3)), "right": Request(pose(0.3, -0.25, 0.3))})
        self.assertGreaterEqual(p.cmd["left"][1], 0.29)
        self.assertEqual(r["left"].state, "held")
        self.assertIn("too close", r["left"].notes[-1])
        self.assertAlmostEqual(p.cmd["right"][1], -0.25)

    def test_moving_away_from_an_existing_problem_is_allowed(self):
        near = lambda q: {"left link5 and the table": 0.002 + (q["left"][2] - 0.1)} if q["left"][2] < 0.108 else {}  # noqa: E731
        start = {"left": [0.3, 0.3, 0.1, 0, 0, 0, 1.0], "right": [0.3, -0.3, 0.3, 0, 0, 0, 1.0]}
        c, p = control(FakePair(start, near))
        c.engage("left")
        c.tick({"left": Request(pose(0.3, 0.3, 0.09))})  # closer: held
        self.assertAlmostEqual(p.cmd["left"][2], 0.1)
        for _ in range(5):
            c.tick({"left": Request(pose(0.3, 0.3, 0.103))})  # away: allowed
        self.assertAlmostEqual(p.cmd["left"][2], 0.103)

    def test_both_moves_together_held_when_only_the_pair_is_a_problem(self):
        both = lambda q: {"left tip and right tip": 0.004} if q["left"][1] < 0.3 and q["right"][1] > -0.3 else {}  # noqa: E731
        c, p = control(FakePair(problems=both))
        c.engage("left"), c.engage("right")
        r = c.tick({"left": Request(pose(0.3, 0.298, 0.3)), "right": Request(pose(0.3, -0.298, 0.3))})
        self.assertEqual((r["left"].state, r["right"].state), ("held", "held"))
        self.assertEqual((p.cmd["left"][1], p.cmd["right"][1]), (0.3, -0.3))

    def test_tracking_fault_stops_both_where_they_are(self):
        c, p = control()
        c.engage("left"), c.engage("right")
        p.offset["right"][2] = LIM.track_err + 0.01
        r = c.tick({"left": Request(pose(0.31, 0.3, 0.3)), "right": Request(pose(0.3, -0.3, 0.3))})
        self.assertEqual(r["left"].state, "fault")
        self.assertIn("right arm joint 3", c.fault)
        self.assertAlmostEqual(p.cmd["left"][0], 0.3)  # did not move
        self.assertAlmostEqual(p.cmd["right"][2], 0.3 + LIM.track_err + 0.01)  # stopped at the measured position
        self.assertEqual(c.engage("left"), [f"not engaged: {c.fault}"])

    def test_a_moving_joint_may_trail_further_than_a_still_one(self):
        c, p = control()
        c.engage("left")
        for k in range(10):  # left x moving at the speed limit
            c.tick({"left": Request(pose(0.3 + 0.003 * (k + 1), 0.3, 0.3)), "right": Request(pose(0.3, -0.3, 0.3))})
        p.offset["left"][0] = -(LIM.track_err + 0.02)  # trailing a little more than a still joint may
        c.tick({"left": Request(pose(0.333, 0.3, 0.3)), "right": Request(pose(0.3, -0.3, 0.3))})
        self.assertEqual(c.fault, "")
        p.offset["right"][0] = LIM.track_err + 0.02  # the right arm is still: the same error stops both
        c.tick({"left": Request(pose(0.333, 0.3, 0.3)), "right": Request(pose(0.3, -0.3, 0.3))})
        self.assertIn("right arm joint 1", c.fault)

    def test_enter_clears_a_fault_without_moving_and_a_hand_must_press_again(self):
        c, p = control()
        c.engage("left")
        p.offset["left"][2] = 0.3
        c.tick({"left": Request(pose(0.3, 0.3, 0.3))})
        self.assertTrue(c.fault)
        p.offset["left"][2] = 0.0  # the arm is where it was stopped (the stand-in reads command + offset)
        held = list(p.cmd["left"])
        self.assertTrue(c.clear_fault())
        self.assertEqual(c.fault, "")
        self.assertFalse(c.engaged["left"])
        c.tick({"left": Request(pose(0.35, 0.3, 0.3))})  # not engaged: nothing moves
        self.assertEqual(p.cmd["left"][:6], held[:6])

    def test_gripper_rate_limit(self):
        c, p = control()
        r = c.tick({"left": Request(None, grip=0.0)})
        self.assertAlmostEqual(r["left"].q[6], 1.0 - LIM.grip_speed * LIM.dt)


class SeparationTests(unittest.TestCase):
    """The YAM teleop and the xArm teleop share only the Quest side and pure math: neither imports the other's
    arm code. The YAM control modules need nothing beyond the standard library (they run on any PC)."""

    XARM = {"live_teleop", "teleop_adapter", "teleop_limits", "teleop_backend", "teleop_backend_xarm", "xarm_live_backend",
            "xarm_command_link", "xarm_readonly_preflight", "arm_kinematics", "xarm"}  # fmt: skip

    @staticmethod
    def imports(path):
        out = set()
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                out |= {a.name.split(".")[0] for a in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module:
                out.add(node.module.split(".")[0])
        return out

    def test_yam_does_not_import_xarm_code(self):
        for f in (ROOT / "scripts").glob("yam_*.py"):
            self.assertFalse(self.imports(f) & self.XARM, f.name)

    def test_xarm_code_does_not_import_yam(self):
        for f in (ROOT / "scripts").glob("*.py"):
            if not f.name.startswith("yam_"):
                self.assertFalse({m for m in self.imports(f) if m.startswith("yam_")}, f.name)

    def test_control_modules_need_only_the_standard_library(self):
        ours = {f.stem for f in (ROOT / "scripts").glob("*.py")}
        for name in ("yam_setup", "yam_limits", "yam_control"):
            outside = {m for m in self.imports(ROOT / "scripts" / f"{name}.py") - ours if m not in sys.stdlib_module_names}
            self.assertFalse(outside, name)

    def test_setup_boxes_are_inside_out_ordered(self):
        for a in ARMS:
            self.assertTrue(all(a.box[2 * i] < a.box[2 * i + 1] for i in range(3)), a.name)
        self.assertTrue(dataclasses.is_dataclass(LIM))


if __name__ == "__main__":
    unittest.main()
