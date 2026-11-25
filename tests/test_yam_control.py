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

    def test_speeds_up_gently_to_the_speed_limit(self):
        p, steps = self.start, []
        for _ in range(15):
            s = self.t.step(pose(0.6, 0.3, 0.3), p)
            steps.append(dist(s.target[0], p[0]))
            p = s.target
        self.assertAlmostEqual(steps[0], LIM.tool_accel * LIM.dt * LIM.dt)  # the first step is small
        self.assertTrue(all(b - a <= LIM.tool_accel * LIM.dt * LIM.dt + 1e-12 for a, b in zip(steps, steps[1:])))
        self.assertAlmostEqual(steps[-1], V)  # full speed after 0.15 s, never more
        self.assertLessEqual(max(steps), V + 1e-12)
        self.assertIn("speed limit", s.notes)

    def test_slows_down_and_stops_on_the_target_without_overshoot(self):
        p, goal, steps = self.start, 0.36, []
        for _ in range(80):
            s = self.t.step(pose(goal, 0.3, 0.3), p)
            steps.append(s.target[0][0] - p[0][0])
            p = s.target
        self.assertAlmostEqual(p[0][0], goal, places=9)
        self.assertLessEqual(max(x for x in [q[0] for q in [p[0]]]), goal + 1e-9)
        self.assertTrue(all(x >= -1e-12 for x in steps))  # never went past and back
        moving = [x for x in steps if x > 1e-9]
        self.assertLess(moving[-1], V / 2)  # it slowed down before stopping

    def test_reverses_smoothly(self):
        p = self.start
        for _ in range(15):  # the hand moves forward, 3 cm ahead of the target
            p = self.t.step(pose(p[0][0] + 0.03, 0.3, 0.3), p).target
        x0, v0 = p[0][0], self.t.vel[0]
        s = self.t.step(pose(x0 - 0.03, 0.3, 0.3), p)  # the hand turned back: the target slows, it does not jump back
        self.assertFalse(s.glitch)
        self.assertGreater(s.target[0][0], x0)
        self.assertAlmostEqual(v0 - self.t.vel[0], LIM.tool_accel * LIM.dt)

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
        p, turns = self.start, []
        for _ in range(15):  # the arm follows exactly, so the lag rule does not hold the turn
            s = self.t.step(pose(0.3, 0.3, 0.3, q), p)
            turns.append(q_angle(p[1], s.target[1]))
            p = s.target
        self.assertAlmostEqual(turns[0], LIM.tool_rot_accel * LIM.dt * LIM.dt, places=9)
        self.assertAlmostEqual(turns[-1], LIM.tool_rot_speed * LIM.dt, places=6)
        self.assertLessEqual(max(turns), LIM.tool_rot_speed * LIM.dt + 1e-9)

    def test_start_outside_the_box_stretches_it(self):
        t = TargetLimiter(LEFT.box, LIM)
        notes = t.reset(pose(0.0, 0.3, 0.3))  # x min is BOX_NEAR, 0.03
        self.assertTrue(notes)
        s = t.step(pose(-0.05, 0.3, 0.3), pose(0.0, 0.3, 0.3))
        self.assertAlmostEqual(s.target[0][0], 0.0)  # may stay, not go further out
        s = t.step(pose(0.01, 0.3, 0.3), pose(0.0, 0.3, 0.3))
        self.assertGreater(s.target[0][0], 0.0)  # may come back in


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


class VelocityTests(unittest.TestCase):
    def test_the_velocity_is_the_commanded_movement_per_step(self):
        from yam_limits import joint_velocity

        q0 = [0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.5]
        q1 = [0.01, 1.0, 0.99, 0.0, 0.0, 0.02, 0.4]
        v = joint_velocity(q0, q1, LIM)
        self.assertEqual([round(x, 9) for x in v], [0.5, 0.0, -0.5, 0.0, 0.0, 1.0, 0.0])  # gripper: 0

    def test_no_velocity_without_a_last_command_or_across_a_jump(self):
        from yam_limits import joint_velocity

        q = [0.0] * 7
        self.assertEqual(joint_velocity(None, q, LIM), [0.0] * 7)
        jump = [0.0, 0.0, 0.0, LIM.joint_speed * LIM.dt * 2, 0.0, 0.0, 0.0]  # faster than the limits allow
        self.assertEqual(joint_velocity(q, jump, LIM), [0.0] * 7)


class WristPair(FakePair):
    """The full pose cannot be reached: the full IK answer lands `off_full` m beside the target (trading position
    for angle, as the real IK does), the angle-giving-way answer `off_give` m beside it, `angle` rad short."""

    def __init__(self, off_full, off_give, angle=0.3):
        super().__init__()
        self.off_full, self.off_give, self.angle = off_full, off_give, angle

    def solve(self, n, p, q6_init, orientation_cost=1.0):
        off = self.off_full if orientation_cost >= 1.0 else self.off_give
        return [p[0][0], p[0][1] + off, p[0][2]] + list(q_to_rotvec(p[1])), off, self.angle


class WristTests(unittest.TestCase):
    def test_the_angle_gives_way_and_the_position_follows(self):
        c, p = control(WristPair(off_full=0.05, off_give=0.0))
        c.engage("left")
        for _ in range(30):
            r = c.tick({"left": Request(pose(0.32, 0.3, 0.3))})
        self.assertEqual(r["left"].state, "following")
        self.assertAlmostEqual(p.cmd["left"][1], 0.3)  # not pulled 5 cm aside
        self.assertTrue(any(n.startswith("wrist at its limit") for n in r["left"].notes))

    def test_a_step_that_takes_the_tool_away_from_the_hand_is_not_taken(self):
        c, p = control(WristPair(off_full=0.05, off_give=0.04))
        c.engage("left")
        r = c.tick({"left": Request(pose(0.3001, 0.3, 0.3))})
        self.assertEqual(r["left"].state, "held")
        self.assertIn("away from the hand", r["left"].notes[-1])
        self.assertAlmostEqual(p.cmd["left"][1], 0.3)


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
        for _ in range(30):
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
        for _ in range(40):
            r = c.tick({"left": Request(pose(0.3, 0.2, 0.3)), "right": Request(pose(0.3, -0.25, 0.3))})
        self.assertGreaterEqual(p.cmd["left"][1], 0.29)  # it does not go closer (it has nowhere to slide: it stays)
        self.assertIn("sliding past the other arm", r["left"].notes)
        self.assertAlmostEqual(p.cmd["right"][1], -0.25)

    def test_the_arm_slides_along_the_table_instead_of_stopping(self):
        table = 0.06  # in the stand-in, the left fingertips touch the table when the tool is at this height
        start = {"left": [0.3, 0.3, 0.16, 0, 0, 0, 1.0], "right": [0.3, -0.3, 0.3, 0, 0, 0, 1.0]}
        low = [dataclasses.replace(a, box=a.box[:4] + (-0.2, a.box[5])) for a in ARMS]
        pair = FakePair(start=start, problems=lambda q: {"left fingertip pressing into the table": q["left"][2] - table} if q["left"][2] < table else {})
        c = BimanualControl(pair, low, LIM)
        c.engage("left")
        slid = 0
        for i in range(150):  # the hand goes 5 cm below the table and 6 cm sideways
            k = min(1.0, i / 100)
            r = c.tick({"left": Request(pose(0.3 + 0.06 * k, 0.3, 0.16 - 0.15 * k))})["left"]
            self.assertGreaterEqual(pair.cmd["left"][2], table - 1e-9)  # never commanded into the table
            slid += "sliding along the table" in r.notes
        self.assertLess(pair.cmd["left"][2], table + 0.004)  # it rests on the table ...
        self.assertAlmostEqual(pair.cmd["left"][0], 0.36, delta=0.002)  # ... and went sideways with the hand
        self.assertGreater(slid, 0)
        for i in range(90):  # the hand comes back up: the arm follows it up at once
            c.tick({"left": Request(pose(0.36, 0.3, 0.01 + 0.15 * min(1.0, i / 60)))})
        self.assertAlmostEqual(pair.cmd["left"][2], 0.16, delta=0.002)
        self.assertIsNone(c.floor["left"])

    def test_moving_away_from_an_existing_problem_is_allowed(self):
        near = lambda q: {"left link5 and the table": 0.002 + (q["left"][2] - 0.1)} if q["left"][2] < 0.108 else {}  # noqa: E731
        start = {"left": [0.3, 0.3, 0.1, 0, 0, 0, 1.0], "right": [0.3, -0.3, 0.3, 0, 0, 0, 1.0]}
        c, p = control(FakePair(start, near))
        c.engage("left")
        c.tick({"left": Request(pose(0.3, 0.3, 0.09))})  # closer: held
        self.assertAlmostEqual(p.cmd["left"][2], 0.1)
        for _ in range(20):
            c.tick({"left": Request(pose(0.3, 0.3, 0.103))})  # away: allowed
        self.assertAlmostEqual(p.cmd["left"][2], 0.103)

    def test_both_moves_together_held_when_only_the_pair_is_a_problem(self):
        both = lambda q: {"left tip and right tip": 0.004} if q["left"][1] < 0.3 and q["right"][1] > -0.3 else {}  # noqa: E731
        c, p = control(FakePair(problems=both))
        c.engage("left"), c.engage("right")
        r = c.tick({"left": Request(pose(0.3, 0.298, 0.3)), "right": Request(pose(0.3, -0.298, 0.3))})
        self.assertEqual((r["left"].state, r["right"].state), ("following", "held"))  # one moves, the other waits this step
        self.assertEqual(p.cmd["right"][1], -0.3)
        self.assertLess(p.cmd["left"][1], 0.3)

    def test_slides_past_the_other_arm_instead_of_stopping(self):
        gap = 0.10  # in the stand-in, the grippers are too close when the tools are nearer than this in y
        near = lambda q: {"left tip_left and right tip_left": q["left"][1] - q["right"][1] - gap} if q["left"][1] - q["right"][1] < gap else {}  # noqa: E731
        start = {"left": [0.3, 0.07, 0.3, 0, 0, 0, 1.0], "right": [0.3, -0.07, 0.3, 0, 0, 0, 1.0]}
        c, p = control(FakePair(start=start, problems=near))
        p.approach_direction = lambda q, n: (0.0, 1.0, 0.0) if near(q) else None  # the parts face each other along y
        c.engage("left")
        slid = held = 0
        for i in range(160):  # the left hand goes 8 cm toward the right arm and 6 cm forward
            k = min(1.0, i / 80)
            r = c.tick({"left": Request(pose(0.3 + 0.06 * k, 0.07 - 0.08 * k, 0.3))})["left"]
            self.assertGreaterEqual(p.cmd["left"][1] - p.cmd["right"][1], gap - 1e-4 - 1e-9)  # never commanded closer than allowed
            slid, held = slid + ("sliding past the other arm" in r.notes), held + (r.state == "held")
        self.assertLess(held, 16)
        self.assertAlmostEqual(p.cmd["left"][0], 0.36, delta=0.002)  # forward with the hand ...
        self.assertLess(p.cmd["left"][1] - p.cmd["right"][1], gap + 0.004)  # ... stopped at the gap
        self.assertGreater(slid, 0)
        for i in range(110):  # and away again: follows at once
            c.tick({"left": Request(pose(0.36, -0.01 + 0.2 * min(1.0, i / 40), 0.3))})
        self.assertAlmostEqual(p.cmd["left"][1], 0.19, delta=0.002)

    def test_a_turn_the_wrist_cannot_make_keeps_the_angle_and_follows_the_position(self):
        class Wrist(FakePair):  # every pose turned more than 0.2 rad needs another arm posture: a far jump of a joint
            def solve(self, n, p, q6_init, orientation_cost=1.0):
                q = list(p[0]) + list(q_to_rotvec(p[1]))
                if q_angle(p[1], UP) > 0.2:
                    q[5] += 2.0
                return q, 0.0, 0.0

        c, p = control(Wrist())
        c.engage("left")
        turned = q_from_rotvec((0.0, 0.0, 0.6))
        for i in range(150):
            r = c.tick({"left": Request(pose(0.3 + 0.05 * min(1.0, i / 60), 0.3, 0.3, turned))})["left"]
            self.assertNotEqual(r.state, "held")
        self.assertAlmostEqual(p.cmd["left"][0], 0.35, delta=0.002)  # the position followed
        self.assertLess(abs(p.cmd["left"][5]), 0.21)  # the angle stopped where the wrist can go, no jump
        self.assertTrue(any("wrist at its limit" in x for x in r.notes))

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
