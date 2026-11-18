#!/usr/bin/env python3
"""Bimanual YAM, the parts around the real arms that run anywhere: lowering to rest, the bring-up's hardware file,
and the live program refusing real arms without the operator's confirmation. Nothing here opens hardware."""

from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

from test_yam_control import FakePair  # noqa: E402
from yam_control import BimanualControl, Request  # noqa: E402
from yam_setup import ARMS, HardwareFileError, Limits, load_hardware  # noqa: E402

LIM = Limits()
REST = {"left": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0], "right": [0.0, -0.3, 0.3, 0.0, 0.0, 0.0]}
GOOD = {"measured": "test", "arms": {n: {"gripper_limits": [0.0, -6.5], "rest_q": [0.0] * 6} for n in ("left", "right")}}


class ParkTests(unittest.TestCase):
    def test_lowers_both_at_the_joint_speed_and_lets_go(self):
        c = BimanualControl(FakePair(), ARMS, LIM)
        c.engage("left")
        steps, done = 0, False
        while not done and steps < 500:
            q0 = {n: list(c.q[n]) for n in c.arms}
            done, said = c.park_tick(REST, 0.3)
            steps += 1
            for n in c.arms:
                self.assertLessEqual(max(abs(a - b) for a, b in zip(q0[n][:6], c.q[n][:6])), 0.3 * LIM.dt + 1e-12)
        self.assertTrue(done)
        self.assertFalse(any(c.engaged.values()))
        self.assertEqual([round(v, 9) for v in c.q["left"][:6]], REST["left"])
        self.assertEqual(c.q["left"][6], 1.0)  # the gripper is left as it was
        self.assertEqual(said, {"left": "at rest", "right": "at rest"})

    def test_an_arm_that_would_come_too_close_waits_for_the_other(self):
        # the left arm may not go below y 0.31 while the right arm is still above y -0.29
        rule = lambda q: {"left tip and right tip": 0.005} if q["left"][1] < 0.31 and q["right"][1] > -0.29 else {}  # noqa: E731
        p = FakePair({"left": [0.3, 0.32, 0.3, 0, 0, 0, 1.0], "right": [0.3, -0.25, 0.3, 0, 0, 0, 1.0]}, rule)
        c = BimanualControl(p, ARMS, LIM)
        done, said = c.park_tick(REST, 0.3)
        for _ in range(400):
            if done:
                break
            done, s = c.park_tick(REST, 0.3)
            said = said if "waiting" in said["left"] else s
        self.assertTrue(done)
        self.assertTrue(said["left"].startswith("waiting") or said["left"] == "lowering")

    def test_an_open_gripper_is_closed_to_the_rest_maximum(self):
        c = BimanualControl(FakePair(), ARMS, LIM)  # grippers start fully open (1.0)
        done, steps, g = False, 0, [1.0]
        while not done and steps < 500:
            done, _ = c.park_tick(REST, 0.3, grip_max=0.4)
            g.append(c.q["left"][6])
            steps += 1
        self.assertTrue(done)
        self.assertAlmostEqual(c.q["left"][6], 0.4)
        self.assertLessEqual(max(a - b for a, b in zip(g, g[1:])), LIM.grip_speed * LIM.dt + 1e-12)

    def test_teleop_requests_are_ignored_after_parking_starts(self):
        c = BimanualControl(FakePair(), ARMS, LIM)
        c.engage("left")
        c.park_tick(REST, 0.3)
        r = c.tick({"left": Request(((0.5, 0.3, 0.3), (0, 0, 0, 1)))})
        self.assertEqual(r["left"].state, "idle")


class HardwareFileTests(unittest.TestCase):
    def check(self, content, ranges=None):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "hw.json"
            if content is not None:
                p.write_text(content if isinstance(content, str) else json.dumps(content))
            return load_hardware(p, ["left", "right"], ranges)

    def test_good(self):
        self.assertEqual(self.check(GOOD)["right"]["gripper_limits"], [0.0, -6.5])

    def test_refusals(self):
        bad = [None, "{not json", {"arms": GOOD["arms"]}, {"measured": "x", "arms": {"left": GOOD["arms"]["left"]}},
               {"measured": "x", "arms": {n: {"gripper_limits": [1.0, 1.0], "rest_q": [0.0] * 6} for n in ("left", "right")}},
               {"measured": "x", "arms": {n: {"gripper_limits": [0.0, 1.0], "rest_q": [0.0] * 5} for n in ("left", "right")}},
               {"measured": "x", "arms": {n: {"gripper_limits": [0.0, float("nan")], "rest_q": [0.0] * 6} for n in ("left", "right")}}]  # fmt: skip
        for content in bad:
            with self.assertRaises(HardwareFileError, msg=str(content)[:60]):
                self.check(content)

    def test_rest_pose_outside_the_ranges(self):
        with self.assertRaises(HardwareFileError):
            self.check(GOOD, [(0.1, 1.0)] * 6)


class LiveProgramTests(unittest.TestCase):
    def test_real_arms_need_the_confirmation(self):
        import yam_live

        for argv in (["--arms", "real"], ["--arms", "real", "--moves-real-arms"], ["--arms", "real", "--human-confirmation", "yes"]):
            err = io.StringIO()
            with redirect_stderr(err):
                self.assertEqual(yam_live.main(argv), 2)
            self.assertIn("nothing was opened", err.getvalue())

    def test_only_one_program_may_hold_the_real_arms(self):
        import yam_live

        with tempfile.TemporaryDirectory() as d:
            lock = Path(d) / "arms.lock"
            first = yam_live.take_arms_lock(lock)
            self.assertIsNotNone(first)
            self.assertIsNone(yam_live.take_arms_lock(lock))  # a second program is refused
            first.close()  # the first program ended
            again = yam_live.take_arms_lock(lock)
            self.assertIsNotNone(again)
            again.close()

    def test_first_runs_are_at_half_speed(self):
        import yam_live

        h = yam_live.first_run_limits(LIM)
        self.assertEqual((h.tool_speed, h.joint_speed), (LIM.tool_speed / 2, LIM.joint_speed / 2))


if __name__ == "__main__":
    unittest.main()
