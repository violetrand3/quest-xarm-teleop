"""Tests for wrist rotation: the rotation math and the adapter's rotation following.

ALL DATA HERE IS SYNTHETIC. A fake backend records what was requested; nothing here shows how a real
arm rotates. No Quest, no robot.
"""
from __future__ import annotations

import json
import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import quest_pose_v1 as qp  # noqa: E402
import teleop_adapter as ta  # noqa: E402
import teleop_rotation as tr  # noqa: E402
from test_teleop_adapter import CONFIG, P0, Rig, off  # noqa: E402

D = math.radians


def about(axis: str, deg: float) -> tuple:
    s, c = math.sin(D(deg) / 2), math.cos(D(deg) / 2)
    return {"x": (s, 0.0, 0.0, c), "y": (0.0, s, 0.0, c), "z": (0.0, 0.0, s, c)}[axis]


def matmul3(a, b):  # type: ignore[no-untyped-def]
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rcfg(mode: str = "full", **rot):  # type: ignore[no-untyped-def]
    d = json.loads(CONFIG.read_text())
    d["rotation"] = dict({"mode": mode, "scale": 1.0, "max_angle_deg": 30, "w_max_deg_s": 40, "max_error_deg": 10, "max_step_deg": 90}, **rot)
    return ta.parse_config(d)


class RotationMathTests(unittest.TestCase):
    def test_rpy_convention_is_rz_ry_rx(self) -> None:
        for roll, pitch, yaw in ((179.73, -8.63, -6.95), (10, 20, 30), (-170, 5, 90), (0, 0, 0), (180, 0, 0), (-179.9, -40, 120)):
            cr, sr, cp, sp, cy, sy = math.cos(D(roll)), math.sin(D(roll)), math.cos(D(pitch)), math.sin(D(pitch)), math.cos(D(yaw)), math.sin(D(yaw))
            rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
            ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
            rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
            want = matmul3(rz, matmul3(ry, rx))
            got = qp.quat_to_rot(tr.rpy_to_q(D(roll), D(pitch), D(yaw)))
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(got[i][j], want[i][j], places=9)

    def test_rpy_round_trip_including_near_180_roll(self) -> None:
        random.seed(3)
        cases = [(D(179.73), D(-8.63), D(-6.95)), (D(-179.99), 0.1, -0.2), (math.pi, 0.0, 0.0)]
        cases += [(random.uniform(-math.pi, math.pi), random.uniform(-1.4, 1.4), random.uniform(-math.pi, math.pi)) for _ in range(300)]
        for rpy in cases:
            q = tr.rpy_to_q(*rpy)
            back = tr.q_to_rpy(q)
            self.assertLess(tr.q_angle(q, tr.rpy_to_q(*back)), 1e-9)  # the same rotation, whatever the angle wrap
            self.assertLessEqual(abs(back[1]), math.pi / 2 + 1e-9)

    def test_rotvec_angle_step_and_twist(self) -> None:
        q = about("z", 90)
        r = tr.q_to_rotvec(q)
        self.assertAlmostEqual(r[2], math.pi / 2, places=12)
        self.assertLess(tr.q_angle(tr.q_from_rotvec(r), q), 1e-12)
        self.assertLess(tr.q_angle(q, tuple(-v for v in q)), 1e-7)  # q and -q are the same rotation
        self.assertAlmostEqual(math.degrees(tr.q_angle(tr.IDENTITY, q)), 90.0, places=9)
        s = tr.q_step_toward(tr.IDENTITY, q, D(10))
        self.assertAlmostEqual(math.degrees(tr.q_angle(tr.IDENTITY, s)), 10.0, places=9)
        self.assertAlmostEqual(math.degrees(tr.q_angle(s, q)), 80.0, places=9)  # on the way, not somewhere else
        self.assertLess(tr.q_angle(tr.q_step_toward(tr.IDENTITY, q, D(100)), q), 1e-12)
        self.assertLess(tr.q_angle(tr.q_twist(about("x", 40), (0, 1, 0)), tr.IDENTITY), 1e-12)
        self.assertLess(tr.q_angle(tr.q_twist(about("y", 40), (0, 1, 0)), about("y", 40)), 1e-12)
        both = tr.q_mul(about("y", 25), about("x", 30))  # tilt, then turn about "up"
        self.assertAlmostEqual(math.degrees(tr.q_angle(tr.IDENTITY, tr.q_twist(both, (0, 1, 0)))), 25.0, delta=1.5)
        self.assertLess(tr.q_angle(tr.q_mul(about("z", 30), about("z", 20)), about("z", 50)), 1e-12)

    def test_config_validation(self) -> None:
        self.assertIsNone(ta.parse_config(json.loads(CONFIG.read_text())).rotation)
        self.assertAlmostEqual(rcfg("yaw").rotation.max_angle_rad, D(30))
        for bad in ({"mode": "roll"}, {"scale": 0}, {"max_angle_deg": 200}, {"w_max_deg_s": -1}):
            with self.assertRaises(ta.ConfigError):
                rcfg(**bad)
        d = json.loads(CONFIG.read_text())
        d["rotation"] = {"mode": "yaw"}
        with self.assertRaises(ta.ConfigError):
            ta.parse_config(d)


class AdapterRotationTests(unittest.TestCase):
    def tool_q(self, r: Rig) -> tuple:
        return r.backend.read_feedback().q_xyzw

    def turn(self, r: Rig, axis: str, deg: float, steps: int, hold: int = 80, p=P0, start=None) -> dict:  # type: ignore[no-untyped-def]
        out = {}
        base = tr.IDENTITY if start is None else start
        for i in range(1, steps + 1):
            out = r.step(p, 1.0, q=tr.q_mul(about(axis, deg * i / steps), base))
        for _ in range(hold):
            out = r.step(p, 1.0, q=tr.q_mul(about(axis, deg), base))
        return out

    def test_tool_follows_the_hand_about_the_mapped_axis_at_a_limited_rate(self) -> None:
        r = Rig(rcfg("full"), rotation=True)
        r.engage()
        out = self.turn(r, "y", 20, 10)  # hand turns 20 deg about LOCAL up in 0.2 s
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), about("z", 20))), 0.01)  # LOCAL up is the backend's +z
        self.assertAlmostEqual(out["rot_deg"], 20.0, places=2)
        qs = [c[2][1:] for c in r.backend.commands if c[2][0] == "q"]
        self.assertLessEqual(max(math.degrees(tr.q_angle(a, b)) for a, b in zip(qs, qs[1:])), 40 * 0.02 + 1e-6)  # 40 deg/s
        self.assertEqual({c[1] for c in r.backend.commands}, {(0.0, 0.0, 0.0)})  # a pure hand rotation asks for no translation
        r2 = Rig(rcfg("full"), rotation=True)
        r2.engage()
        self.turn(r2, "x", 15, 10)  # LOCAL +x is the backend's -y in the example mapping
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r2), about("y", -15))), 0.01)

    def test_first_target_keeps_the_tool_orientation_exactly(self) -> None:
        r = Rig(rcfg("full"), rotation=True)
        r.step(P0, 0.0, q=about("x", 70))  # the hand is at some arbitrary orientation when the clutch is pressed
        out = r.step(P0, 1.0, q=about("x", 70))
        self.assertEqual(out["state"], ta.ACTIVE)
        self.assertEqual(r.backend.commands[0][2], r.backend.START_ORIENTATION)  # first target: the feedback orientation, verbatim
        for _ in range(20):
            r.step(P0, 1.0, q=about("x", 70))
        self.assertLess(tr.q_angle(self.tool_q(r), tr.IDENTITY), 1e-9)  # no rotation until the hand turns

    def test_yaw_mode_only_passes_the_turn_about_up(self) -> None:
        r = Rig(rcfg("yaw"), rotation=True)
        r.engage()
        self.turn(r, "x", 25, 10)
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), tr.IDENTITY)), 0.01)  # tilting the hand does nothing
        self.turn(r, "z", 25, 10)
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), tr.IDENTITY)), 0.01)
        self.turn(r, "y", 15, 10)
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), about("z", 15))), 0.01)

    def test_rotation_is_held_at_the_limit_and_clutching_cannot_ratchet_it(self) -> None:
        r = Rig(rcfg("full"), rotation=True)
        r.engage()
        out = self.turn(r, "y", 50, 25)  # asks for 50 deg; the limit is 30
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertAlmostEqual(math.degrees(tr.q_angle(self.tool_q(r), tr.IDENTITY)), 30.0, places=3)
        r.step(P0, 0.0, q=about("y", 50))  # clutch: release, bring the hand back, press, turn again
        r.step(P0, 0.0, q=tr.IDENTITY)
        self.assertEqual(r.step(P0, 1.0, q=tr.IDENTITY)["state"], ta.ACTIVE)
        self.turn(r, "y", 50, 25)
        self.assertAlmostEqual(math.degrees(tr.q_angle(self.tool_q(r), tr.IDENTITY)), 30.0, places=3)  # still 30 from the session's first orientation
        self.turn(r, "y", -20, 20, start=about("y", 50))  # hand back to +30: still past the limit, like a hand beyond the box edge
        self.assertAlmostEqual(math.degrees(tr.q_angle(self.tool_q(r), tr.IDENTITY)), 30.0, places=3)
        self.turn(r, "y", -50, 25, start=about("y", 30))  # hand at -20 from this engagement: 30 - 20 = 10
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), about("z", 10))), 0.01)

    def test_orientation_jump_is_rejected_before_it_is_rate_limited(self) -> None:
        r = Rig(rcfg("full"), rotation=True)
        r.engage()
        n = r.backend.command_count
        out = r.step(P0, 1.0, q=about("y", 120))  # 120 deg between two samples
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("controller_discontinuity:rotation step"))
        self.assertEqual((r.backend.command_count, r.backend.latched_stop_count), (n, 1))

    def test_rotation_tracking_error_latches(self) -> None:
        r = Rig(rcfg("full", max_angle_deg=60), rotation=True)
        r.engage()
        r.backend.tracking = False  # the synthetic tool stops following
        out = {}
        for i in range(1, 80):
            out = r.step(P0, 1.0, q=about("y", min(40, i)))
            if out["state"] != ta.ACTIVE:
                break
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("target_feedback_error:rotation"), r.rec.faults[-1])

    def test_engagement_is_refused_when_the_backend_gives_no_orientation(self) -> None:
        r = Rig(rcfg("full"))  # synthetic backend without rotation
        r.step(P0, 0.0)
        out = r.step(P0, 1.0)
        self.assertEqual(out["state"], ta.WAIT_RELEASE)
        self.assertIn("no orientation from the backend", out["blocked"])
        self.assertEqual(r.backend.command_count, 0)

    def test_translation_and_rotation_together(self) -> None:
        r = Rig(rcfg("full"), rotation=True)
        r.engage()
        out = {}
        for i in range(1, 41):  # hand moves 100 mm along LOCAL -z while turning 20 deg about up
            out = r.step(off(P0, (0.0, 0.0, -0.0025 * i)), 1.0, q=about("y", 0.5 * i))
        for _ in range(60):
            out = r.step(off(P0, (0.0, 0.0, -0.10)), 1.0, q=about("y", 20))
        self.assertAlmostEqual(out["lim_mm"][0], 20.0, places=3)  # 100 mm at the example scale 0.2
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), about("z", 20))), 0.01)

    def test_release_and_input_loss_stop_rotation_targets(self) -> None:
        r = Rig(rcfg("full"), rotation=True)
        r.engage()
        for i in range(1, 6):
            r.step(P0, 1.0, q=about("y", 4 * i))  # asks for 20 deg; the limited target is still on its way
        n = r.backend.command_count
        r.step(P0, 0.0, q=about("y", 20))
        for _ in range(30):
            r.step(P0, 0.0, q=about("y", 20))
        self.assertEqual(r.backend.command_count, n)  # the rest of the rotation is dropped, not finished
        self.assertLess(math.degrees(tr.q_angle(self.tool_q(r), tr.IDENTITY)), 19.0)


if __name__ == "__main__":
    unittest.main()
