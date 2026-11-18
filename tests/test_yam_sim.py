#!/usr/bin/env python3
"""Bimanual YAM in simulation: every scenario of scripts/yam_sim_check.py passes. Needs I2RT's environment
(MuJoCo, mink, i2rt), so it runs on the lab host and is skipped elsewhere:

    cd <copy of this repository> && /home/dev/yam/i2rt/.venv/bin/python -m unittest tests.test_yam_sim
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

HAVE_SIM = all(importlib.util.find_spec(m) is not None for m in ("mujoco", "mink", "i2rt"))


@unittest.skipUnless(HAVE_SIM, "needs I2RT's environment (MuJoCo, mink, i2rt): run on the lab host")
class SimScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import yam_sim_check
        from yam_setup import ARMS, SPEEDS, limits_at
        from yam_sim import SimYamPair

        cls.results = {}
        for speed in SPEEDS:  # every speed level
            lim = limits_at(speed)
            pair = SimYamPair(ARMS, lim)
            try:
                cls.results[speed] = yam_sim_check.scenarios(pair, lim)
            finally:
                pair.close()

    def test_every_scenario_passes_at_every_speed_level(self):
        failed = [f"{speed:g} cm/s, {name}: {why}" for speed, res in self.results.items() for name, ok, why in res if not ok]
        self.assertFalse(failed, "\n".join(failed))
        self.assertTrue(all(len(res) >= 10 for res in self.results.values()))




@unittest.skipUnless(HAVE_SIM, "needs I2RT's environment (MuJoCo, mink, i2rt): run on the lab host")
class RealPairStartTests(unittest.TestCase):
    """yam_real's start-up checks, with I2RT's get_yam_robot replaced by a stand-in: NO HARDWARE is opened."""

    def fake_arms(self, grip):
        import numpy as np

        made = []

        class Arm:
            def __init__(self, name):
                self.name, self.closed = name, False

            def get_joint_pos(self):
                return np.array([0.0] * 6 + [grip[self.name]])

            def close(self):
                self.closed = True

        def get_yam_robot(channel, **kw):
            assert kw["sim"] is False and kw["zero_gravity_mode"] is False and kw["gripper_limits_override"] is not None
            made.append(Arm("left" if channel == "can_left" else "right"))
            return made[-1]

        return made, get_yam_robot

    def open_pair(self, grip):
        import json
        import tempfile
        from pathlib import Path
        from unittest import mock

        import yam_real
        from yam_setup import ARMS, Limits

        made, fake = self.fake_arms(grip)
        with tempfile.TemporaryDirectory() as d:
            hw = Path(d) / "hw.json"
            hw.write_text(json.dumps({"measured": "test", "arms": {n: {"gripper_limits": [0.001, -5.069], "rest_q": [0.0] * 6} for n in ("left", "right")}}))
            with mock.patch.object(yam_real, "get_yam_robot", fake):
                try:
                    return yam_real.RealYamPair(ARMS, Limits(), hw), made, None
                except yam_real.GripperStartError as e:
                    return None, made, e

    def test_grippers_inside_their_stroke_open(self):
        pair, made, err = self.open_pair({"left": 0.3, "right": 0.0})
        self.assertIsNone(err)
        self.assertEqual([a.closed for a in made], [False, False])
        pair.close()
        self.assertEqual([a.closed for a in made], [True, True])

    def test_a_gripper_shifted_by_a_turn_refuses_and_switches_everything_off(self):
        pair, made, err = self.open_pair({"left": 0.2, "right": -0.30})  # the right one came back 2*pi off
        self.assertIsNone(pair)
        self.assertIn("right gripper reads -0.30", str(err))
        self.assertTrue(made and all(a.closed for a in made))


if __name__ == "__main__":
    unittest.main()
