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
        from yam_setup import ARMS, Limits
        from yam_sim import SimYamPair

        pair = SimYamPair(ARMS, Limits())
        try:
            cls.results = yam_sim_check.scenarios(pair)
        finally:
            pair.close()

    def test_every_scenario_passes(self):
        failed = [f"{name}: {why}" for name, ok, why in self.results if not ok]
        self.assertFalse(failed, "\n".join(failed))
        self.assertGreaterEqual(len(self.results), 10)


if __name__ == "__main__":
    unittest.main()
