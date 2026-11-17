#!/usr/bin/env python3
"""Bimanual YAM demo rows and statistics (the recorder's camera and file machinery is the xArm's, tested there)."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from yam_recorder import ARM_FIELDS, YamEpisodeRecorder  # noqa: E402


def arm(clutch="ACTIVE", p=(0.3, 0.2, 0.3), grip=1.0):
    return {"clutch": clutch, "arm": "following", "q_cmd": [0.0] * 6 + [grip], "q_meas": [0.0] * 6 + [grip], "tcp_p": list(p),
            "tcp_q": [0, 0, 0, 1], "target_p": list(p), "target_q": [0, 0, 0, 1], "grip_cmd": grip, "trig": 0.0, "squeeze": 1.0,
            "hand_p": [0, 1, 0], "hand_q": [0, 0, 0, 1], "input_age_ms": 5.0, "notes": [], "extra": "not saved"}  # fmt: skip


def episode():
    return {"active_ns": 0, "last_p": None, "travel_mm": 0.0, "rows_without_pose": 0, "grip_min": math.inf, "grip_max": -math.inf}


class RowTests(unittest.TestCase):
    def test_row_has_both_arms_with_exactly_the_documented_fields(self):
        rec = {"k": "tick", "t": 5, "dt_ms": 20.0, "arms": {"left": arm(), "right": arm("WAIT_RELEASE")}, "btn_a": False}
        row = YamEpisodeRecorder._row(rec, None)
        self.assertEqual(set(row), {"t", "dt_ms", "left", "right"})
        self.assertEqual(set(row["left"]), set(ARM_FIELDS))
        self.assertEqual(row["right"]["clutch"], "WAIT_RELEASE")

    def test_statistics_for_two_arms(self):
        ep = episode()
        for k, (pl, g) in enumerate((((0.3, 0.2, 0.3), 1.0), ((0.31, 0.2, 0.3), 0.5), ((0.31, 0.2, 0.32), 0.0))):
            rec = {"t": k, "dt_ms": 20.0, "arms": {"left": arm(p=pl, grip=g), "right": arm("WAIT_RELEASE", p=(0.3, -0.2, 0.3))}}
            YamEpisodeRecorder._count(ep, YamEpisodeRecorder._row(rec, None), rec)
        self.assertEqual(ep["active_ns"], 60_000_000)  # one arm following counts
        self.assertAlmostEqual(ep["travel_mm"], 30.0, places=6)  # 10 mm + 20 mm on the left, the right still
        self.assertEqual((ep["grip_min"], ep["grip_max"]), (0.0, 100.0))  # percent closed, as the xArm's
        self.assertEqual(ep["rows_without_pose"], 0)

    def test_session_files_name_the_yam_format(self):
        self.assertEqual(YamEpisodeRecorder.FORMAT, "quest-yam-bimanual-episode-v1")
        self.assertIn("left", YamEpisodeRecorder.STEP_FIELDS)


if __name__ == "__main__":
    unittest.main()
