"""The colour-and-depth server's distance coding, and that the headset page decodes it with the same numbers."""

import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import lab_rgbd  # noqa: E402

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


class DepthCodeTests(unittest.TestCase):
    @unittest.skipIf(np is None, "numpy not installed")
    def test_round_trip(self):
        z = np.array([0.0, 0.25, 0.5, 1.0, 2.0, 3.0, 9.0], dtype=np.float32)
        g = lab_rgbd.encode_depth(z, np)
        self.assertEqual(g[0], 0)  # none measured
        self.assertEqual(g[1], 255)  # NEAR
        self.assertEqual(g[-1], 8)  # beyond FAR: the last valid grey, never 0
        for zi, gi in zip(z[2:6], g[2:6]):
            self.assertLess(abs(lab_rgbd.decode_depth(float(gi)) - zi) / zi, 0.03)

    def test_page_uses_the_same_range(self):
        page = (ROOT / "scripts/web/headset.html").read_text()
        m = re.search(r"const DEPTH_NEAR = ([0-9.]+), DEPTH_FAR = ([0-9.]+);", page)
        self.assertIsNotNone(m)
        self.assertEqual((float(m.group(1)), float(m.group(2))), (lab_rgbd.NEAR, lab_rgbd.FAR))

    def test_camera_file(self):
        import json

        cam = json.loads((ROOT / "scripts/web/depth_camera.json").read_text())
        self.assertEqual(sorted(cam), ["cx", "cy", "focal", "pitch_deg", "roll_deg"])


if __name__ == "__main__":
    unittest.main()
