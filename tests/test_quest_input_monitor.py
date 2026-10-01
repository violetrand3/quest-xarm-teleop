"""Offline tests for scripts/quest_input_monitor.py (stdlib unittest, no device)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import quest_input_monitor as qim  # noqa: E402

IDENT = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
# Row-major: translation in the 4th column (indices 3, 7, 11).
R_POSE = "0.999336 0.0220316 0.0290289 -0.526015 -0.0268525 0.983694 0.177833 0.416647 -0.0246376 -0.178495 0.983632 0.25859 0 0 0 1"
BUTTONS = "L,leftJS 0.000000 0.000000,leftTrig 0.000000,leftGrip 0.000000,R,rightJS 0.000000 0.000000,rightTrig 0.000000,rightGrip 0.000000"
# Verbatim shape of a live line from the headset (logcat -v epoch -v usec -v threadtime).
LIVE_LINE = f"         1790800025.520487 19405 19437 I wE9ryARX: l:{IDENT} |r:{R_POSE} &{BUTTONS}\n"

MS = 1_000_000


class ParseTests(unittest.TestCase):
    def test_valid_protocol_message(self) -> None:
        line = qim.parse_logcat_line(LIVE_LINE)
        assert line is not None
        self.assertEqual(line.device_log_time, "1790800025.520487")
        self.assertEqual(line.pid, 19405)
        msg = qim.parse_message(line.message)
        self.assertEqual(msg.pose_present, {"r": True, "l": True})
        self.assertEqual(msg.translation("r"), (-0.526015, 0.416647, 0.25859))
        assert msg.poses["r"] is not None
        self.assertEqual(msg.poses["r"][3], [0.0, 0.0, 0.0, 1.0])
        self.assertTrue(msg.pose_checks["r"].homogeneous_ok)  # type: ignore[union-attr]
        self.assertFalse(msg.buttons["A"])
        self.assertEqual(msg.buttons["rightJS"], (0.0, 0.0))

    def test_non_tag_lines_ignored(self) -> None:
        self.assertIsNone(qim.parse_logcat_line("--------- beginning of main\n"))
        other = LIVE_LINE.replace("wE9ryARX", "OtherTag")
        self.assertIsNone(qim.parse_logcat_line(other))

    def test_only_one_controller(self) -> None:
        msg = qim.parse_message(f"r:{R_POSE}&R,rightTrig 0.5,rightGrip 0.25")
        self.assertEqual(msg.pose_present, {"r": True, "l": False})
        self.assertIsNone(msg.poses["l"])  # missing, not zeros/identity
        self.assertIsNone(msg.translation("l"))
        self.assertNotIn("X", msg.buttons)  # left booleans absent without 'L'
        self.assertIn("A", msg.buttons)

    def test_no_controllers(self) -> None:
        msg = qim.parse_message("&")
        self.assertEqual(msg.pose_present, {"r": False, "l": False})
        self.assertEqual(msg.buttons, {})

    def test_malformed_matrix(self) -> None:
        bad = [
            f"r:{' '.join(IDENT.split()[:15])}&R",  # 15 values
            f"r:{IDENT} 1&R",  # 17 values
            "r:1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 x&R",  # not a number
            f"q:{IDENT}&R",  # unknown controller key
            f"r:{IDENT}|r:{IDENT}&R",  # duplicate
            f"r:{IDENT}",  # no '&'
            f"r:{IDENT}&R&L",  # two '&'
        ]
        for payload in bad:
            with self.subTest(payload=payload[-30:]):
                with self.assertRaises(qim.MessageError):
                    qim.parse_message(payload)

    def test_non_finite_matrix(self) -> None:
        for token in ("nan", "inf", "-inf", "NaN"):
            with self.subTest(token=token):
                with self.assertRaises(qim.MessageError):
                    qim.parse_message(f"r:1 0 0 {token} 0 1 0 0 0 0 1 0 0 0 0 1&R")

    def test_homogeneous_inconsistency_flagged_not_rejected(self) -> None:
        msg = qim.parse_message("r:2 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1&R")  # scaled rotation
        check = msg.pose_checks["r"]
        assert check is not None
        self.assertFalse(check.homogeneous_ok)
        msg = qim.parse_message("r:1 0 0 0 0 1 0 0 0 0 1 0 0 0 1 1&R")  # bad bottom row
        self.assertFalse(msg.pose_checks["r"].homogeneous_ok)  # type: ignore[union-attr]

    def test_analog_button_parsing(self) -> None:
        buttons, unknown = qim.parse_buttons(
            "R,A,RTr,rightJS -0.25 0.75,rightTrig 0.831373,rightGrip 1.000000,L,Y,leftTrig 0.0"
        )
        self.assertEqual(buttons["rightTrig"], (0.831373,))
        self.assertEqual(buttons["rightGrip"], (1.0,))
        self.assertEqual(buttons["rightJS"], (-0.25, 0.75))
        self.assertEqual(buttons["leftTrig"], (0.0,))
        self.assertTrue(buttons["A"])
        self.assertTrue(buttons["RTr"])
        self.assertFalse(buttons["B"])
        self.assertTrue(buttons["Y"])
        self.assertFalse(buttons["X"])
        self.assertEqual(unknown, [])

    def test_bad_analog_and_unknown_tokens(self) -> None:
        with self.assertRaises(qim.MessageError):
            qim.parse_buttons("R,rightTrig nan")
        with self.assertRaises(qim.MessageError):
            qim.parse_buttons("R,rightTrig abc")
        _, unknown = qim.parse_buttons("R,Zz")
        self.assertEqual(unknown, ["Zz"])

    def test_malformed_line_counted_not_crashing(self) -> None:
        tracker = qim.FreshnessTracker()
        mon = qim.Monitor(tracker, out=None)
        mon.handle_line(LIVE_LINE.replace("0.999336", "nan", 1), 1)
        mon.handle_line(LIVE_LINE, 2)
        self.assertEqual(tracker.errors, 1)
        self.assertEqual(tracker.total, 1)


class FreshnessTests(unittest.TestCase):
    def _msg(self) -> qim.ControllerMessage:
        return qim.parse_message(f"l:{IDENT} |r:{R_POSE} &{BUTTONS}")

    def test_stale_when_no_new_messages(self) -> None:
        t = qim.FreshnessTracker(stale_ms=500)
        t.set_transport(True, "test", 0)
        self.assertEqual(t.snapshot(0).status, qim.Status.STALE)  # nothing received yet
        t.record_message(self._msg(), 1_000 * MS)
        s = t.snapshot(1_100 * MS)
        self.assertEqual((s.status, s.new_messages), (qim.Status.LIVE, 1))
        # Repeated polling/printing must not refresh freshness or counts.
        for now in (1_200, 1_400, 1_499):
            s = t.snapshot(now * MS)
            self.assertEqual((s.status, s.new_messages, s.total_messages), (qim.Status.LIVE, 0, 1))
        s = t.snapshot(1_501 * MS)
        self.assertEqual(s.status, qim.Status.STALE)
        self.assertAlmostEqual(s.age_ms or 0, 501.0)
        self.assertEqual(t.total, 1)
        self.assertEqual(t.last_recv_ns, 1_000 * MS)

    def test_disconnected_overrides(self) -> None:
        t = qim.FreshnessTracker(stale_ms=500)
        t.set_transport(True, "test", 0)
        t.record_message(self._msg(), 10 * MS)
        t.set_transport(False, "app not running", 20 * MS)
        self.assertEqual(t.snapshot(20 * MS).status, qim.Status.DISCONNECTED)

    def test_fresh_identical_poses_stay_live(self) -> None:
        t = qim.FreshnessTracker(stale_ms=500)
        t.set_transport(True, "test", 0)
        for i in range(200):  # stationary controller, ~72 Hz, 2.8 s
            t.record_message(self._msg(), i * 14 * MS)
            s = t.snapshot(i * 14 * MS + 5 * MS, consume=False)
            self.assertEqual(s.status, qim.Status.LIVE)
        s = t.snapshot(199 * 14 * MS + 5 * MS)
        self.assertEqual(s.total_messages, 200)
        self.assertGreater(s.rate_hz, 60)
        self.assertIn("track=unknown", qim.format_status(s))  # never invented True

    def test_gap_stats(self) -> None:
        st = qim.gap_stats_ms([10 * MS] * 19 + [900 * MS], stale_ms=500)
        self.assertEqual(st["median_ms"], 10.0)
        self.assertEqual(st["p95_ms"], 10.0)
        self.assertEqual(st["max_ms"], 900.0)
        self.assertEqual(st["gaps_over_stale_threshold"], 1)

    def test_record_contains_required_fields(self) -> None:
        line = qim.parse_logcat_line(LIVE_LINE)
        assert line is not None
        rec = qim.message_record(7, 123, line, qim.parse_message(line.message))
        self.assertEqual(rec["seq"], 7)
        self.assertEqual(rec["host_monotonic_ns"], 123)
        self.assertEqual(rec["device_log_time"], "1790800025.520487")
        self.assertIsNone(rec["tracking_valid"])
        self.assertEqual(rec["pose_present"], {"r": True, "l": True})
        self.assertEqual(rec["buttons"]["rightTrig"], [0.0])


if __name__ == "__main__":
    unittest.main()
