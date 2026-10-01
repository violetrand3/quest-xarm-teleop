"""Offline tests for QuestXarmPoseV1 (Step 4A).

ALL DATA HERE IS SYNTHETIC. These tests exercise the decoder, the pose math and
(where a host C++ compiler exists) the app's own formatter header compiled on
this PC. They do NOT establish Quest 3S compatibility, OpenXR runtime behaviour
or physical tracking performance.
"""

from __future__ import annotations

import copy
import json
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import quest_pose_v1 as qp  # noqa: E402

APP_SRC = ROOT / "external" / "oculus_reader_openxr" / "app" / "src"
HARNESS = ROOT / "tests" / "fixtures" / "qx_format_harness.cpp"
ALL = qp.POSITION_VALID | qp.ORIENTATION_VALID | qp.POSITION_TRACKED | qp.ORIENTATION_TRACKED
S45 = math.sin(math.radians(22.5))
C45 = math.cos(math.radians(22.5))


def syn_pose(p=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0), flags=ALL, active=True, head=False) -> dict:  # type: ignore[no-untyped-def]
    d = {
        "queried": True, "result": 0, "flags": flags,
        "pos_valid": bool(flags & qp.POSITION_VALID), "ori_valid": bool(flags & qp.ORIENTATION_VALID),
        "pos_tracked": bool(flags & qp.POSITION_TRACKED), "ori_tracked": bool(flags & qp.ORIENTATION_TRACKED),
        "p": list(p) if flags & qp.POSITION_VALID else None,
        "q": list(q) if flags & qp.ORIENTATION_VALID else None,
    }  # fmt: skip
    if not head:
        d["active"] = active
    return d


def syn_inputs(right: bool, **vals) -> dict:  # type: ignore[no-untyped-def]
    d = {k: None for k in qp._INPUT_FIELDS["in_r" if right else "in_l"]}
    d.update(vals)
    return d


def syn_record(seq=1, sid="00000000deadbeef", t=1_000_000_000, head=None, gl=None, gr=None,
               in_l=None, in_r=None, state="FOCUSED", focused=True, epoch=0, change=0) -> dict:  # type: ignore[no-untyped-def]
    """SYNTHETIC pose record."""
    return {
        "v": 1, "type": "pose", "build": "s4a-test", "sid": sid, "seq": seq, "space": "LOCAL",
        "t_query_ns": t, "t_query_kind": "predicted_display_time", "session_state": state, "focused": focused,
        "local_epoch": epoch, "local_change_time_ns": change,
        "head": head or syn_pose((0.0, 1.6, 0.0), head=True),
        "grip_l": gl or syn_pose((-0.2, 1.0, -0.3)),
        "grip_r": gr or syn_pose((0.2, 1.0, -0.3)),
        "in_l": in_l or syn_inputs(False), "in_r": in_r or syn_inputs(True),
    }  # fmt: skip


def dec(obj: dict) -> qp.PoseRecord | qp.EventRecord:
    return qp.decode_record(json.dumps(obj))


def close(a: list[list[float]], b: list[list[float]], tol: float = 1e-9) -> bool:
    return all(abs(a[i][j] - b[i][j]) <= tol for i in range(4) for j in range(4))


class MatrixConventionTests(unittest.TestCase):
    def test_layout_and_direction(self) -> None:
        # 90 deg about +Y, translation (1,2,3): maps grip-frame points INTO LOCAL.
        q = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
        t = qp.pose_matrix((1.0, 2.0, 3.0), q)
        self.assertEqual([t[0][3], t[1][3], t[2][3]], [1.0, 2.0, 3.0])  # translation = last column
        self.assertEqual(t[3], [0.0, 0.0, 0.0, 1.0])
        x_local = qp.apply(t, (1.0, 0.0, 0.0))  # +X of the grip frame
        self.assertTrue(all(abs(a - b) < 1e-12 for a, b in zip(x_local, (1.0, 2.0, 2.0))))  # R*x -> -Z, +t
        self.assertTrue(close(qp.matmul(t, qp.invert_rigid(t)), qp.pose_matrix((0, 0, 0), (0, 0, 0, 1))))

    def test_stationary_controller_moving_head(self) -> None:
        grip = dec(syn_record(gr=syn_pose((0.3, 0.9, -0.4), (0, S45, 0, C45)))).poses["grip_r"]  # type: ignore[union-attr]
        rels, locals_ = [], []
        for hx, yaw in ((0.0, 0.0), (0.1, 0.3), (-0.2, -0.8)):  # SYNTHETIC head motion
            hq = (0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2))
            rec = dec(syn_record(head=syn_pose((hx, 1.6, 0.05), hq, head=True), gr=syn_pose((0.3, 0.9, -0.4), (0, S45, 0, C45))))
            assert isinstance(rec, qp.PoseRecord)
            locals_.append(rec.poses["grip_r"].matrix())
            rels.append(qp.head_relative(rec.poses["grip_r"], rec.poses["head"]))
        self.assertTrue(all(close(m, locals_[0]) for m in locals_))  # LOCAL pose fixed
        self.assertFalse(close(rels[0], rels[1], 1e-3))  # head-relative pose changes
        self.assertFalse(close(rels[1], rels[2], 1e-3))
        self.assertTrue(close(grip.matrix(), locals_[0]))  # type: ignore[arg-type]


class DecodeTests(unittest.TestCase):
    def test_independent_left_right(self) -> None:
        rec = dec(syn_record(gl=syn_pose((-0.25, 1.0, -0.3)), gr=syn_pose((0.25, 1.1, -0.35))))
        assert isinstance(rec, qp.PoseRecord)
        self.assertEqual(rec.poses["grip_l"].p, (-0.25, 1.0, -0.3))
        self.assertEqual(rec.poses["grip_r"].p, (0.25, 1.1, -0.35))
        self.assertEqual(rec.poses["grip_r"].status, qp.TRACKED)

    def test_buttons_preserved(self) -> None:
        rec = dec(syn_record(in_r=syn_inputs(True, trigger=0.75, squeeze=0.3, a=True, b=False, thumbstick=[0.0, -0.5])))
        assert isinstance(rec, qp.PoseRecord)
        r = rec.inputs["in_r"]
        self.assertEqual((r["trigger"], r["squeeze"], r["a"], r["b"]), (0.75, 0.3, True, False))
        self.assertEqual(r["thumbstick"], (0.0, -0.5))
        self.assertIsNone(r["thumbrest_touch"])  # inactive stays missing, not False/0

    def test_missing_pose_and_missing_fields(self) -> None:
        rec = dec(syn_record(gr=syn_pose(flags=0, active=False)))
        assert isinstance(rec, qp.PoseRecord)
        self.assertEqual(rec.poses["grip_r"].status, qp.MISSING)
        self.assertIsNone(rec.poses["grip_r"].matrix())  # no identity fallback
        self.assertEqual(rec.poses["grip_l"].status, qp.TRACKED)
        obj = syn_record()
        del obj["in_r"]["trigger"]  # inactive must be explicit null; absence is malformed
        with self.assertRaises(qp.RecordError):
            dec(obj)

    def test_invalid_vs_untracked(self) -> None:
        untracked = dec(syn_record(gr=syn_pose((0.2, 1.0, -0.3), flags=qp.POSITION_VALID | qp.ORIENTATION_VALID)))
        invalid = dec(syn_record(gr=syn_pose((0.2, 1.0, -0.3), flags=qp.ORIENTATION_VALID)))
        pos_only_tracked = dec(syn_record(gr=syn_pose((0.2, 1.0, -0.3), flags=ALL & ~qp.ORIENTATION_TRACKED)))
        self.assertEqual(untracked.poses["grip_r"].status, qp.VALID_UNTRACKED)  # type: ignore[union-attr]
        self.assertIsNotNone(untracked.poses["grip_r"].matrix())  # type: ignore[union-attr]
        self.assertEqual(invalid.poses["grip_r"].status, qp.INVALID)  # type: ignore[union-attr]
        self.assertIsNone(invalid.poses["grip_r"].p)  # type: ignore[union-attr]
        self.assertEqual(pos_only_tracked.poses["grip_r"].status, qp.VALID_UNTRACKED)  # type: ignore[union-attr]

    def test_focus_loss(self) -> None:
        rec = dec(syn_record(state="VISIBLE", focused=False, gl=syn_pose(flags=0, active=False),
                             gr=syn_pose(flags=0, active=False)))  # fmt: skip
        assert isinstance(rec, qp.PoseRecord)
        self.assertFalse(rec.focused)
        self.assertEqual({rec.poses[k].status for k in ("grip_l", "grip_r")}, {qp.MISSING})
        self.assertEqual(rec.poses["head"].status, qp.TRACKED)

    def test_flag_consistency_enforced(self) -> None:
        obj = syn_record()
        obj["grip_r"]["pos_tracked"] = False  # disagrees with flags
        with self.assertRaises(qp.RecordError):
            dec(obj)
        obj = syn_record(gr=syn_pose((0.2, 1.0, -0.3), flags=qp.ORIENTATION_VALID))
        obj["grip_r"]["p"] = [0.2, 1.0, -0.3]  # position without POSITION_VALID
        with self.assertRaises(qp.RecordError):
            dec(obj)

    def test_malformed_nonfinite_truncated(self) -> None:
        good = json.dumps(syn_record())
        bad = [
            good[: len(good) // 2],  # truncated
            good.replace('"p": [0.2, 1.0, -0.3]', '"p": [0.2, NaN, -0.3]'),
            good.replace('"p": [0.2, 1.0, -0.3]', '"p": [0.2, 1e999, -0.3]'),
            good.replace('"p": [0.2, 1.0, -0.3]', '"p": [0.2, 1.0]'),
            good.replace('"v": 1', '"v": 2'),
            good.replace('"space": "LOCAL"', '"space": "STAGE"'),
            "not json",
            "{" + '"x":' + '"' + "y" * 3000 + '"}',  # over the size cap
        ]
        self.assertIn('"p": [0.2, 1.0, -0.3]', good)
        for text in bad:
            with self.subTest(text=text[:40]):
                with self.assertRaises(qp.RecordError):
                    qp.decode_record(text)
        obj = syn_record(gr=syn_pose((0.2, 1.0, -0.3), (0, 0, 0, 2.0)))  # non-unit quaternion
        with self.assertRaises(qp.RecordError):
            dec(obj)


class StreamTests(unittest.TestCase):
    def test_session_change_and_duplicates(self) -> None:
        st = qp.StreamState()
        self.assertTrue(st.accept(dec(syn_record(seq=5))))
        self.assertFalse(st.accept(dec(syn_record(seq=5))))  # duplicate
        self.assertFalse(st.accept(dec(syn_record(seq=4))))  # out of order
        self.assertTrue(st.accept(dec(syn_record(seq=1, sid="1111111111111111"))))  # new app run resets
        self.assertEqual((st.sessions, st.rejected, st.last_seq), (2, 2, 1))

    def test_reference_space_change_epochs(self) -> None:
        st = qp.StreamState()
        st.accept(dec(syn_record(seq=1, t=1000)))
        ev = {"v": 1, "type": "event", "build": "b", "sid": "00000000deadbeef", "seq": 2,
              "event": "local_space_change_pending", "space": "LOCAL", "local_epoch": 1, "time_ns": 5000,
              "pose_valid": True, "pose_in_previous_space": {"p": [0, 0, 0], "q": [0, 0, 0, 1]}}  # fmt: skip
        self.assertTrue(st.accept(dec(ev)))
        before = dec(syn_record(seq=3, t=4000, epoch=1, change=5000))  # queried before changeTime
        after = dec(syn_record(seq=4, t=6000, epoch=1, change=5000))
        st.accept(before)
        st.accept(after)
        self.assertEqual(st.origin_epoch(before), 0)  # type: ignore[arg-type]
        self.assertEqual(st.origin_epoch(after), 1)  # type: ignore[arg-type]
        lost = dict(ev, seq=5, event="events_lost", time_ns=0)
        del lost["pose_valid"], lost["pose_in_previous_space"]
        st.accept(dec(lost))
        self.assertTrue(st.events_lost)


@unittest.skipUnless(shutil.which("g++") and (APP_SRC / "QuestXarmTelemetry.h").exists(), "needs g++ and the app source")
class FormatterRoundTripTests(unittest.TestCase):
    """The app's real C++ formatter, compiled for the host (SYNTHETIC inputs)."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.tmp.name) / "harness"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(APP_SRC), str(HARNESS),
                        "-o", str(cls.exe)], check=True)  # fmt: skip

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def run_sc(self, sc: str) -> str:
        return subprocess.run([str(self.exe), sc], check=True, capture_output=True, text=True).stdout.strip()

    def test_full_record(self) -> None:
        text = self.run_sc("full")
        self.assertLess(len(text), qp.MAX_RECORD_BYTES)
        rec = qp.decode_record(text)
        assert isinstance(rec, qp.PoseRecord)
        self.assertEqual(rec.poses["grip_r"].status, qp.TRACKED)
        self.assertEqual(rec.poses["grip_r"].p, (0.2, 1.0, -0.3))
        self.assertEqual((rec.inputs["in_r"]["trigger"], rec.inputs["in_r"]["squeeze"]), (0.75, 0.3))
        self.assertEqual((rec.inputs["in_r"]["a"], rec.inputs["in_r"]["b"]), (True, False))
        self.assertIsNone(rec.inputs["in_l"]["x"])  # never set active -> null

    def test_scenarios(self) -> None:
        r = qp.decode_record(self.run_sc("right_missing"))
        self.assertEqual(r.poses["grip_r"].status, qp.MISSING)  # type: ignore[union-attr]
        self.assertIsNone(r.inputs["in_r"]["trigger"])  # type: ignore[union-attr]
        u = qp.decode_record(self.run_sc("untracked"))
        self.assertEqual(u.poses["grip_r"].status, qp.VALID_UNTRACKED)  # type: ignore[union-attr]
        f = qp.decode_record(self.run_sc("unfocused"))
        self.assertFalse(f.focused)  # type: ignore[union-attr]
        with self.assertRaises(qp.RecordError):  # NaN is reported, never emitted as a number
            qp.decode_record(self.run_sc("nonfinite"))
        self.assertNotIn("nan", self.run_sc("nonfinite").lower().replace("nonfinite", ""))
        ov = qp.decode_record(self.run_sc("overflow"))  # over-cap -> intact overflow record only
        assert isinstance(ov, qp.OverflowRecord)
        self.assertEqual((ov.seq, ov.cap_bytes), (8, qp.MAX_RECORD_BYTES))
        self.assertGreater(ov.attempted_bytes or 0, qp.MAX_RECORD_BYTES)
        e = qp.decode_record(self.run_sc("event_space"))
        assert isinstance(e, qp.EventRecord)
        self.assertEqual((e.event, e.time_ns, e.pose_valid), ("local_space_change_pending", 5000, True))


@unittest.skipUnless(shutil.which("g++") and (APP_SRC / "QuestXarmTelemetry.h").exists(), "needs g++ and the app source")
class LoggingBoundaryTests(unittest.TestCase):
    """SYNTHETIC: the app's real emission gate (prepare_log_message) and probe builder, compiled
    for the host. These do NOT verify Android log delivery (see the on-device probe test)."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.tmp.name) / "harness"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(APP_SRC), str(HARNESS),
                        "-o", str(cls.exe)], check=True)  # fmt: skip

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def h(self, *args: str) -> str:
        return subprocess.run([str(self.exe), *args], check=True, capture_output=True, text=True).stdout.rstrip("\n")

    def test_full_pose_above_old_cutoff_is_complete(self) -> None:
        for sc in ("full", "right_missing", "worst"):
            text = self.h(sc)
            n = len(text.encode())
            self.assertGreater(n, qp.LEGACY_TRUNCATION_BYTES, sc)  # would have been cut by __android_log_print
            self.assertLessEqual(n, qp.MAX_RECORD_BYTES, sc)
            rec = qp.decode_record(text)
            assert isinstance(rec, qp.PoseRecord)
            self.assertIn("b", rec.inputs["in_r"])  # trailing input fields present
            self.assertTrue(text.endswith("}}"))

    def test_probe_sizes_exact_and_at_cap(self) -> None:
        for pid, n, utf8 in ((1, 1023, 0), (2, 1024, 0), (3, 1500, 0), (4, 1500, 1), (5, qp.MAX_RECORD_BYTES, 0)):
            text = self.h("probe", str(pid), str(n), str(utf8))
            self.assertEqual(len(text.encode("utf-8")), n)
            self.assertEqual(text, qp.expected_probe("s4a-test", "00000000deadbeef", 100 + pid, pid, n, bool(utf8)))
            rec = qp.decode_record(text)
            assert isinstance(rec, qp.ProbeRecord)
            self.assertTrue(text.endswith(f'"end":"QXEND-{pid}"}}'))

    def test_utf8_byte_accounting(self) -> None:
        text = self.h("probe", "4", "1500", "1")
        self.assertEqual(len(text.encode("utf-8")), 1500)
        self.assertLess(len(text), 1500)  # characters != bytes
        over = qp.expected_probe("s4a-test", "00000000deadbeef", 1, 9, 3001, True)
        self.assertLessEqual(len(over), qp.MAX_RECORD_BYTES)  # under the cap in characters...
        with self.assertRaises(qp.RecordError):  # ...but over it in bytes
            qp.decode_record(over)

    def test_over_cap_emits_only_intact_overflow(self) -> None:
        for n in (qp.MAX_RECORD_BYTES + 1, 3500):
            text = self.h("probe", "6", str(n), "0")
            self.assertLess(len(text), 200)
            self.assertNotIn("pad", text)  # no portion of the dropped record
            rec = qp.decode_record(text)
            assert isinstance(rec, qp.OverflowRecord)
            self.assertEqual((rec.seq, rec.attempted_bytes, rec.cap_bytes), (106, n, qp.MAX_RECORD_BYTES))


class RecorderInputTests(unittest.TestCase):
    """SYNTHETIC lines through the Step 4B recorder state machine."""

    LINE = "         1790800025.520487 19405 19437 I QuestXarmPoseV1: "

    def setUp(self) -> None:
        import io

        import quest_pose_v1_record as rr

        self.rr = rr
        self.rec = rr.Recorder(io.StringIO(), 500)

    def test_overflow_clears_input_without_refreshing(self) -> None:
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=1, in_r=syn_inputs(True, squeeze=1.0))), 0)
        self.assertTrue(self.rec.status_line(1_000_000, True, "", 500).startswith("R=FRESH_TRACKED"))
        ov = {"v": 1, "type": "overflow", "build": "b", "sid": "00000000deadbeef", "seq": 2,
              "attempted_bytes": 3100, "cap_bytes": 3000}  # fmt: skip
        self.rec.on_line(self.LINE + json.dumps(ov), 100_000_000)
        self.assertIsNone(self.rec.latest)
        self.assertEqual(self.rec.last_pose_ns, 0)  # input freshness NOT refreshed by overflow
        self.assertTrue(self.rec.status_line(110_000_000, True, "", 500).startswith("R=OVERFLOW"))
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=3, in_r=syn_inputs(True, squeeze=1.0))), 200_000_000)
        self.assertTrue(self.rec.status_line(210_000_000, True, "", 500).startswith("R=FRESH_TRACKED"))

    def test_probe_never_counts_as_input(self) -> None:
        probe = qp.expected_probe("b", "00000000deadbeef", 1, 1, 1500, False)
        self.rec.on_line(self.LINE + probe, 0)
        self.assertEqual((self.rec.n_probe, self.rec.n_pose), (1, 0))
        self.assertIsNone(self.rec.latest)
        self.assertTrue(self.rec.status_line(1_000_000, True, "", 500).startswith("R=MISSING"))
        self.assertTrue(self.rec.probes[0]["exact_match"])

    def test_grip_unavailable_blocks_availability(self) -> None:
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=1, in_r=syn_inputs(True, squeeze=None))), 0)
        self.assertTrue(self.rec.status_line(1_000_000, True, "", 500).startswith("R=GRIP_UNAVAILABLE"))
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=2, in_r=syn_inputs(True, squeeze=1.0))), 10_000_000)
        self.assertTrue(self.rec.status_line(11_000_000, True, "", 500).startswith("R=FRESH_TRACKED"))

    def test_focus_loss_event_invalidates_on_processing(self) -> None:
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=1, in_r=syn_inputs(True, squeeze=1.0))), 0)
        ev = {"v": 1, "type": "event", "build": "b", "sid": "00000000deadbeef", "seq": 2, "event": "session_state",
              "space": "LOCAL", "local_epoch": 0, "session_state": "VISIBLE", "time_ns": 1}  # fmt: skip
        self.rec.on_line(self.LINE + json.dumps(ev), 5_000_000)
        last = json.loads(self.rec.out.getvalue().splitlines()[-1])  # type: ignore[attr-defined]
        self.assertEqual(last["status_after"], "UNFOCUSED")  # at processing time, before any refresh
        self.assertIsNone(self.rec.latest)
        line = self.rec.status_line(6_000_000, True, "", 500)
        self.assertTrue(line.startswith("R=UNFOCUSED"))
        self.assertNotIn("sqz=", line)  # no stale grip/pose shown as current
        ev2 = dict(ev, seq=3, session_state="FOCUSED")  # a FOCUSED event alone does not restore input
        self.rec.on_line(self.LINE + json.dumps(ev2), 7_000_000)
        self.assertTrue(self.rec.status_line(8_000_000, True, "", 500).startswith("R=UNFOCUSED"))
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=4, in_r=syn_inputs(True, squeeze=1.0))), 9_000_000)
        self.assertTrue(self.rec.status_line(10_000_000, True, "", 500).startswith("R=FRESH_TRACKED"))

    def test_unfocused_pose_and_unknown_state_never_available(self) -> None:
        for i, (state, foc) in enumerate((("VISIBLE", False), ("UNKNOWN", False), ("FOCUSED", False))):
            self.rec.on_line(self.LINE + json.dumps(syn_record(seq=10 + i, state=state, focused=foc,
                                                               in_r=syn_inputs(True, squeeze=1.0))), i * 1_000_000)  # fmt: skip
            self.assertTrue(self.rec.status_line(i * 1_000_000 + 1, True, "", 500).startswith("R=UNFOCUSED"))

    def test_untracked_or_stale_or_disconnected_not_available(self) -> None:
        gr = syn_pose((0.2, 1.0, -0.3), flags=qp.POSITION_VALID | qp.ORIENTATION_VALID | qp.ORIENTATION_TRACKED)
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=1, gr=gr, in_r=syn_inputs(True, squeeze=1.0))), 0)
        self.assertTrue(self.rec.status_line(1_000_000, True, "", 500).startswith("R=VALID_UNTRACKED"))
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=2, in_r=syn_inputs(True, squeeze=1.0))), 10_000_000)
        self.assertTrue(self.rec.status_line(600_000_000, True, "", 500).startswith("R=STALE"))  # watchdog
        self.assertTrue(self.rec.status_line(20_000_000, False, "app not running", 500).startswith("R=DISCONNECTED"))

    def test_single_tracking_flag_clear_makes_right_unavailable(self) -> None:
        """SYNTHETIC: focused, action active, finite pose, grip 1.0, LEFT missing; only one tracked bit cleared."""
        left_missing = syn_pose(flags=0, active=False)
        cases = {"position untracked": ALL & ~qp.POSITION_TRACKED, "orientation untracked": ALL & ~qp.ORIENTATION_TRACKED}
        for i, (name, flags) in enumerate(cases.items()):
            with self.subTest(name):
                gr = syn_pose((0.2, 1.0, -0.3), flags=flags)
                obj = syn_record(seq=1 + 2 * i, gr=gr, gl=left_missing, in_r=syn_inputs(True, squeeze=1.0))
                rec = dec(obj)
                assert isinstance(rec, qp.PoseRecord)
                self.assertTrue(rec.focused and rec.poses["grip_r"].active and rec.poses["grip_r"].matrix() is not None)
                self.rec.on_line(self.LINE + json.dumps(obj), i * 10_000_000)
                line = self.rec.status_line(i * 10_000_000 + 1, True, "", 500)
                self.assertTrue(line.startswith("R=VALID_UNTRACKED"), line)
                self.assertNotIn("sqz=", line)  # values hidden while unavailable
                ok = syn_record(seq=2 + 2 * i, gl=left_missing, in_r=syn_inputs(True, squeeze=1.0))
                self.rec.on_line(self.LINE + json.dumps(ok), i * 10_000_000 + 5_000_000)
                self.assertTrue(self.rec.status_line(i * 10_000_000 + 5_000_001, True, "", 500).startswith("R=FRESH_TRACKED"))

    def test_events_keep_stream_fresh_but_not_input(self) -> None:
        self.rec.on_line(self.LINE + json.dumps(syn_record(seq=1)), 0)
        ev = {"v": 1, "type": "event", "build": "b", "sid": "00000000deadbeef", "seq": 2, "event": "session_state",
              "space": "LOCAL", "local_epoch": 0, "session_state": "FOCUSED", "time_ns": 1}  # fmt: skip
        self.rec.on_line(self.LINE + json.dumps(ev), 700_000_000)
        self.assertTrue(self.rec.status_line(800_000_000, True, "", 500).startswith("R=INPUT_STALE"))


if __name__ == "__main__":
    unittest.main()
