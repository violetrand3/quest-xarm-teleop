"""Tests for the COMMANDING arm link and the first-movement test program.

NO ROBOT IS INVOLVED. The real SDK code talks to a SYNTHETIC controller on loopback that records
every frame. The synthetic controller's motion behaviour is an assumption; these tests show what
OUR program sends and how it reacts, not how the real arm behaves.
"""
from __future__ import annotations

import io
import json
import math
import os
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "tests" / "fixtures"))

import xarm_readonly_preflight as rp  # noqa: E402
from fake_xarm_controller import POSE, FakeController  # noqa: E402
from test_xarm_readonly_preflight import LoopbackSdkTests, _sdk_available  # noqa: E402

READS = set(rp.READ_REGISTERS)
_DEVNULL = open(os.devnull, "w")  # kept open: the SDK's logger holds on to the stream it first sees
NEAR_FENCE = [257.36, -290.89, 495.78, math.radians(-179.42), math.radians(-32.62), math.radians(-47.73)]  # like the real arm today


def writes(ctl: FakeController) -> list:
    return [(reg, p) for _, reg, p in ctl.frames if reg not in READS]


@unittest.skipUnless(_sdk_available(), "xArm SDK not importable here (set XARM_SDK_SRC to a copy of its source)")
class CommandLinkTests(unittest.TestCase):
    def setUp(self) -> None:
        import xarm_command_link as cl
        self.cl = cl
        self.real_stdout = sys.stdout
        sys.stdout = _DEVNULL  # the SDK prints
        self.addCleanup(self._restore)

    def _restore(self) -> None:
        sys.stdout = self.real_stdout

    def link(self, ctl: FakeController, box=((250, -50, 400), (350, 50, 520)), speed=20.0, acc=200.0):  # type: ignore[no-untyped-def]
        undo = LoopbackSdkTests.remap(ctl)
        api, cmd, _ = self.cl.load_sdk()
        L = self.cl.XArmCommandLink(api, cmd, "127.0.0.1", box[0], box[1], speed, acc)

        def cleanup() -> None:
            L.close()
            undo()
            time.sleep(0.05)
            ctl.close()

        self.addCleanup(cleanup)
        L.connect()
        return L

    def assert_only_allowed_frames(self, ctl: FakeController) -> None:
        for prot, reg, p in ctl.frames:
            self.assertEqual(prot, 2)
            if reg in READS:
                self.assertEqual(p, b"")
            elif reg == 11:
                self.assertEqual(p, bytes([8, 1]))
            elif reg == 19:
                self.assertIn(p, (bytes([0, 0]), bytes([7, 0]), bytes([0]), bytes([7])))
            elif reg == 12:
                self.assertIn(p, (bytes([0]), bytes([4])))
            elif reg == 21:
                self.assertEqual((len(p), p[40:]), (43, bytes([0, 0, 0])))
            else:
                self.fail(f"unexpected register {reg} reached the controller")

    def test_connect_sends_only_reads(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        s = L.read()
        self.assertEqual((s.state, s.error, s.warn, s.cmdnum), (4, 0, 0, 0))
        self.assertAlmostEqual(s.pose[0], POSE[0], places=3)
        self.assertAlmostEqual(s.pose[3], POSE[3], places=5)  # radians, as reported
        self.assertEqual(writes(ctl), [])
        self.assertIn("2.5.5", L.version)

    def test_arming_sends_enable_mode_state_in_order_and_nothing_else(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        s = L.arm_motion(7)
        self.assertEqual([(r, p) for r, p in writes(ctl)], [(11, bytes([8, 1])), (19, bytes([7, 0])), (12, bytes([0]))])
        self.assertEqual((s.state, L.reported_mode(), L.armed_mode, ctl.mode), (2, 7, 7, 7))
        with self.assertRaises(self.cl.LinkError):
            L.arm_motion(1)  # servo mode cannot be selected
        self.assert_only_allowed_frames(ctl)

    def test_move_frame_is_byte_identical_to_the_sdk_set_position_frame(self) -> None:
        tgt = (300.0, 13.5, 460.0, POSE[3], POSE[4], POSE[5])
        ctl = FakeController()
        L = self.link(ctl)
        L.arm_motion(7)
        L.move_line(tgt, 12.0, 150.0)
        ours = [p for r, p in writes(ctl) if r == 21][0]
        # the SDK's own high-level call, unguarded, against a second synthetic controller
        ctl2 = FakeController()
        undo = LoopbackSdkTests.remap(ctl2)
        api, _, _ = self.cl.load_sdk()
        try:
            arm = api("127.0.0.1", is_radian=True, timed_comm=False, forbid_uds=True)
            arm.motion_enable(True); arm.set_mode(7); arm.set_state(0)  # noqa: E702
            time.sleep(0.3)
            code = arm.set_position(*tgt, speed=12.0, mvacc=150.0, wait=False)
            arm.disconnect()
        finally:
            undo()
            time.sleep(0.05)
            ctl2.close()
        sdk = [p for _, r, p in ctl2.frames if r == 21]
        self.assertEqual(code, 0)
        self.assertEqual(ours, sdk[0])
        self.assertEqual(struct.unpack("<10f", ours[:40])[6:], (12.0, 150.0, 0.0, -1.0))

    def test_move_follows_and_a_new_target_replaces_the_old_one_in_the_model(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        s = L.arm_motion(7)
        L.move_line((s.pose[0], s.pose[1] + 10.0) + tuple(s.pose[2:]), 20.0, 200.0)
        time.sleep(0.25)
        L.move_line(s.pose, 20.0, 200.0)  # back to the start before the first target is reached
        ys = []
        for _ in range(30):
            ys.append(L.read().pose[1])
            time.sleep(0.03)
        self.assertLess(max(ys) - s.pose[1], 9.0)
        self.assertAlmostEqual(ys[-1], s.pose[1], delta=0.05)
        self.assert_only_allowed_frames(ctl)

    def test_targets_outside_the_hard_box_or_over_the_limits_never_reach_the_wire(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        s = L.arm_motion(7)
        ok = tuple(s.pose)
        for pose, speed, acc in (((ok[0], 60.0) + ok[2:], 10.0, 100.0),  # outside the box
                                 ((float("nan"),) + ok[1:], 10.0, 100.0),
                                 (ok, 25.0, 100.0),  # over the speed limit
                                 (ok, 10.0, 900.0)):  # over the acceleration limit
            L.armed_mode = 7
            with self.assertRaises(self.cl.LinkError):
                L.move_line(pose, speed, acc)
            self.assertIsNone(L.armed_mode)  # a refused move disarms
        self.assertEqual([r for r, _ in writes(ctl)].count(21), 0)
        self.assertEqual(len(L.ledger.summary()["wire_frames_refused"]), 4)

    def test_unarmed_and_after_halt_nothing_moves(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        with self.assertRaises(self.cl.LinkError):
            L.move_line(tuple(POSE), 10.0, 100.0)
        s = L.arm_motion(7)
        L.move_line((s.pose[0], s.pose[1] + 10.0) + tuple(s.pose[2:]), 20.0, 200.0)
        time.sleep(0.15)
        h = L.halt()
        y = L.read().pose[1]
        time.sleep(0.3)
        self.assertEqual((h.state, L.armed_mode), (4, None))
        self.assertAlmostEqual(L.read().pose[1], y, delta=0.01)  # the model stops; the real arm must be measured
        with self.assertRaises(self.cl.LinkError):
            L.move_line(tuple(s.pose), 10.0, 100.0)  # needs an explicit re-arm
        self.assertEqual([r for r, _ in writes(ctl)].count(21), 1)

    def test_errors_and_warnings_are_never_cleared_and_block_arming(self) -> None:
        for kw in ({"warn": 11}, {"err": 22}):
            ctl = FakeController(**kw)
            L = self.link(ctl)
            with self.assertRaises(self.cl.LinkError):
                L.arm_motion(7)
            self.assertEqual(writes(ctl), [], kw)  # no enable, no clean (16/17), nothing
            self.assertEqual(L.core.clean_error(), -99)
            self.assertEqual(L.core.clean_warn(), -99)
            L.core.arm_cmd.clean_err()  # even the low-level call is stopped at the wire
            L.core.arm_cmd.clean_war()
            self.assertEqual(writes(ctl), [], kw)
            self.assertEqual({w["register"] for w in L.ledger.summary()["wire_frames_refused"]}, {16, 17})

    def test_every_other_sdk_command_is_refused(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        L.arm_motion(7)
        n = len(writes(ctl))
        c = L.core
        for call in (lambda: c.set_servo_angle(angle=[0] * 6), lambda: c.set_servo_cartesian(list(POSE)), lambda: c.emergency_stop(),
                     lambda: c.set_tcp_offset([0] * 6), lambda: c.set_reduced_mode(False), lambda: c.set_fence_mode(False),
                     lambda: c.set_gripper_position(100), lambda: c.reset(), lambda: c.move_gohome(), lambda: c.set_position(x=300)):  # fmt: skip
            self.assertEqual(call(), -99)
        c.arm_cmd.set_state(3)  # pause: not in the write rules
        c.arm_cmd.set_mode(1)  # servo mode: not in the write rules
        c.arm_cmd.motion_en(8, 0)  # disable: not in the write rules
        self.assertEqual(len(writes(ctl)), n)
        self.assert_only_allowed_frames(ctl)

    def test_controller_side_boundary_error_is_reported_not_cleared(self) -> None:
        ctl = FakeController()
        L = self.link(ctl, box=((100, -400, 200), (600, 400, 700)))  # a (wrong) box wider than the controller's fence
        s = L.arm_motion(7)
        with self.assertRaises(self.cl.LinkError):
            L.move_line((s.pose[0], 290.0) + tuple(s.pose[2:]), 20.0, 200.0)  # beyond the fence y-max 270
        st = L.read()
        self.assertEqual((st.error, st.state, L.armed_mode), (35, 4, None))
        with self.assertRaises(self.cl.LinkError):
            L.arm_motion(7)
        self.assertNotIn(16, [r for r, _ in writes(ctl)])

    def test_park_leaves_mode_0_and_state_4(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        L.arm_motion(7)
        p = L.park()
        self.assertEqual((p.state, ctl.mode, ctl.state, L.armed_mode), (4, 0, 4, None))

    def test_silent_controller_fails_within_the_short_timeout(self) -> None:
        ctl = FakeController()
        L = self.link(ctl)
        s = L.arm_motion(7)
        ctl.silent = True
        t0 = time.monotonic()
        with self.assertRaises(self.cl.LinkError):
            L.move_line(tuple(s.pose), 10.0, 100.0)
        self.assertLess(time.monotonic() - t0, 1.5)
        self.assertIsNone(L.armed_mode)


@unittest.skipUnless(_sdk_available(), "xArm SDK not importable here (set XARM_SDK_SRC to a copy of its source)")
class FirstMoveProgramTests(unittest.TestCase):
    def run_program(self, ctl: FakeController, extra: list | None = None, during=None):  # type: ignore[no-untyped-def]
        import arm_first_move_test as fm
        import xarm_command_link as cl
        undo = LoopbackSdkTests.remap(ctl)
        real, buf = sys.stdout, io.StringIO()
        sys.stdout = buf
        try:
            with tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp) / "o.json"
                if during is not None:
                    threading.Thread(target=during, daemon=True).start()
                rc = fm.main(["--address", "127.0.0.1", "--moves-real-arm", "--human-confirmation", "SYNTHETIC controller on loopback",
                              "--distance", "6", "--speed", "20", "--acc", "300", "--out", str(out)] + (extra or []), sdk=cl.load_sdk())  # fmt: skip
                res = json.loads(out.read_text()) if out.exists() else None
        finally:
            sys.stdout = real
            undo()
            time.sleep(0.05)
            ctl.close()
        return rc, res, buf.getvalue()

    def test_plan_only_without_the_flag_makes_no_connection(self) -> None:
        import arm_first_move_test as fm
        ctl = FakeController(pose=NEAR_FENCE)
        undo = LoopbackSdkTests.remap(ctl)
        real, sys.stdout = sys.stdout, io.StringIO()
        try:
            self.assertEqual(fm.main(["--address", "127.0.0.1"]), 0)
            self.assertEqual(fm.main(["--address", "127.0.0.1", "--moves-real-arm"]), 2)  # no confirmation
            self.assertEqual(fm.main(["--address", "127.0.0.1", "--distance", "50"]), 2)
        finally:
            sys.stdout = real
            undo()
            ctl.close()
        self.assertEqual(ctl.frames, [])

    def test_plan_moves_away_from_the_nearest_fence_wall(self) -> None:
        import arm_first_move_test as fm
        b = rp.decode_boundary([502, 180, 270, -300, 585, 270])
        p = fm.plan(tuple(NEAR_FENCE), b, "y", 10.0)
        self.assertEqual((p["sign"], p["end_coordinate_mm"]), (1.0, -280.89))  # 9 mm from y-min: go +y
        self.assertEqual(fm.plan(tuple(NEAR_FENCE), b, "z", 10.0)["sign"], -1.0)  # nearer the z-max wall
        with self.assertRaises(fm.Abort):
            fm.plan((182.0, 0.0, 400.0, 0, 0, 0), b, "x", 2.0)  # would end inside the 5 mm fence margin

    def test_full_run_classifies_replace_and_stops_and_parks(self) -> None:
        ctl = FakeController(pose=NEAR_FENCE)
        rc, res, text = self.run_program(ctl)
        self.assertEqual((rc, res["result"]), (0, "ok"), res.get("abort_reason"))
        t = res["tests"]
        self.assertTrue(t["A_single_move"]["reached"])
        self.assertTrue(t["B_second_target"]["verdict"].startswith("REPLACED"))
        self.assertLess(t["B_second_target"]["peak_travel_mm"], 5.5)
        self.assertEqual((t["C_stop_during_move"]["state_after"], t["C_stop_during_move"]["resumed_by_itself"]), (4, False))
        self.assertLess(t["C_stop_during_move"]["distance_after_request_mm"], 1.0)
        self.assertTrue(t["returned_to_start"])
        self.assertEqual((res["after"]["state"], ctl.mode, ctl.state), (4, 0, 4))
        self.assertLess(res["after"]["distance_from_start_mm"], 0.1)
        self.assertEqual(res["plan"]["sign"], 1.0)
        ys = [struct.unpack("<10f", p[:40])[1] for _, r, p in ctl.frames if r == 21]
        self.assertTrue(all(-290.9 <= y <= -284.88 for y in ys), ys)  # every target within 6 mm, toward +y
        for _, reg, p in ctl.frames:
            self.assertIn(reg, READS | {11, 12, 19, 21})
        self.assertEqual(res["ledger"]["api_calls_refused"], {})
        self.assertNotIn("127.0.0.1", json.dumps(res))  # address redacted in the saved file

    def test_queueing_controller_is_classified_as_queued(self) -> None:
        ctl = FakeController(pose=NEAR_FENCE, mode7_replaces=False)
        rc, res, _ = self.run_program(ctl)
        self.assertEqual(rc, 0, res.get("abort_reason"))
        self.assertTrue(res["tests"]["B_second_target"]["verdict"].startswith("QUEUED"))
        self.assertGreaterEqual(res["tests"]["B_second_target"]["peak_travel_mm"], 5.5)

    def test_reduced_mode_off_is_refused_unless_waived_for_position_mode(self) -> None:
        for extra, expect_run in (([], False), (["--mode", "7", "--allow-reduced-mode-off"], False), (["--mode", "0", "--allow-reduced-mode-off"], True)):
            ctl = FakeController(pose=NEAR_FENCE)
            ctl.reduced_on = False
            rc, res, _ = self.run_program(ctl, extra)
            moved = [r for _, r, _ in ctl.frames if r not in READS]
            if expect_run:
                self.assertEqual((rc, res["result"]), (0, "ok"), res.get("abort_reason"))
                self.assertTrue(res["before"]["reduced_mode_waived_by_operator"])
                self.assertTrue(res["tests"]["B_second_target"]["verdict"].startswith("QUEUED"))  # the model queues in mode 0
                self.assertEqual((ctl.mode, ctl.state), (0, 4))
            else:
                self.assertEqual((rc, res["result"], moved), (3, "aborted", []), extra)
                self.assertIn("reduced mode is not on", res["abort_reason"])

    def test_simulation_mode_commands_nothing(self) -> None:
        ctl = FakeController(pose=NEAR_FENCE, sim=True)  # the controller would report motion while the real arm stays still
        rc, res, _ = self.run_program(ctl)
        self.assertEqual((rc, res["result"]), (3, "aborted"))
        self.assertIn("SIMULATION", res["abort_reason"])
        self.assertEqual([r for _, r, _ in ctl.frames if r not in READS], [])

    def test_orientation_check_is_wrap_safe_near_180_degrees(self) -> None:
        import arm_first_move_test as fm
        import xarm_command_link as cl
        t = fm.FirstMove(None, {"k": 2, "sign": 1.0}, 10.0, 10.0, 100.0, 0, [])  # type: ignore[arg-type]
        t.start = (300.0, 0.0, 400.0, math.radians(-179.42), 0.0, 0.0)
        ok = cl.ArmState((300.0, 0.0, 405.0, math.radians(179.97), 0.0, 0.0), 1, 0, 0, 0, 0, 0.0)  # roll wrapped: a 0.61 deg change
        t.check(ok)
        with self.assertRaises(fm.Abort):
            t.check(cl.ArmState((300.0, 0.0, 405.0, math.radians(175.0), 0.0, 0.0), 1, 0, 0, 0, 0, 0.0))  # a real 5.6 deg change

    def test_error_or_busy_arm_at_start_commands_nothing(self) -> None:
        for kw in ({"err": 22}, {"warn": 11}):
            ctl = FakeController(pose=NEAR_FENCE, **kw)
            rc, res, _ = self.run_program(ctl)
            self.assertEqual((rc, res["result"]), (3, "aborted"))
            self.assertEqual([r for _, r, _ in ctl.frames if r not in READS], [], kw)

    def test_error_during_the_run_aborts_and_parks_without_clearing(self) -> None:
        ctl = FakeController(pose=NEAR_FENCE)

        def inject() -> None:
            time.sleep(1.6)
            ctl.err = 31  # SYNTHETIC collision-type error mid-run
            with ctl.lock:
                ctl.state, ctl.target, ctl.queue = 4, None, []

        rc, res, _ = self.run_program(ctl, during=inject)
        self.assertEqual((rc, res["result"]), (3, "aborted"))
        regs = [r for _, r, _ in ctl.frames]
        self.assertNotIn(16, regs)
        self.assertNotIn(17, regs)
        self.assertEqual((ctl.state, ctl.mode), (4, 0))  # parked
        self.assertEqual(res["after"]["error"], 31)

    def test_controller_going_silent_aborts_and_says_to_use_the_physical_stop(self) -> None:
        ctl = FakeController(pose=NEAR_FENCE)
        rc, res, text = self.run_program(ctl, during=lambda: (time.sleep(2.0), setattr(ctl, "silent", True)))
        self.assertEqual((rc, res["result"]), (3, "aborted"))
        self.assertIn("park_error", res)
        self.assertIn("USE THE PHYSICAL STOP", text)


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(_sdk_available(), "xArm SDK not importable here (set XARM_SDK_SRC to a copy of its source)")
class LiveBackendWithAdapterTests(unittest.TestCase):
    """The teleop adapter driving the REAL backend class against the SYNTHETIC controller, in real time."""

    FENCE_WS = {"limits__workspace_min_m": [0.19, -0.29, 0.28], "limits__workspace_max_m": [0.49, 0.26, 0.575]}

    def setUp(self) -> None:
        import teleop_adapter as ta
        import xarm_command_link as cl
        import xarm_live_backend as lb
        from test_teleop_adapter import P0, Rig, cfg, off
        self.ta, self.cl, self.P0, self.off = ta, cl, P0, off
        self.real_stdout, sys.stdout = sys.stdout, _DEVNULL
        self.ctl = FakeController()
        undo = LoopbackSdkTests.remap(self.ctl)
        api, cmd, _ = cl.load_sdk()
        self.link = cl.XArmCommandLink(api, cmd, "127.0.0.1", (280, -20, 440), (330, 30, 480), 60.0, 500.0)
        self.link.connect()
        self.r = Rig(cfg(**self.FENCE_WS))
        self.backend = lb.XArmLiveBackend(self.link, speed_mm_s=60.0, acc_mm_s2=500.0)
        self.r.ad.backend = self.backend

        def cleanup() -> None:
            self.link.close()
            undo()
            time.sleep(0.05)
            self.ctl.close()
            sys.stdout = self.real_stdout

        self.addCleanup(cleanup)

    def step(self, p=None, grip=1.0, **kw):  # type: ignore[no-untyped-def]
        time.sleep(0.02)  # the synthetic arm moves in real time
        return self.r.step(self.P0 if p is None else p, grip, **kw)

    def moves(self) -> list:
        return [struct.unpack("<10f", p[:40]) for _, reg, p in self.ctl.frames if reg == 21]

    def test_engagement_is_refused_until_the_operator_arms(self) -> None:
        self.step(grip=0.0)
        out = self.step(grip=1.0)
        self.assertEqual(out["state"], self.ta.WAIT_RELEASE)
        self.assertIn("arm not armed", out["blocked"])
        self.assertEqual(writes(self.ctl), [])

    def test_arm_follows_hand_with_orientation_passed_back_unchanged(self) -> None:
        self.backend.arm_motion()
        self.step(grip=0.0)
        out = self.step(grip=1.0)
        self.assertEqual(out["state"], self.ta.ACTIVE)
        first = self.moves()[0]
        for k in range(3):
            self.assertAlmostEqual(first[k], POSE[k], places=3)  # first target = where the arm is: no jump
        for i in range(1, 26):  # hand 50 mm along LOCAL -z in 0.5 s -> tool +10 mm in x (example mapping, scale 0.2)
            out = self.step(self.off(self.P0, (0.0, 0.0, -0.002 * i)))
        for _ in range(25):
            out = self.step(self.off(self.P0, (0.0, 0.0, -0.05)))
        self.assertEqual(out["state"], self.ta.ACTIVE, self.r.rec.faults)
        self.assertAlmostEqual(self.ctl.pose[0], POSE[0] + 10.0, delta=0.3)
        self.assertAlmostEqual(self.ctl.pose[1], POSE[1], delta=0.05)
        m = self.moves()
        self.assertEqual({v[3:6] for v in m}, {first[3:6]})  # roll, pitch, yaw bit-identical on every target
        self.assertTrue(all(v[6:] == (60.0, 500.0, 0.0, -1.0) for v in m))
        self.assertEqual([r for r, _ in writes(self.ctl) if r not in (21,)], [11, 19, 12])  # arming only; nothing else

    def test_clutch_release_sends_no_stop_and_reengages_without_rearming(self) -> None:
        self.backend.arm_motion()
        self.step(grip=0.0)
        self.step(grip=1.0)
        for i in range(1, 11):
            self.step(self.off(self.P0, (0.0, 0.0, -0.002 * i)))
        n = len(self.moves())
        out = self.step(self.off(self.P0, (0.0, 0.0, -0.02)), grip=0.0)
        self.assertNotEqual(out["state"], self.ta.ACTIVE)
        for _ in range(10):
            self.step(self.off(self.P0, (0.0, 0.0, -0.02)), grip=0.0)
        self.assertEqual(len(self.moves()), n)  # nothing after the release
        self.assertEqual([p for r, p in writes(self.ctl) if r == 12], [bytes([0])])  # no state 4 on an ordinary release
        self.assertEqual((self.backend.stop_count, self.backend.halt_count), (1, 0))
        out = self.step(self.off(self.P0, (0.0, 0.0, -0.02)), grip=1.0)
        self.assertEqual(out["state"], self.ta.ACTIVE)

    def test_fault_halts_and_needs_acknowledgement_and_rearming(self) -> None:
        self.backend.arm_motion()
        self.step(grip=0.0)
        self.step(grip=1.0)
        out = self.step(self.off(self.P0, (0.30, 0.0, 0.0)))  # 300 mm jump in one sample: discontinuity
        self.assertEqual(out["state"], self.ta.FAULT_LATCHED)
        self.assertEqual([p for r, p in writes(self.ctl) if r == 12], [bytes([0]), bytes([4])])  # halted
        self.assertEqual((self.ctl.state, self.backend.halt_count), (4, 1))
        self.r.ad.request_acknowledge()
        self.step(grip=1.0)
        self.step(grip=0.0)
        out = self.step(grip=1.0)  # acknowledged, released, pressed: still refused, the arm is not armed
        self.assertEqual(out["state"], self.ta.WAIT_RELEASE)
        self.assertIn("arm not armed", out["blocked"])
        self.backend.arm_motion()  # the explicit operator step
        self.step(grip=0.0)
        self.assertEqual(self.step(grip=1.0)["state"], self.ta.ACTIVE)

    def test_controller_error_during_teleop_halts_and_is_not_cleared(self) -> None:
        self.backend.arm_motion()
        self.step(grip=0.0)
        self.step(grip=1.0)
        self.ctl.err = 31  # SYNTHETIC
        out = self.step()
        self.assertEqual(out["state"], self.ta.FAULT_LATCHED)
        self.assertTrue(self.r.rec.faults[-1]["reason"].startswith("feedback_failed:controller error 31"))
        regs = [r for r, _ in writes(self.ctl)]
        self.assertEqual(regs.count(12), 2)
        self.assertNotIn(16, regs)
        with self.assertRaises(Exception):
            self.backend.arm_motion()  # arming refused while the error is present

    def test_target_outside_the_links_hard_box_is_refused_and_halts(self) -> None:
        self.backend.arm_motion()
        self.step(grip=0.0)
        self.step(grip=1.0)
        out = {}
        for i in range(1, 160):  # walk the tool toward +x; the link's box ends at x = 330 mm (30 mm away)
            out = self.step(self.off(self.P0, (0.0, 0.0, -0.002 * i)))
            if out["state"] != self.ta.ACTIVE:
                break
        self.assertEqual(out["state"], self.ta.FAULT_LATCHED)
        self.assertTrue(self.r.rec.faults[-1]["reason"].startswith("command_failed:move refused (code 8)"), self.r.rec.faults[-1])
        self.assertLessEqual(max(v[0] for v in self.moves()), 330.0)
        self.assertLessEqual(self.ctl.pose[0], 330.0 + 1e-6)
        self.assertEqual(self.ctl.state, 4)
