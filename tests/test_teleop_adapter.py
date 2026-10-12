"""Tests for the teleop adapter core (engagement, mapping, limits, scheduling, backends).

ALL DATA HERE IS SYNTHETIC unless a test says REPLAY. Synthetic packets are built with the same
sender-side ContinuityTracker and packet builder as the live path and pass through the same
receiver state. A fake backend records what was requested: nothing here shows that a robot
stops, moves or holds. No Quest, no adb, no SSH, no robot, no gripper.

ReplayTests feed saved Quest recordings (REPLAY evidence, not live data) and run only where
those recordings exist (the local PC).
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import quest_pose_transport as pt  # noqa: E402
import teleop_adapter as ta  # noqa: E402
import teleop_backend as tb  # noqa: E402
import teleop_backend_xarm as tx  # noqa: E402
import teleop_input as ti  # noqa: E402
import teleop_limits as tl  # noqa: E402
import teleop_mapping as tm  # noqa: E402
from test_quest_pose_transport import ev_obj, feed, hello, pose_pkt, rec_obj, status_pkt, tracker  # noqa: E402
from test_quest_pose_v1 import ALL, syn_inputs, syn_pose  # noqa: E402

import quest_pose_v1 as qp  # noqa: E402

MS = 1_000_000
CONFIG = ROOT / "config" / "teleop_dryrun_example.json"
P0 = (0.20, 1.00, -0.30)  # synthetic controller start position (LOCAL, metres)


def cfg(**over):  # type: ignore[no-untyped-def]
    d = json.loads(CONFIG.read_text())
    for path, v in over.items():
        sec, key = path.split("__")
        d[sec][key] = v
    return ta.parse_config(d)


class Rig:
    """Synthetic sender -> v2 packets -> receiver state -> channel -> adapter -> fake backend, one virtual clock."""

    def __init__(self, config=None, **backend_kw):  # type: ignore[no-untyped-def]
        self.now = 1_000 * MS
        self.cfg = config or cfg()
        self.channel = ti.InputChannel()
        self.backend = tb.FakeRobotBackend(clock=lambda: self.now, **backend_kw)
        self.rec = ta.Recorder()
        self.ad = ta.TeleopAdapter(self.cfg, self.channel, self.backend, self.rec)
        self.state = ti.ChannelReceiverState(self.channel)
        self.tr = tracker()
        self.seq = self.pseq = 0
        self.tq = 1_000_000_000
        self.sid = "00000000deadbeef"
        self.state.open_connection(self.now)
        self.state.process(hello(), self.now)

    def _send(self, pkt: dict) -> dict:
        return self.state.process(pkt, self.now)  # type: ignore[return-value]

    def pose(self, p=P0, grip=1.0, trigger=0.0, q=(0.0, 0.0, 0.0, 1.0), flags=ALL, tq_ms=33.0, **kw):  # type: ignore[no-untyped-def]
        self.seq += 1
        self.pseq += 1
        self.tq += int(tq_ms * MS)
        kw.setdefault("sid", self.sid)
        a, b = kw.pop("a", False), kw.pop("b", False)
        obj = rec_obj(self.seq, t=self.tq, gr=syn_pose(p, q, flags=flags),
                      in_r=syn_inputs(True, squeeze=grip, trigger=trigger, a=a, b=b), **kw)  # fmt: skip
        feed(self.tr, obj, self.now)
        return self._send(pose_pkt(self.tr, obj, self.pseq, recv=self.now))

    def event(self, name: str, **kw) -> None:  # type: ignore[no-untyped-def]
        self.seq += 1
        obj = ev_obj(self.seq, name, sid=self.sid, **kw)
        feed(self.tr, obj, self.now)
        self.pseq += 1
        self._send({"v": 2, "type": "source_event", "session": "sessA", "seq": self.pseq, "acq_gen": self.tr.acq_gen,
                    "event": obj, "source_recv_mono_ns": self.now})  # fmt: skip

    def status(self) -> None:
        self.pseq += 1
        self._send(status_pkt(self.tr, self.pseq))

    def tick(self, dt_ms: float = 20.0) -> dict:
        self.now += int(dt_ms * MS)
        return self.ad.tick(self.now)

    def step(self, p=P0, grip=1.0, **kw) -> dict:  # type: ignore[no-untyped-def]
        """One new pose, then one scheduler tick."""
        self.pose(p, grip, **kw)
        return self.tick()

    def glide(self, d, steps: int = 20, grip=1.0, **kw) -> dict:  # type: ignore[no-untyped-def]
        """Move the SYNTHETIC controller from P0 to P0+d in small steps (a human-like motion)."""
        out = {}
        for i in range(1, steps + 1):
            out = self.step(off(P0, tuple(x * i / steps for x in d)), grip, **kw)
        return out

    def engage(self, p=P0) -> dict:  # type: ignore[no-untyped-def]
        self.step(p, 0.0)
        out = self.step(p, 1.0)
        assert out["state"] == ta.ACTIVE, out
        return out

    def reasons(self) -> list:
        return [t["reason"] for t in self.rec.transitions]


def off(p, d):  # type: ignore[no-untyped-def]
    return tuple(p[k] + d[k] for k in range(3))


class EngagementTests(unittest.TestCase):
    def test_starts_in_wait_release_and_held_grip_never_engages(self) -> None:
        r = Rig()
        self.assertEqual(r.ad.eng.state, ta.WAIT_RELEASE)
        for _ in range(50):
            self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)
        self.assertEqual((r.backend.command_count, r.backend.stop_count, r.ad.eng.engagements), (0, 0, 0))

    def test_release_then_press_engages_with_zero_target_jump(self) -> None:
        r = Rig(start_p_m=(0.01, -0.02, 0.03))
        self.assertEqual(r.step(P0, 0.1)["state"], ta.ARMED)
        self.assertEqual(r.step(P0, 0.5)["state"], ta.ARMED)  # between the thresholds: not a press
        out = r.step(P0, 0.9)
        self.assertEqual(out["state"], ta.ACTIVE)
        t, p, ori = r.backend.commands[0]
        self.assertEqual(p, (0.01, -0.02, 0.03))  # first target is exactly the fresh tool feedback
        self.assertEqual(ori, tb.FakeRobotBackend.START_ORIENTATION)
        self.assertEqual(r.reasons(), ["valid_release_observed", "valid_press_after_valid_release"])

    def test_null_or_inactive_grip_is_not_a_release(self) -> None:
        r = Rig()
        r.step(P0, 1.0)
        for _ in range(5):
            self.assertEqual(r.step(P0, None)["state"], ta.WAIT_RELEASE)  # grip action inactive / value null
        self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)
        self.assertEqual(r.step(P0, 1.0, flags=0)["state"], ta.WAIT_RELEASE)  # pose missing is not a release either
        self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)
        self.assertEqual(r.backend.command_count, 0)

    def test_release_must_be_followed_by_press_in_one_unbroken_stream(self) -> None:
        r = Rig()
        self.assertEqual(r.step(P0, 0.0)["state"], ta.ARMED)
        r.seq += 3  # SYNTHETIC source gap between the release and the press
        self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)
        self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)
        self.assertEqual(r.backend.command_count, 0)

    def test_grip_release_cancels_pending_motion(self) -> None:
        r = Rig()
        r.engage()
        out = r.glide((0.0, 0.0, -0.10), steps=6)  # request 20 mm; the limiter is still far behind
        self.assertAlmostEqual(out["req_mm"][0], 20.0, places=6)
        self.assertLess(out["lim_mm"][0], 10.0)
        n = r.backend.command_count
        out = r.step(off(P0, (0.0, 0.0, -0.10)), 0.1)
        self.assertEqual(out["state"], ta.WAIT_RELEASE)
        self.assertEqual((r.backend.stop_count, r.backend.stops[-1][1]), (1, "grip_released"))
        self.assertIsNone(r.ad.p_req)  # desired motion dropped, not finished
        self.assertIsNone(r.ad.eng.ref)  # controller and tool references cleared
        for _ in range(30):  # no further target requests, although the old target was never reached
            r.tick()
        self.assertEqual(r.backend.command_count, n)

    def _loss_case(self, break_fn, expect: str, latched: bool = False, holds: int = 0) -> None:  # type: ignore[no-untyped-def]
        r = Rig()
        r.engage()
        r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        n = r.backend.command_count
        out = break_fn(r)
        self.assertNotEqual(out["state"], ta.ACTIVE, expect)
        self.assertTrue(any(expect in x for x in r.reasons()), (expect, r.reasons()))
        self.assertEqual(r.backend.stop_count, 1)
        n += holds  # ticks that legitimately held the last request while the input was still fresh
        self.assertEqual(r.backend.command_count, n)  # the tick that saw the problem issued no target
        self.assertEqual(out["state"], ta.FAULT_LATCHED if latched else ta.WAIT_RELEASE)
        # recovery with the grip still held must not re-engage
        for _ in range(20):
            self.assertNotEqual(r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)["state"], ta.ACTIVE)
        self.assertEqual(r.backend.command_count, n)
        if not latched:  # a deliberate release and press re-engages
            r.step(P0, 0.0)
            self.assertEqual(r.step(P0, 1.0)["state"], ta.ACTIVE)

    def test_focus_loss_disengages(self) -> None:
        self._loss_case(lambda r: r.step(P0, 1.0, state="VISIBLE", focused=False), "continuity_generation_changed (input_unavailable:unfocused)")

    def test_tracking_loss_disengages(self) -> None:
        self._loss_case(lambda r: r.step(P0, 1.0, flags=ALL & ~qp.POSITION_TRACKED), "continuity_generation_changed (input_unavailable:pos_untracked)")

    def test_null_grip_while_active_disengages_as_missing_not_release(self) -> None:
        self._loss_case(lambda r: r.step(P0, None), "continuity_generation_changed (input_unavailable:grip_action_inactive)")

    def test_stale_input_disengages(self) -> None:
        def go(r: Rig) -> dict:
            out = {}
            for i in range(14):  # 280 ms without a new packet (limit 250 ms)
                out = r.tick()
                if i < 11:  # still fresh: the last request is held, never extrapolated
                    self.assertEqual(out["req_mm"], [0.0, -2.0, 0.0])
            return out
        self._loss_case(go, "input_stale", holds=11)

    def test_sender_reported_stale_disengages(self) -> None:
        def go(r: Rig) -> dict:
            r.now += 600 * MS
            r.tr.watchdog(r.now)
            r.status()
            return r.ad.tick(r.now)
        self._loss_case(go, "source_stale")

    def test_disconnect_disengages(self) -> None:
        def go(r: Rig) -> dict:
            r.state.close_connection(r.now, "EOF")
            out = r.tick()
            r.state.open_connection(r.now)
            r.pseq = 0
            r.state.process(hello(), r.now)
            return out
        self._loss_case(go, "transport_disconnected")

    def test_continuity_generation_change_disengages_even_between_poses(self) -> None:
        def go(r: Rig) -> dict:
            r.seq += 1  # one SYNTHETIC missing source record: the next pose looks perfectly normal
            return r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        self._loss_case(go, "continuity_generation_changed")

    def test_status_break_without_a_pose_disengages(self) -> None:
        def go(r: Rig) -> dict:
            r.tr._break("parse_error", r.now)
            r.status()
            return r.tick()
        self._loss_case(go, "continuity_break_after_last_pose")

    def test_app_restart_disengages(self) -> None:
        def go(r: Rig) -> dict:
            r.sid, r.seq = "1111111122222222", 0
            return r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        self._loss_case(go, "source_identity_changed")

    def test_acquisition_restart_disengages(self) -> None:
        def go(r: Rig) -> dict:
            r.tr.new_generation(r.now, "restart")
            r.seq = 0
            return r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        self._loss_case(go, "source_identity_changed")

    def test_rejected_packet_disengages(self) -> None:
        def go(r: Rig) -> dict:
            r.pseq += 1
            bad = pose_pkt(r.tr, rec_obj(r.seq + 1), r.pseq)
            bad["matrices"]["grip_r"][0][3] = 9.9  # not the unmodified LOCAL pose
            r.seq += 1
            self.assertEqual(r._send(bad)["status"], "rejected")
            return r.tick()
        self._loss_case(go, "packet_rejected")


class RecordingFieldTests(unittest.TestCase):
    """What episode recording reads from the adapter: thumb buttons, the orientation sent, and the record tap."""

    def test_thumb_buttons_are_reported_and_change_nothing_about_motion(self) -> None:
        plain, pressed = Rig(), Rig()
        for r, btn in ((plain, {}), (pressed, {"a": True, "b": True})):
            for _ in range(5):
                self.assertEqual(r.step(P0, 1.0, **btn)["state"], ta.WAIT_RELEASE)  # a button is not a clutch
            r.step(P0, 0.0, **btn)
            r.step(P0, 1.0, **btn)
            out = r.glide((0.0, 0.0, -0.05), 10, **btn)
            self.assertEqual((out["btn_a"], out["btn_b"]), (btn.get("a", False), btn.get("b", False)))
        self.assertEqual([c[1:] for c in plain.backend.commands], [c[1:] for c in pressed.backend.commands])
        self.assertGreater(len(plain.backend.commands), 5)
        r = Rig()
        self.assertEqual((r.tick()["btn_a"], r.tick()["btn_b"]), (None, None))  # no input yet: missing, not released

    def test_the_orientation_sent_is_in_the_tick_only_when_a_target_was_sent(self) -> None:
        r = Rig()
        self.assertIsNone(r.step(P0, 0.0)["cmd_ori"])
        first = r.step(P0, 1.0)
        self.assertEqual((first["cmd_ori"], first["cmd_mm"] is not None), (list(r.backend.commands[-1][2]), True))
        out = r.glide((0.0, 0.0, -0.03), 5)
        self.assertEqual(out["cmd_ori"], list(r.backend.commands[-1][2]))
        out = r.step(off(P0, (0.0, 0.0, -0.03)), 0.0)  # released
        self.assertEqual((out["cmd_mm"], out["cmd_ori"]), (None, None))

    def test_the_tap_sees_every_record_and_a_failing_tap_is_dropped_not_propagated(self) -> None:
        seen: list = []
        rec = ta.Recorder(tap=seen.append)
        rec.write("tick", 1, state="X")
        rec.write("fault", 2, reason="r")
        self.assertEqual(seen, [{"k": "tick", "t": 1, "state": "X"}, {"k": "fault", "t": 2, "reason": "r"}])

        def bad(_: dict) -> None:
            raise RuntimeError("observer broke")

        rec = ta.Recorder(tap=bad)
        rec.write("tick", 1)
        rec.write("tick", 2)
        self.assertEqual((rec.counts["tick"], rec.counts["tap_failed"]), (2, 1))


class ReferenceChangeTests(unittest.TestCase):
    def test_reference_counter_change_is_handled_before_its_pose(self) -> None:
        r = Rig()
        r.engage()
        n = r.backend.command_count
        jumped = off(P0, (0.05, 0.0, 0.0))
        out = r.step(jumped, 1.0, epoch=1, change=123)  # counter changed, no event seen
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.backend.command_count, n)  # no target was computed from that pose
        self.assertEqual(r.backend.stop_count, 1)
        self.assertTrue(any("reference" in x for x in r.ad.eng.latch_reasons))

    def test_reference_event_latches_and_needs_acknowledgement_release_and_press(self) -> None:
        r = Rig()
        r.engage()
        r.event("local_space_change_pending", t=5, epoch=1)
        r.status()
        out = r.tick()
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.backend.stop_count, 1)
        n = r.backend.command_count
        for g in (1.0, 0.0, 1.0, 0.0, 1.0):  # release and press alone are not enough
            self.assertEqual(r.step(P0, g, epoch=1, change=5)["state"], ta.FAULT_LATCHED)
        r.ad.request_acknowledge()
        self.assertEqual(r.step(P0, 1.0, epoch=1, change=5)["state"], ta.WAIT_RELEASE)  # ack never engages
        self.assertEqual(r.step(P0, 0.0, epoch=1, change=5)["state"], ta.ARMED)
        self.assertEqual(r.backend.command_count, n)
        self.assertEqual(r.step(P0, 1.0, epoch=1, change=5)["state"], ta.ACTIVE)
        self.assertTrue(r.rec.acks[0]["accepted"])

    def test_unknown_space_change_while_disengaged_latches(self) -> None:
        r = Rig()
        r.event("other_space_change_pending", t=5)
        r.tick()
        self.assertEqual(r.ad.eng.state, ta.FAULT_LATCHED)
        r.step(P0, 0.0)
        self.assertEqual(r.step(P0, 1.0)["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.backend.command_count, 0)

    def test_acknowledge_is_refused_while_active(self) -> None:
        r = Rig()
        r.engage()
        r.ad.request_acknowledge()
        self.assertEqual(r.step(P0, 1.0)["state"], ta.ACTIVE)
        self.assertFalse(r.rec.acks[0]["accepted"])


class MappingTests(unittest.TestCase):
    def test_rotation_must_be_proper(self) -> None:
        tm.validate_rotation([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        for bad in ([[1, 0, 0], [0, 1, 0], [0, 0, -1]],  # reflection, det -1
                    [[1, 0, 0], [0, 1, 0], [0, 0, 1.1]],  # not unit length
                    [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]],  # not orthogonal
                    [[1, 0, 0], [0, 1, 0]], "identity", [[float("nan"), 0, 0], [0, 1, 0], [0, 0, 1]]):
            with self.assertRaises(tm.ConfigError):
                tm.validate_rotation(bad)
        with self.assertRaises(tm.ConfigError):
            tm.AnchoredTranslation(0.0, [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        with self.assertRaises(tm.ConfigError):
            cfg(mapping__R_backend_from_local=[[1, 0, 0], [0, 1, 0], [0, 0, -1]])

    def test_scaling_axis_mapping_and_units(self) -> None:
        # Example mapping (UNVERIFIED): LOCAL -z -> +x, LOCAL +x -> -y, LOCAL +y -> +z; scale 0.2.
        for d_local, expect_mm in (((0.0, 0.0, -0.100), (20.0, 0.0, 0.0)), ((0.100, 0.0, 0.0), (0.0, -20.0, 0.0)),
                                   ((0.0, 0.100, 0.0), (0.0, 0.0, 20.0)), ((0.05, -0.02, 0.03), (-6.0, -10.0, -4.0))):  # fmt: skip
            r = Rig()
            r.engage()
            out = r.glide(d_local)
            for _ in range(80):  # hold the displaced controller until the limited target arrives
                out = r.step(off(P0, d_local), 1.0)
            for k in range(3):
                self.assertAlmostEqual(out["req_mm"][k], expect_mm[k], places=6)
                self.assertAlmostEqual(out["lim_mm"][k], expect_mm[k], places=3)
                self.assertAlmostEqual(r.backend.commands[-1][1][k], expect_mm[k] / 1000.0, places=6)  # metres at the interface

    def test_controller_rotation_never_changes_tool_orientation(self) -> None:
        r = Rig()
        r.engage()
        for i in range(1, 60):
            a = math.radians(3 * i)
            q = (math.sin(a / 2) * 0.6, math.sin(a / 2) * 0.8, 0.0, math.cos(a / 2))
            out = r.step(off(P0, (0.0005 * i, 0.0, 0.0)), 1.0, q=q)
            self.assertEqual(out["state"], ta.ACTIVE)
        self.assertEqual({c[2] for c in r.backend.commands}, {tb.FakeRobotBackend.START_ORIENTATION})

    def test_pure_rotation_requests_no_translation(self) -> None:
        r = Rig()
        r.engage()
        for i in range(1, 30):
            a = math.radians(5 * i)
            out = r.step(P0, 1.0, q=(0.0, math.sin(a / 2), 0.0, math.cos(a / 2)))
            self.assertEqual(out["req_mm"], [0.0, 0.0, 0.0])

    def test_xarm_boundary_conversion_with_injected_fake(self) -> None:
        kw = tx.XArmPreparedBackend.native_target((0.3001, -0.0125, 0.4612), (179.86, -15.1, -6.64))
        self.assertEqual({k: round(v, 6) for k, v in kw.items()},
                         {"x": 300.1, "y": -12.5, "z": 461.2, "roll": 179.86, "pitch": -15.1, "yaw": -6.64})  # fmt: skip


class LimitTests(unittest.TestCase):
    def test_discontinuity_is_rejected_before_rate_limiting(self) -> None:
        r = Rig()
        r.engage()
        r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        n = r.backend.command_count
        last = r.backend.commands[-1][1]
        out = r.step(off(P0, (0.30, 0.0, 0.0)), 1.0)  # 290 mm jump in one sample (limit 120 mm)
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("controller_discontinuity:step"))
        self.assertEqual(r.backend.command_count, n)  # not turned into slow motion toward the jump
        self.assertEqual(r.backend.commands[-1][1], last)
        self.assertEqual(r.backend.stop_count, 1)

    def test_fast_small_jump_is_rejected_by_source_time(self) -> None:
        r = Rig()
        r.engage()
        out = r.step(off(P0, (0.08, 0.0, 0.0)), 1.0, tq_ms=14.0)  # 80 mm in 14 ms of device time = 5.7 m/s
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("controller_discontinuity:speed"))

    def test_fixed_workspace_does_not_move_with_reengagement(self) -> None:
        r = Rig()
        r.engage()
        p, out = P0, {}
        for i in range(1, 201):
            p = off(P0, (0.0, 0.0, -0.002 * i))  # 2 mm per sample: 400 mm of hand travel -> 80 mm of tool travel
            out = r.step(p, 1.0)
        for _ in range(120):
            out = r.step(p, 1.0)
        self.assertEqual(out["state"], ta.ACTIVE)
        self.assertAlmostEqual(out["fb_mm"][0], 80.0, places=2)
        r.step(p, 0.0)  # clutch: release, move the hand back, press again
        r.step(P0, 0.0)
        self.assertEqual(r.step(P0, 1.0)["state"], ta.ACTIVE)
        self.assertAlmostEqual(r.backend.commands[-1][1][0], 0.080, places=5)  # re-engaged without a jump
        for i in range(1, 101):  # another 30 mm would reach +110 mm: outside the FIXED +100 mm limit
            out = r.step(off(P0, (0.0, 0.0, -0.0015 * i)), 1.0)
            if out["state"] != ta.ACTIVE:
                break
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("workspace_violation:requested x=+10"), r.rec.faults[-1])
        self.assertTrue(r.rec.faults[-1]["limit"])
        self.assertLessEqual(max(c[1][0] for c in r.backend.commands), 0.100 + 1e-9)

    def test_clamp_option_holds_at_the_workspace_edge_instead_of_faulting(self) -> None:
        r = Rig(cfg(limits__workspace_edge="clamp", limits__max_anchor_displacement_m=0.5))
        r.engage()
        out = {}
        for i in range(1, 401):  # 800 mm of hand travel -> 160 mm requested; the fixed workspace ends at +100 mm
            out = r.step(off(P0, (0.0, 0.0, -0.002 * i)), 1.0)
        self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
        self.assertAlmostEqual(out["req_mm"][0], 99.8, places=6)  # held 0.2 mm inside the edge
        self.assertLessEqual(max(c[1][0] for c in r.backend.commands), 0.100 + 1e-9)
        for i in range(400, 300, -1):  # coming back: the tool leaves the edge only once the hand is back inside
            out = r.step(off(P0, (0.0, 0.0, -0.002 * i)), 1.0)
        self.assertAlmostEqual(out["req_mm"][0], 99.8, places=6)
        for i in range(300, 199, -1):
            out = r.step(off(P0, (0.0, 0.0, -0.002 * i)), 1.0)
        self.assertAlmostEqual(out["req_mm"][0], 80.0, places=6)
        with self.assertRaises(tm.ConfigError):
            cfg(limits__workspace_edge="bounce")

    def test_holding_at_an_awkward_edge_never_faults(self) -> None:
        # Regression for the first live session (2026-10-12): the request was held on the box edge at
        # y = +1.59 mm while the hand kept moving along the other axes, and the rate-limited target landed
        # a rounding error outside the edge -> latched "workspace_violation:limited".
        import math as m
        for edge in (0.00159, 0.0503177, 0.0271828):
            r = Rig(cfg(limits__workspace_edge="clamp", limits__max_anchor_displacement_m=0.5,
                        limits__workspace_max_m=[edge, 0.10, 0.10]))  # fmt: skip
            r.engage()
            out, dts = {}, [20.0, 20.02, 19.99, 20.01, 20.03, 19.98]
            for i in range(1, 1500):
                hand = (0.05 * m.sin(i * 0.013), 0.04 * m.sin(i * 0.0071), -min(0.6, 0.0011 * i))  # oblique: x keeps pushing past the edge
                r.pose(off(P0, hand), 1.0)
                out = r.tick(dts[i % len(dts)])
                self.assertEqual(out["state"], ta.ACTIVE, (edge, i, r.rec.faults))
            self.assertLessEqual(out["lim_mm"][0], edge * 1000.0)
            self.assertGreater(out["lim_mm"][0], edge * 1000.0 - 0.25)

    def test_anchor_displacement_limit(self) -> None:
        r = Rig(cfg(limits__max_anchor_displacement_m=0.01))
        r.engage()
        out = r.glide((0.06, 0.0, 0.0))  # 12 mm of tool displacement requested
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("anchor_displacement_violation"))

    def test_rate_and_acceleration_bounds_under_irregular_arrival(self) -> None:
        r = Rig()
        r.engage()
        v_max, a_max = r.cfg.limits.v_max_m_s, r.cfg.limits.a_max_m_s2
        prev_p, prev_v, prev_t = r.backend.commands[-1][1], (0.0, 0.0, 0.0), r.now
        gaps = [0, 0, 3, 0, 2, 1, 0, 0, 3, 2, 0, 1] * 14  # ticks without a new packet; then bursts
        dts = [20.0, 20.0, 35.0, 12.0, 20.0, 40.0, 20.0, 8.0, 20.0, 30.0]  # irregular scheduler timing too
        i = 0
        target = off(P0, (0.0, 0.0, -0.30))
        cur = P0
        for g in gaps:
            cur = tuple(cur[k] + max(-0.004, min(0.004, target[k] - cur[k])) for k in range(3))
            r.pose(cur, 1.0, tq_ms=14.0 * (g + 1))
            for _ in range(g + 1):
                dt = dts[i % len(dts)]
                i += 1
                out = r.tick(dt)
                self.assertEqual(out["state"], ta.ACTIVE, r.rec.faults)
                p = r.backend.commands[-1][1]
                v = tuple((p[k] - prev_p[k]) / (dt / 1000.0) for k in range(3))
                self.assertLessEqual(math.sqrt(sum(x * x for x in v)), v_max * (1 + 1e-9))
                dv = math.sqrt(sum((v[k] - prev_v[k]) ** 2 for k in range(3)))
                self.assertLessEqual(dv, a_max * (dt / 1000.0) * (1 + 1e-6) + 1e-12)
                prev_p, prev_v, prev_t = p, v, r.now
        self.assertGreater(prev_p[0], 0.02)  # it did move

    def test_no_backlog_only_newest_input_is_used(self) -> None:
        r = Rig()
        r.engage()
        r2 = r
        for i in range(1, 11):  # ten packets arrive between two ticks (network recovery)
            r2.pose(off(P0, (0.0, 0.0, -0.001 * i)), 1.0)
        n = r2.backend.command_count
        out = r2.tick()
        self.assertEqual(out["state"], ta.ACTIVE)
        self.assertEqual(r2.backend.command_count, n + 1)  # one target for ten packets
        self.assertAlmostEqual(out["req_mm"][0], 2.0, places=6)  # from the newest packet only

    def test_scheduler_overrun_latches(self) -> None:
        r = Rig()
        r.engage()
        r.pose(P0, 1.0)
        out = r.tick(150.0)  # one tick arrives 150 ms late (limit 100 ms)
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("scheduler_overrun"))


class BackendFaultTests(unittest.TestCase):
    def _latched(self, r: Rig, prefix: str) -> None:
        self.assertEqual(r.ad.eng.state, ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith(prefix), r.rec.faults[-1])
        n = r.backend.command_count
        r.backend.fail_feedback = r.backend.fail_command = None
        r.backend.frozen_read_ns = None
        for g in (0.0, 1.0, 0.0, 1.0):  # never resumes by itself, not even with a release and press
            self.assertEqual(r.step(P0, g)["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.backend.command_count, n)
        r.ad.request_acknowledge()
        r.step(P0, 1.0)
        r.step(P0, 0.0)
        self.assertEqual(r.step(P0, 1.0)["state"], ta.ACTIVE)

    def test_command_failure_latches(self) -> None:
        r = Rig()
        r.engage()
        r.backend.fail_command = "synthetic"
        r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        self.assertEqual(r.backend.stop_count, 1)
        self._latched(r, "command_failed")

    def test_feedback_failure_latches(self) -> None:
        r = Rig()
        r.engage()
        r.backend.fail_feedback = "synthetic"
        n = r.backend.command_count
        r.step(off(P0, (0.01, 0.0, 0.0)), 1.0)
        self.assertEqual((r.backend.command_count, r.backend.stop_count), (n, 1))
        self._latched(r, "feedback_failed")

    def test_stale_feedback_latches(self) -> None:
        r = Rig()
        r.engage()
        r.backend.frozen_read_ns = r.now - 500 * MS
        r.step(P0, 1.0)
        self._latched(r, "feedback_stale")

    def test_target_feedback_error_latches(self) -> None:
        r = Rig()
        r.engage()
        r.backend.tracking = False  # the synthetic tool stops following
        out = {}
        for i in range(1, 200):
            out = r.step(off(P0, (0.0, 0.0, -0.002 * min(i, 40))), 1.0)
            if out["state"] != ta.ACTIVE:
                break
        r.backend.tracking = True
        self._latched(r, "target_feedback_error")

    def test_engagement_refused_without_fresh_feedback(self) -> None:
        r = Rig()
        r.backend.fail_feedback = "synthetic"
        r.step(P0, 0.0)
        self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)
        r.backend.fail_feedback = None
        self.assertEqual(r.step(P0, 1.0)["state"], ta.WAIT_RELEASE)  # grip held: a new release is required
        self.assertEqual(r.backend.command_count, 0)

    def test_monitor_detects_blocked_scheduler_and_requests_stop(self) -> None:
        r = Rig()
        r.engage()
        gate = threading.Event()
        r.backend.command_gate = gate
        r.pose(off(P0, (0.01, 0.0, 0.0)), 1.0)
        r.now += 20 * MS
        result: list = []
        th = threading.Thread(target=lambda: result.append(r.ad.tick(r.now)), daemon=True)
        th.start()
        for _ in range(200):  # wait until the tick is inside the blocking backend call
            if r.backend.command_count >= 2:
                break
            threading.Event().wait(0.005)
        self.assertEqual(r.backend.command_count, 2)
        self.assertIsNone(r.ad.monitor(r.now + 100 * MS))  # not yet over the stall limit
        why = r.ad.monitor(r.now + 200 * MS)  # the tick has not returned
        self.assertTrue(why.startswith("monitor:scheduler_blocked_in_tick"), why)
        self.assertEqual((r.backend.stop_count, r.backend.stops[-1][1]), (1, why))  # stop without waiting for the tick
        self.assertTrue(th.is_alive())
        gate.set()
        th.join(2.0)
        r.backend.command_gate = None
        self.assertEqual(result[0]["state"], ta.FAULT_LATCHED)
        n = r.backend.command_count
        self.assertEqual(r.step(P0, 1.0)["state"], ta.FAULT_LATCHED)
        self.assertEqual(r.backend.command_count, n)

    def test_monitor_detects_scheduler_that_stopped_running(self) -> None:
        r = Rig()
        r.engage()
        self.assertIsNone(r.ad.monitor(r.now + 50 * MS))
        why = r.ad.monitor(r.now + 300 * MS)
        self.assertTrue(why.startswith("monitor:scheduler_not_running"))
        self.assertEqual(r.backend.stop_count, 1)
        self.assertIsNone(r.ad.monitor(r.now + 400 * MS))  # one request, not a stream of them

    def test_no_target_after_disengagement_until_deliberate_reengagement(self) -> None:
        r = Rig()
        r.engage()
        r.glide((0.0, 0.0, -0.05))
        r.step(off(P0, (0.0, 0.0, -0.05)), 1.0, state="VISIBLE", focused=False)
        t_dis = r.now
        for _ in range(100):
            r.step(off(P0, (0.0, 0.0, -0.05)), 1.0)
        self.assertFalse([c for c in r.backend.commands if c[0] >= t_dis])
        r.step(P0, 0.0)
        r.step(P0, 1.0)
        after = [c for c in r.backend.commands if c[0] >= t_dis]
        self.assertEqual(len(after), 1)
        self.assertEqual(after[0][1], r.backend.read_feedback().p_m)


class GripperTests(unittest.TestCase):
    def test_trigger_mapping_direction_range_and_missing(self) -> None:
        self.assertEqual([tm.trigger_to_pct_closed(v) for v in (0.0, 0.25, 1.0)], [0.0, 25.0, 100.0])
        for missing in (None, float("nan"), 1.5, -0.2, True, "0.5"):
            self.assertIsNone(tm.trigger_to_pct_closed(missing))  # missing stays missing, never 0
        self.assertEqual([tm.pct_closed_to_ticks(p) for p in (0.0, 100.0, 50.0)], [3033, 1181, 2107])

    def test_no_initial_jump_pickup_before_following(self) -> None:
        r = Rig()  # synthetic jaws at 0 % closed
        r.step(P0, 0.0, trigger=0.9)
        out = r.step(P0, 1.0, trigger=0.9)
        for _ in range(20):
            out = r.step(P0, 1.0, trigger=0.9)  # trigger far from the jaw position: nothing proposed
        self.assertEqual((out["gripper"], r.backend.preview_count), ("waiting_pickup", 0))
        out = r.step(P0, 1.0, trigger=0.03)  # within 5 % of the jaws: pickup
        self.assertEqual(out["gripper"], "following")
        out = r.step(P0, 1.0, trigger=1.0)
        self.assertLessEqual(out["gripper_pct"], 3.0 + 100.0 * 0.02 + 1e-6)  # rate limited, 100 %/s
        self.assertGreater(r.backend.preview_count, 0)

    def test_missing_trigger_and_disengagement_send_nothing(self) -> None:
        r = Rig()
        r.engage()
        r.step(P0, 1.0, trigger=0.0)
        n = r.backend.preview_count
        self.assertEqual(r.step(P0, 1.0, trigger=None)["gripper"], "trigger_missing")
        self.assertEqual(r.backend.preview_count, n)
        r.step(P0, 1.0, trigger=0.0)
        n = r.backend.preview_count
        r.step(P0, 0.0, trigger=0.0)  # arm disengaged
        for _ in range(10):
            r.step(P0, 0.0, trigger=1.0)
        self.assertEqual(r.backend.preview_count, n)  # no opening or closing request after disengagement
        self.assertFalse(r.ad.follower.following)

    def test_gripper_preview_cannot_change_hardware(self) -> None:
        calls: list = []
        g = tx.GripctlPreparedGripper(lambda path: calls.append(("GET", path)) or {"values": {"position": 2107}, "status": {"connected": True}})
        self.assertAlmostEqual(g.read().pct_closed, 50.0, places=3)
        prop = tb.GripperProposal(40.0, tm.pct_closed_to_ticks(40.0), 0.4)
        self.assertEqual(g.native_command(prop), ("/api/do/grip", {"value": 40.0}))
        with self.assertRaises(tb.LiveIntegrationBlocked):
            g.command(prop)
        self.assertEqual(calls, [("GET", "/api/state")])  # only the read; no POST exists
        self.assertFalse(hasattr(tb.RobotBackend, "command_gripper") or hasattr(tb.FakeRobotBackend, "command_gripper"))
        r = Rig()
        r.engage()
        for _ in range(10):
            r.step(P0, 1.0, trigger=0.0)
        self.assertEqual(r.backend.read_gripper().pct_closed, 0.0)  # synthetic jaws never move


class FakeArm:
    """Injected stand-in for the SDK object. Records every call."""

    def __init__(self) -> None:
        self.calls: list = []
        self.pose = [299.83, 3.43, 461.18, 179.86, -15.1, -6.64]
        self.err = [0, 0]
        self.code = 0

    def get_position(self):  # type: ignore[no-untyped-def]
        self.calls.append("get_position")
        return self.code, list(self.pose)

    def get_err_warn_code(self):  # type: ignore[no-untyped-def]
        self.calls.append("get_err_warn_code")
        return 0, list(self.err)

    def emergency_stop(self) -> None:
        self.calls.append("emergency_stop")

    def set_state(self, s: int) -> int:
        self.calls.append(f"set_state({s})")
        return 0

    def __getattr__(self, name: str):  # type: ignore[no-untyped-def]
        def other(*a, **k):  # type: ignore[no-untyped-def]
            self.calls.append(name)
            return 0
        return other


class PreparedXArmBackendTests(unittest.TestCase):
    FORBIDDEN = {"set_position", "motion_enable", "set_mode", "set_state(0)", "clean_error", "clean_warn", "reset",
                 "set_servo_angle", "move_gohome", "connect", "set_reduced_mode", "set_tcp_offset"}  # fmt: skip

    def test_feedback_units_and_error_refusal(self) -> None:
        arm = FakeArm()
        b = tx.XArmPreparedBackend(arm, is_radian=False, clock=lambda: 42)
        fb = b.read_feedback()
        self.assertEqual(tuple(round(v, 6) for v in fb.p_m), (0.29983, 0.00343, 0.46118))
        self.assertEqual((fb.orientation, fb.synthetic, fb.read_mono_ns), ((179.86, -15.1, -6.64), False, 42))
        arm.err = [22, 0]
        with self.assertRaises(tb.BackendError):
            b.read_feedback()
        arm.err, arm.code = [0, 0], 1
        with self.assertRaises(tb.BackendError):
            b.read_feedback()
        self.assertFalse(self.FORBIDDEN & set(arm.calls))  # an error is never cleared

    def test_command_path_is_blocked_not_guessed(self) -> None:
        arm = FakeArm()
        b = tx.XArmPreparedBackend(arm, is_radian=False)
        with self.assertRaises(tb.LiveIntegrationBlocked):
            b.command_target((0.3, 0.0, 0.46), (179.86, -15.1, -6.64))
        self.assertEqual(arm.calls, [])
        with self.assertRaises(tb.LiveIntegrationBlocked):
            tx.XArmPreparedBackend(arm, is_radian=True)

    def test_stop_sequence_is_the_inventory_one(self) -> None:
        arm = FakeArm()
        tx.XArmPreparedBackend(arm, is_radian=False).stop("test")
        self.assertEqual(arm.calls, ["emergency_stop", "set_state(4)"])

    def test_adapter_on_prepared_backend_latches_at_the_first_command(self) -> None:
        r = Rig()
        arm = FakeArm()
        arm.pose[:3] = [10.0, 20.0, 30.0]  # inside the synthetic test workspace (mm)
        r.ad.backend = tx.XArmPreparedBackend(arm, is_radian=False, clock=lambda: r.now)
        r.step(P0, 0.0)
        out = r.step(P0, 1.0)
        self.assertEqual(out["state"], ta.FAULT_LATCHED)
        self.assertTrue(r.rec.faults[-1]["reason"].startswith("command_failed:arm command path is blocked"))
        self.assertFalse(self.FORBIDDEN & set(arm.calls))
        self.assertEqual(arm.calls[-2:], ["emergency_stop", "set_state(4)"])


class DryRunIsolationTests(unittest.TestCase):
    def test_sources_never_import_or_construct_hardware_clients(self) -> None:
        for f in sorted((ROOT / "scripts").glob("teleop_*.py")):
            src = f.read_text()
            code = "\n".join(ln for ln in src.splitlines() if ln.lstrip().startswith(("import ", "from ")))
            for bad in ("xarm", "dynamixel", "serial", "urllib", "requests", "http", "aiohttp", "rclpy", "rospy"):
                self.assertNotIn(bad, code, f"{f.name} imports {bad}")
            self.assertNotIn("XArmAPI(", src, f.name)
        dry = (ROOT / "scripts" / "teleop_dryrun.py").read_text()
        self.assertNotIn("teleop_backend_xarm", dry)
        self.assertEqual(dry.count("FakeRobotBackend()"), 1)

    def test_dry_run_process_loads_no_hardware_module_and_opens_no_outgoing_connection(self) -> None:
        prog = r"""
import socket, sys, importlib.abc
BLOCK = ("xarm", "dynamixel_sdk", "serial", "aiohttp", "requests", "urllib.request", "http.client", "rclpy", "rospy", "teleop_backend_xarm")
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name in BLOCK or name.split(".")[0] in BLOCK:
            raise ImportError("FORBIDDEN IMPORT " + name)
sys.meta_path.insert(0, Guard())
def no_connect(self, *a, **k):
    raise RuntimeError("FORBIDDEN OUTGOING CONNECTION %r" % (a,))
socket.socket.connect = no_connect
socket.socket.connect_ex = no_connect
sys.path.insert(0, sys.argv[1])
import teleop_dryrun
rc = teleop_dryrun.main(["--config", sys.argv[2], "--port", "0", "--duration", "0.6"])
bad = [m for m in sys.modules if m in BLOCK or m.split(".")[0] in ("xarm", "dynamixel_sdk", "serial", "aiohttp", "requests")]
print("RC", rc, "BAD", bad)
"""
        out = subprocess.run([sys.executable, "-c", prog, str(ROOT / "scripts"), str(CONFIG)], capture_output=True, text=True, timeout=30)
        self.assertIn("RC 0 BAD []", out.stdout, out.stdout[-800:] + out.stderr[-800:])
        self.assertIn("DRY RUN: NO ROBOT OR GRIPPER CONNECTED", out.stdout)
        self.assertIn('"final_state": "WAIT_RELEASE"', out.stdout)
        self.assertIn('"synthetic_target_requests": 0', out.stdout)

    def test_dry_run_refuses_a_live_marked_configuration(self) -> None:
        import tempfile
        import teleop_dryrun
        d = json.loads(CONFIG.read_text())
        d["approved_for_live"] = True
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "c.json"
            p.write_text(json.dumps(d))
            self.assertEqual(teleop_dryrun.main(["--config", str(p), "--port", "0", "--duration", "0.1"]), 2)

    def test_fake_backend_declares_itself_synthetic(self) -> None:
        b = tb.FakeRobotBackend(clock=lambda: 1)
        fb = b.read_feedback()
        self.assertTrue(fb.synthetic and b.synthetic and b.read_gripper().synthetic)
        self.assertIn("SYNTHETIC", fb.frame)
        self.assertTrue(b.describe()["synthetic"])


class DryRunProcessTests(unittest.TestCase):
    """The real entry point over a real loopback socket, fed SYNTHETIC packets in real time."""

    def test_entry_point_engages_follows_and_stops_over_tcp(self) -> None:
        import os
        import signal
        import socket
        import tempfile
        import time

        import quest_transport as qt

        with socket.socket() as s0:
            s0.bind(("127.0.0.1", 0))
            port = s0.getsockname()[1]
        with tempfile.TemporaryDirectory() as tmp:
            summ, log = Path(tmp) / "s.json", Path(tmp) / "a.jsonl"
            proc = subprocess.Popen([sys.executable, str(ROOT / "scripts" / "teleop_dryrun.py"), "--config", str(CONFIG),
                                     "--port", str(port), "--summary", str(summ), "--log", str(log)],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)  # fmt: skip
            try:
                sock = None
                for _ in range(100):
                    try:
                        sock = socket.create_connection(("127.0.0.1", port), timeout=2)
                        break
                    except OSError:
                        time.sleep(0.05)
                assert sock is not None
                framer = qt.LineFramer(qt.MAX_MESSAGE_BYTES)

                def rpc(pkt: dict) -> dict:
                    sock.sendall(qt.encode(pkt))
                    while True:
                        frames = framer.feed(sock.recv(4096))
                        if frames:
                            return json.loads(frames[0])

                tr, seq = tracker(), [0]

                def pose(p, grip, tq):  # type: ignore[no-untyped-def]
                    seq[0] += 1
                    obj = rec_obj(seq[0], t=tq, gr=syn_pose(p), in_r=syn_inputs(True, squeeze=grip, trigger=0.0, a=False, b=False))
                    feed(tr, obj, 0)
                    self.assertEqual(rpc(pose_pkt(tr, obj, seq[0]))["status"], "ok")
                    time.sleep(0.03)

                self.assertEqual(rpc(hello())["type"], "hello_ack")
                tq = 10**9
                for i in range(60):  # held, released, pressed, glide 50 mm along LOCAL -z, release
                    tq += 33_000_000
                    grip = 1.0 if i < 5 else 0.0 if i < 10 else 1.0 if i < 50 else 0.0
                    d = 0.0 if i < 15 else min(0.05, 0.002 * (i - 15))
                    pose(off(P0, (0.0, 0.0, -d)), grip, tq)
                    if i == 12:
                        os.kill(proc.pid, signal.SIGUSR1)  # acknowledgement while ACTIVE: must be refused
                sock.close()
                time.sleep(0.3)
            finally:
                proc.send_signal(signal.SIGINT)
                out, _ = proc.communicate(timeout=10)
            o = json.loads(summ.read_text())
            self.assertEqual(proc.returncode, 0, out[-500:])
            self.assertEqual((o["engagements"], o["final_state"], o["robot_connection_attempted"]), (1, "WAIT_RELEASE", False))
            self.assertEqual([x["reason"] for x in o["stops"]], ["grip_released"])
            self.assertEqual([x["accepted"] for x in o["acks"]], [False])
            self.assertGreater(o["synthetic_target_requests"], 20)
            ticks = [json.loads(x) for x in log.read_text().splitlines()]
            lim = [t["lim_mm"][0] for t in ticks if t["k"] == "tick" and t["lim_mm"]]
            self.assertEqual(lim[0], 0.0)  # zero jump at engagement
            self.assertTrue(4.0 < max(lim) <= 10.0 + 1e-6, max(lim))  # 50 mm of hand travel -> at most 10 mm
            self.assertIn("DRY RUN: NO ROBOT OR GRIPPER CONNECTED", out)


LIVE1 = ROOT / "data/checkpoint10/src_tele_20261012-182231.jsonl"
LIVE1_SUMMARY = ROOT / "data/checkpoint10/teleop_tele_20261012-182231.summary.json"


@unittest.skipUnless(LIVE1.exists() and LIVE1_SUMMARY.exists(), "the first live session's recording is only on the local PC")
class LiveSessionReplayTests(unittest.TestCase):
    """REPLAY of the recorded input of the first live arm session (2026-10-12) against a FAKE arm."""

    def test_the_box_edge_fault_of_the_first_live_session_is_gone(self) -> None:
        import teleop_limits as tl
        import teleop_replay
        summ = json.loads(LIVE1_SUMMARY.read_text())
        start = tuple(v / 1000 for v in summ["workspace_mm"]["start"])
        run = lambda: teleop_replay.replay(LIVE1, ta.parse_config(summ["config"]), ack_at_s=(8.0,), start_p_m=start)  # noqa: E731
        o = run()
        self.assertEqual((o["engagements"], [f for f in o["faults"] if "reference" not in f["reason"]]), (1, []))
        hi = summ["config"]["limits"]["workspace_max_m"]
        self.assertLessEqual(max(c[1][1] for c in o["commands"]), hi[1])  # the y edge the session ran into
        self.assertGreater(o["synthetic_target_requests"], 700)
        # as that session actually ran (request held exactly on the edge, limited target not confined): the fault comes back
        inset, eps, confine = tl.CLAMP_INSET_M, tl.EDGE_EPS_M, tl.RateLimiter.confine
        tl.CLAMP_INSET_M, tl.EDGE_EPS_M, tl.RateLimiter.confine = 0.0, 0.0, lambda self, lo, hi: self.p
        try:
            old = run()
        finally:
            tl.CLAMP_INSET_M, tl.EDGE_EPS_M, tl.RateLimiter.confine = inset, eps, confine
        self.assertTrue(any(f["reason"].startswith("workspace_violation:limited y=") for f in old["faults"]), old["faults"])


REPLAYS = {
    "t3": ROOT / "data/step04b/test3/t3_focus_menu_20260930-223940.jsonl",
    "t5c": ROOT / "data/step04b/test5/t5c_recenter_attempt3_20260930-233358.jsonl",
    "live5": ROOT / "data/checkpoint5/src_live_20261001-151605.jsonl",
}


@unittest.skipUnless(all(p.exists() for p in REPLAYS.values()), "saved recordings are only on the local PC")
class ReplayTests(unittest.TestCase):
    """REPLAY of saved Quest recordings (not live data, not synthetic, not a robot test)."""

    def test_focus_loss_recording_grip_held_from_start(self) -> None:
        import teleop_replay
        o = teleop_replay.replay(REPLAYS["t3"], cfg())
        self.assertEqual((o["packets_rejected"], o["engagements"]), (0, 1))
        tr = [(x["old"], x["new"], x["reason"]) for x in o["transitions"]]
        self.assertEqual(tr[:3], [("WAIT_RELEASE", "ARMED", "valid_release_observed"),
                                  ("ARMED", "ACTIVE", "valid_press_after_valid_release"),
                                  ("ACTIVE", "WAIT_RELEASE", "grip_released")])  # fmt: skip
        t_engage = o["t0"] + int(o["transitions"][1]["t_s"] * 1e9)
        t_release = o["t0"] + int(o["transitions"][2]["t_s"] * 1e9)
        # held grip, system menu (focus loss) and focus return with the grip held: no request at all
        self.assertFalse([c for c in o["commands"] if c[0] < t_engage - 1_000_000])
        self.assertFalse([c for c in o["commands"] if c[0] > t_release + 1_000_000])
        self.assertEqual(len({c[2] for c in o["commands"]}), 1)  # orientation preserved throughout
        self.assertEqual([s[1] for s in o["stops"]], ["grip_released"])

    def test_recenter_recording_latches_and_never_moves(self) -> None:
        import teleop_replay
        o = teleop_replay.replay(REPLAYS["t5c"], cfg())
        self.assertEqual((o["engagements"], o["synthetic_target_requests"], o["final_state"]), (0, 0, "FAULT_LATCHED"))
        self.assertTrue(any("local_space_change" in x for x in o["final_latch"]))

    def test_restart_and_acquisition_pause_recording_with_grip_held(self) -> None:
        import teleop_replay
        o = teleop_replay.replay(REPLAYS["live5"], cfg(), ack_at_s=(5.0, 58.0, 78.0))
        # grip was held for the whole run, across an app restart and an acquisition restart
        self.assertEqual((o["engagements"], o["synthetic_target_requests"], o["stop_requests_recorded"]), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
