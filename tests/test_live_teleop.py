"""Tests for the LIVE teleop program (scripts/live_teleop.py).

NO ROBOT AND NO QUEST ARE INVOLVED. The real SDK code talks to a SYNTHETIC controller on loopback
and the Quest packets are synthetic. The synthetic controller's motion model is an assumption:
these tests show what our program sends and how it reacts, not how the real arm behaves.
"""
from __future__ import annotations

import io
import json
import os
import signal
import socket
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

import quest_transport as qt  # noqa: E402
import xarm_readonly_preflight as rp  # noqa: E402
from fake_xarm_controller import POSE, FakeController  # noqa: E402
from test_quest_pose_transport import feed, hello, pose_pkt, rec_obj, tracker  # noqa: E402
from test_quest_pose_v1 import syn_inputs, syn_pose  # noqa: E402
from test_xarm_readonly_preflight import LoopbackSdkTests, _sdk_available  # noqa: E402

READS = set(rp.READ_REGISTERS)
P0 = (0.20, 1.00, -0.30)


class QuestFeed:
    """SYNTHETIC Quest link client: sends pose packets like the real sender does."""

    def __init__(self, port: int, wait_s: float = 12.0):
        self.sock = None
        t_end = time.monotonic() + wait_s
        while time.monotonic() < t_end:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=2)
                break
            except OSError:
                time.sleep(0.1)
        assert self.sock is not None, "the live program never opened its port"
        self.framer = qt.LineFramer(qt.MAX_MESSAGE_BYTES)
        self.tr, self.seq, self.tq = tracker(), 0, 10**9
        assert self.rpc(hello())["type"] == "hello_ack"

    def rpc(self, pkt: dict) -> dict:
        self.sock.sendall(qt.encode(pkt))
        while True:
            frames = self.framer.feed(self.sock.recv(4096))
            if frames:
                return json.loads(frames[0])

    def pose(self, p=P0, grip=1.0, trig=0.0, q=(0.0, 0.0, 0.0, 1.0), a=False, b=False) -> None:  # type: ignore[no-untyped-def]
        self.seq += 1
        self.tq += 33_000_000
        obj = rec_obj(self.seq, t=self.tq, gr=syn_pose(p, q), in_r=syn_inputs(True, squeeze=grip, trigger=trig, a=a, b=b))
        feed(self.tr, obj, 0)
        assert self.rpc(pose_pkt(self.tr, obj, self.seq))["status"] == "ok"
        time.sleep(0.03)

    def engage(self) -> None:
        for _ in range(5):
            self.pose(grip=0.0)
        for _ in range(3):
            self.pose(grip=1.0)

    def glide(self, dz_local: float, steps: int, start: float = 0.0) -> None:
        for i in range(1, steps + 1):
            self.pose((P0[0], P0[1], P0[2] + start + (dz_local - start) * i / steps))

    def hold(self, dz_local: float, n: int, grip: float = 1.0) -> None:
        for _ in range(n):
            self.pose((P0[0], P0[1], P0[2] + dz_local), grip=grip)


@unittest.skipUnless(_sdk_available(), "xArm SDK not importable here (set XARM_SDK_SRC to a copy of its source)")
class LiveTeleopProgramTests(unittest.TestCase):
    def run_live(self, ctl: FakeController, drive, extra: list | None = None, duration: float = 9.0):  # type: ignore[no-untyped-def]
        import live_teleop as lt
        import xarm_command_link as cl
        undo = LoopbackSdkTests.remap(ctl)
        with socket.socket() as s0:
            s0.bind(("127.0.0.1", 0))
            port = s0.getsockname()[1]
        saved = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGUSR1, signal.SIGUSR2)}
        notes: dict = {}

        def runner() -> None:
            try:
                drive(QuestFeed(port), notes)
            except Exception as e:  # reported to the main thread
                notes["drive_error"] = f"{type(e).__name__}: {e}"

        if drive is not None:
            threading.Thread(target=runner, daemon=True).start()
        real, buf = sys.stdout, io.StringIO()
        sys.stdout = buf
        try:
            with tempfile.TemporaryDirectory() as tmp:
                summ, log = Path(tmp) / "s.json", Path(tmp) / "t.jsonl"
                rc = lt.main(["--address", "127.0.0.1", "--moves-real-arm", "--human-confirmation", "SYNTHETIC controller on loopback",
                              "--port", str(port), "--duration", str(duration), "--summary", str(summ), "--log", str(log)] + (extra or []),
                             sdk=cl.load_sdk())  # fmt: skip
                res = json.loads(summ.read_text()) if summ.exists() else None
                ticks = [json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
        finally:
            sys.stdout = real
            for sig, h in saved.items():
                signal.signal(sig, h)
            undo()
            time.sleep(0.05)
            ctl.close()
        return rc, res, ticks, buf.getvalue(), notes

    def writes(self, ctl: FakeController) -> list:
        return [(reg, p) for _, reg, p in ctl.frames if reg not in READS]

    def test_hand_motion_moves_the_arm_and_the_session_ends_parked(self) -> None:
        ctl = FakeController()

        def drive(q: QuestFeed, notes: dict) -> None:
            q.engage()
            q.glide(-0.10, 40)  # hand 100 mm forward (LOCAL -z) -> tool +25 mm in x at scale 0.25
            q.hold(-0.10, 25)
            notes["pose_while_held"] = list(ctl.pose[:3])
            q.hold(-0.10, 6, grip=0.0)  # clutch released
            notes["moves_at_release"] = sum(1 for _, r, _ in ctl.frames if r == 21)
            q.hold(-0.30, 10, grip=0.0)  # hand moves on while released: nothing may follow
            notes["moves_later"] = sum(1 for _, r, _ in ctl.frames if r == 21)

        rc, res, ticks, text, notes = self.run_live(ctl, drive)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual((rc, res["result"]), (0, "ok"), res.get("abort_reason"))
        self.assertAlmostEqual(notes["pose_while_held"][0], POSE[0] + 25.0, delta=0.6)
        self.assertAlmostEqual(notes["pose_while_held"][1], POSE[1], delta=0.1)
        self.assertAlmostEqual(notes["pose_while_held"][2], POSE[2], delta=0.1)
        self.assertEqual(notes["moves_later"], notes["moves_at_release"])
        self.assertEqual((res["engagements"], res["halts"], res["faults"], res["final_latch"]), (1, 0, [], []))
        self.assertEqual((ctl.mode, ctl.state), (0, 4))  # parked
        w = self.writes(ctl)
        self.assertEqual([(r, p) for r, p in w if r != 21][:3], [(11, bytes([8, 1])), (19, bytes([7, 0])), (12, bytes([0]))])
        self.assertTrue(all(r in (11, 12, 19, 21) for r, _ in w))
        moves = [struct.unpack("<10f", p[:40]) for r, p in w if r == 21]
        self.assertEqual({m[3:6] for m in moves}, {moves[0][3:6]})  # orientation passed back unchanged
        self.assertTrue(all(m[6] == 50.0 for m in moves))
        self.assertEqual(res["before"]["reduced_mode_on"], 1)
        self.assertNotIn("127.0.0.1", json.dumps(res["before"]))
        self.assertIn("ARM LIVE", text)

    def test_thumb_buttons_record_an_episode_with_pose_joints_targets_and_video(self) -> None:
        from fake_xarm_controller import JOINTS
        from test_episode_recorder import FakeCameras

        import episode_inspect as ei
        ctl, cams = FakeController(), FakeCameras()
        tmp = tempfile.TemporaryDirectory()
        sess = Path(tmp.name) / "sess"

        def press(q: QuestFeed, dz: float, grip: float, **button: bool) -> None:
            for held in (True, True, False, False):
                q.pose((P0[0], P0[1], P0[2] + dz), grip=grip, **({k: held for k in button}))

        def drive(q: QuestFeed, notes: dict) -> None:
            for _ in range(40):  # the camera recorder needs a moment before it reports frames
                q.pose(grip=0.0)
            notes["reads_of_joints_before"] = sum(1 for _, r, _ in ctl.frames if r == 42)
            press(q, 0.0, 0.0, a=True)  # A: start
            q.engage()
            q.glide(-0.10, 40)
            q.hold(-0.10, 20)
            press(q, -0.10, 1.0, a=True)  # A again, clutch still held: stop and save
            q.hold(-0.10, 10)
            press(q, -0.10, 1.0, a=True)  # a second episode ...
            q.hold(-0.10, 15)
            press(q, -0.10, 1.0, b=True)  # ... discarded with B
            q.hold(-0.10, 10, grip=0.0)

        try:
            rc, res, ticks, text, notes = self.run_live(ctl, drive, ["--record-dir", str(sess)] + [x for n, u in cams.cams.items()
                                                                                                      for x in ("--record-cam", f"{n}={u}")], duration=14.0)  # fmt: skip
            self.assertNotIn("drive_error", notes, notes)
            self.assertEqual((rc, res["result"]), (0, "ok"), res.get("abort_reason"))
            e = res["episodes"]
            self.assertEqual((e["saved"], e["discarded"], e["refused"], e["error"]), (1, 1, 0, None), text[-1500:])
            self.assertEqual(sorted(p.name for p in sess.iterdir() if p.is_dir()), ["episode_0001"])
            out, problems, steps, _ = ei.check(sess / "episode_0001")
            self.assertEqual(problems, [])
            self.assertGreater(len(steps), 60)
            self.assertTrue(all(r["joints"] == [round(v, 6) for v in JOINTS[:6]] and len(r["tcp_rpy"]) == 3 and len(r["tcp_q"]) == 4 for r in steps))
            self.assertAlmostEqual(steps[-1]["tcp_mm"][0] - steps[0]["tcp_mm"][0], 25.0, delta=0.6)  # the arm's own motion is in the rows
            sent = [r for r in steps if r["cmd_mm"] is not None]
            self.assertGreater(len(sent), 40)
            self.assertTrue(all(r["state"] == "ACTIVE" and len(r["cmd_rpy"]) == 3 for r in sent))
            self.assertTrue(all(c["frames"] > 40 for c in out["cameras"].values()), out)
            meta = json.loads((sess / "session.json").read_text())
            self.assertNotIn("127.0.0.1\"", json.dumps(meta["args"]))
            self.assertEqual(meta["workspace_mm"], res["workspace_mm"])
            self.assertIn("ep=rec#1", text)
            self.assertTrue(all(r in (11, 12, 19, 21) for r, _ in self.writes(ctl)))  # recording adds reads only
            self.assertGreater(sum(1 for _, r, _ in ctl.frames if r == 42), 100)
            self.assertEqual((ctl.mode, ctl.state), (0, 4))
        finally:
            cams.close()
            tmp.cleanup()

    def test_pushing_past_the_box_is_held_at_its_edge_without_a_fault(self) -> None:
        ctl = FakeController()

        def drive(q: QuestFeed, notes: dict) -> None:
            q.engage()
            q.glide(-0.40, 80)  # asks for +100 mm; the box ends at +50 mm
            q.hold(-0.40, 45)
            notes["x"] = ctl.pose[0]
            q.hold(-0.40, 4, grip=0.0)

        rc, res, ticks, _, notes = self.run_live(ctl, drive, duration=11.0)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual((rc, res["faults"], res["halts"]), (0, [], 0))
        self.assertAlmostEqual(notes["x"], POSE[0] + 50.0, delta=0.6)
        self.assertLessEqual(max(struct.unpack("<10f", p[:40])[0] for r, p in self.writes(ctl) if r == 21), POSE[0] + 50.0 + 1e-3)

    def test_controller_error_halts_is_not_cleared_and_rearming_is_refused(self) -> None:
        ctl = FakeController()

        def drive(q: QuestFeed, notes: dict) -> None:
            q.engage()
            q.glide(-0.04, 15)
            ctl.err = 31  # SYNTHETIC controller error while engaged
            q.hold(-0.04, 10)
            os.kill(os.getpid(), signal.SIGUSR1)  # operator: acknowledge and re-arm
            q.hold(-0.04, 25, grip=0.0)
            q.hold(-0.04, 10, grip=1.0)

        rc, res, ticks, text, notes = self.run_live(ctl, drive)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual(res["halts"], 1)
        self.assertTrue(res["faults"][0]["reason"].startswith("feedback_failed:controller error 31"), res["faults"])
        self.assertIn("re-arm refused", text)
        self.assertEqual(res["engagements"], 1)  # never engaged again
        regs = [r for r, _ in self.writes(ctl)]
        self.assertNotIn(16, regs)
        self.assertNotIn(17, regs)
        self.assertEqual(ctl.state, 4)

    def test_fault_then_acknowledge_rearm_release_press_resumes(self) -> None:
        ctl = FakeController()

        def drive(q: QuestFeed, notes: dict) -> None:
            q.engage()
            q.pose((P0[0] + 0.30, P0[1], P0[2]))  # SYNTHETIC 300 mm jump in one sample
            q.hold(0.0, 6)
            notes["halts_after_jump"] = ctl.state
            os.kill(os.getpid(), signal.SIGUSR1)
            q.hold(0.0, 30)  # still held: must not resume
            notes["moves_before_release"] = sum(1 for _, r, _ in ctl.frames if r == 21)
            q.hold(0.0, 5, grip=0.0)
            q.hold(0.0, 5, grip=1.0)
            q.glide(-0.04, 15)
            q.hold(-0.04, 15)
            notes["x"] = ctl.pose[0]

        rc, res, ticks, text, notes = self.run_live(ctl, drive, duration=11.0)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual(notes["halts_after_jump"], 4)
        self.assertTrue(res["faults"][0]["reason"].startswith("controller_discontinuity"))
        self.assertIn("re-armed by the operator", text)
        self.assertEqual((res["engagements"], res["halts"]), (2, 1))
        self.assertAlmostEqual(notes["x"], POSE[0] + 10.0, delta=0.6)

    def test_trigger_drives_the_gripper_during_arm_teleop(self) -> None:
        from test_live_gripper import FakeGripctl  # SYNTHETIC gripper service on loopback
        g = FakeGripctl(ticks=597, open_ticks=512, closed_ticks=2048)  # the calibration after the repair
        self.addCleanup(g.close)
        ctl = FakeController()

        def drive(q: QuestFeed, notes: dict) -> None:
            for _ in range(5):
                q.pose(grip=0.0, trig=0.6)  # clutch released: the trigger must do nothing
            notes["posts_before_engage"] = len(g.posts())
            for _ in range(8):
                q.pose(grip=1.0, trig=0.0)  # engaged, trigger at rest: jaws (5.5 %) are picked up and opened to 0 %
            for i in range(1, 41):
                q.pose((P0[0], P0[1], P0[2] - 0.001 * i), 1.0, i / 40)  # squeeze fully while moving the hand 40 mm
            for _ in range(45):
                q.pose((P0[0], P0[1], P0[2] - 0.04), 1.0, 1.0)
            notes["ticks_closed"], notes["x"] = g.ticks, ctl.pose[0]
            n = len(g.posts())
            for _ in range(12):
                q.pose((P0[0], P0[1], P0[2] - 0.04), 0.0, 0.0)  # clutch released, trigger released: jaws must stay
            notes["posts_after_release"] = len(g.posts()) - n

        rc, res, ticks, text, notes = self.run_live(ctl, drive, ["--with-gripper", "--gripctl", g.url], duration=11.0)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual((rc, res["result"], res["faults"]), (0, "ok", []), res.get("abort_reason"))
        self.assertEqual(notes["posts_before_engage"], 0)
        self.assertAlmostEqual(notes["ticks_closed"], 512 + 0.9 * 1536, delta=20)  # 90 % closed, not beyond
        self.assertLessEqual(max(r[3]["value"] for r in g.posts()), 90.0)
        self.assertLessEqual(notes["posts_after_release"], 1)
        self.assertAlmostEqual(notes["x"], POSE[0] + 10.0, delta=0.6)  # the arm followed at the same time
        self.assertEqual({(r[1], r[2]) for r in g.requests}, {("GET", "/api/calibration"), ("GET", "/api/state"), ("POST", "/api/do/grip")})
        self.assertEqual(res["gripper_calibration"]["open_ticks"], 512)

    def test_wrist_rotation_turns_the_tool_about_the_vertical(self) -> None:
        import math
        import teleop_rotation as tr
        ctl = FakeController()
        q0 = tr.rpy_to_q(*POSE[3:6])
        up = lambda deg: (0.0, math.sin(math.radians(deg) / 2), 0.0, math.cos(math.radians(deg) / 2))  # noqa: E731  hand turn about LOCAL up
        tilt = lambda deg: (math.sin(math.radians(deg) / 2), 0.0, 0.0, math.cos(math.radians(deg) / 2))  # noqa: E731  about LOCAL x

        def drive(q: QuestFeed, notes: dict) -> None:
            for _ in range(5):
                q.pose(grip=0.0)
            for _ in range(3):
                q.pose(grip=1.0)
            for i in range(1, 21):
                q.pose(q=tilt(1.0 * i))  # tilting the hand 20 deg: must do nothing in yaw mode
            notes["after_tilt"] = math.degrees(tr.q_angle(q0, tr.rpy_to_q(*ctl.pose[3:6])))
            for i in range(1, 21):
                q.pose(q=tr.q_mul(up(1.0 * i), tilt(20.0)))  # then turning it 20 deg about up
            for _ in range(30):
                q.pose(q=tr.q_mul(up(20.0), tilt(20.0)))
            notes["q"] = tr.rpy_to_q(*ctl.pose[3:6])
            notes["xyz"] = list(ctl.pose[:3])
            for i in range(1, 31):
                q.pose(q=tr.q_mul(up(20.0 + 2.0 * i), tilt(20.0)))  # on to 80 deg: the limit is 30
            for _ in range(40):
                q.pose(q=tr.q_mul(up(80.0), tilt(20.0)))
            notes["at_limit"] = math.degrees(tr.q_angle(q0, tr.rpy_to_q(*ctl.pose[3:6])))
            for _ in range(4):
                q.pose(grip=0.0, q=tr.q_mul(up(80.0), tilt(20.0)))

        rc, res, ticks, text, notes = self.run_live(ctl, drive, ["--rotation", "yaw"], duration=13.0)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual((rc, res["result"], res["faults"], res["halts"]), (0, "ok", [], 0), res.get("abort_reason"))
        self.assertLess(notes["after_tilt"], 0.05)
        self.assertLess(math.degrees(tr.q_angle(notes["q"], tr.q_mul(tr.q_from_rotvec((0.0, 0.0, math.radians(20.0))), q0))), 0.3)  # 20 deg about base z
        for k in range(3):
            self.assertAlmostEqual(notes["xyz"][k], POSE[k], delta=0.05)  # the flange stayed put
        self.assertAlmostEqual(notes["at_limit"], 30.0, delta=0.3)
        self.assertLess(res["before"]["rpy_convention_error_deg"], 0.01)
        self.assertEqual(res["config"]["rotation"]["mode"], "yaw")

    def test_wrist_rotation_is_refused_when_the_angle_convention_does_not_match(self) -> None:
        ctl = FakeController()
        ctl.aa_wrong = True
        rc, res, _, _, _ = self.run_live(ctl, None, ["--rotation", "full"], duration=5.0)
        self.assertEqual((rc, res["result"]), (3, "aborted"))
        self.assertIn("convention", res["abort_reason"])
        self.assertEqual(self.writes(ctl), [])
        ctl2 = FakeController()
        ctl2.aa_wrong = True  # without wrist rotation the convention is not used, so the session may run
        rc, res, _, _, _ = self.run_live(ctl2, None, [], duration=5.0)
        self.assertEqual(rc, 0)

    def test_headset_facing_the_operator_flips_the_horizontal_directions(self) -> None:
        import live_teleop as lt
        import teleop_mapping as tm
        self.assertEqual(lt.mapping_for_operator(0.0), [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        r180 = tm.validate_rotation(lt.mapping_for_operator(180.0))  # still a proper rotation
        self.assertEqual([[round(v) for v in row] for row in r180], [[0, 0, 1], [1, 0, 0], [0, 1, 0]])  # LOCAL +z -> +x, LOCAL +x -> +y, up stays up
        r90 = tm.validate_rotation(lt.mapping_for_operator(90.0))  # operator faces the headset's left: LOCAL -x is "forward"
        self.assertEqual([[round(v) for v in row] for row in r90], [[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
        ctl = FakeController()

        def drive(q: QuestFeed, notes: dict) -> None:
            q.engage()
            q.glide(+0.10, 40)  # the hand moves 100 mm AWAY from the operator = toward the headset's back = LOCAL +z
            q.hold(+0.10, 25)
            notes["pose"] = list(ctl.pose[:3])
            q.hold(+0.10, 4, grip=0.0)

        rc, res, _, _, notes = self.run_live(ctl, drive, ["--operator-yaw-deg", "180"])
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual((rc, res["faults"]), (0, []))
        self.assertAlmostEqual(notes["pose"][0], POSE[0] + 25.0, delta=0.6)  # the arm moves away from its base (+x)
        self.assertAlmostEqual(notes["pose"][1], POSE[1], delta=0.1)

    def test_absolute_box_is_used_as_given_and_held_at_its_edges(self) -> None:
        ctl = FakeController()  # starts at x 299.8, y 3.4, z 461.2; fence x 160..502, y -300..270, z 270..585

        def drive(q: QuestFeed, notes: dict) -> None:
            q.engage()
            q.glide(-0.60, 90)  # asks for +150 mm in x; the box ends 40.2 mm ahead
            q.hold(-0.60, 40)
            notes["x"] = ctl.pose[0]
            q.hold(-0.60, 4, grip=0.0)

        rc, res, _, text, notes = self.run_live(ctl, drive, ["--box-abs", "200,340,-290,260,280,575"], duration=11.0)
        self.assertNotIn("drive_error", notes, notes)
        self.assertEqual((rc, res["faults"], res["halts"]), (0, [], 0))
        self.assertEqual(res["workspace_mm"]["min"], [200.0, -290.0, 280.0])  # down to 10 mm above the fence floor
        self.assertEqual(res["workspace_mm"]["max"], [340.0, 260.0, 575.0])
        self.assertAlmostEqual(notes["x"], 340.0, delta=0.6)
        self.assertLessEqual(max(struct.unpack("<10f", p[:40])[0] for r, p in self.writes(ctl) if r == 21), 340.0 + 1e-3)
        self.assertIn("(absolute)", text)

    def test_absolute_box_outside_the_fence_or_not_containing_the_arm_commands_nothing(self) -> None:
        for box, why in (("200,340,-290,260,275,575", "fence"),  # floor 5 mm above the fence floor: inside the margin
                         ("200,340,-290,265,280,575", "fence"),
                         ("310,400,-100,100,300,500", "not inside the requested box"),  # the arm is at x 299.8
                         ("200,340,-100,100,300,461.5", "not inside the requested box")):  # 0.3 mm below the top: too close
            ctl = FakeController()
            rc, res, _, _, _ = self.run_live(ctl, None, ["--box-abs", box], duration=5.0)
            self.assertEqual((rc, res["result"]), (3, "aborted"), box)
            self.assertIn(why, res["abort_reason"])
            self.assertEqual(self.writes(ctl), [], box)
        ctl = FakeController()  # the same floor is accepted with a smaller margin, and refused again when even that is not kept
        rc, res, _, _, _ = self.run_live(ctl, None, ["--box-abs", "200,340,-290,260,275,575", "--fence-margin-mm", "5"], duration=5.0)
        self.assertEqual((rc, res["workspace_mm"]["min"][2]), (0, 275.0))
        ctl = FakeController()
        rc, res, _, _, _ = self.run_live(ctl, None, ["--box-abs", "200,340,-290,260,274,575", "--fence-margin-mm", "5"], duration=5.0)
        self.assertEqual((rc, self.writes(ctl)), (3, []))
        self.assertIn("within 5 mm of the fence", res["abort_reason"])
        import live_teleop as lt
        real, sys.stdout = sys.stdout, io.StringIO()
        try:
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--fence-margin-mm", "1"]), 2)
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--box-abs", "200,210,-100,100,300,500"]), 2)  # 10 mm wide
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--box-abs", "200,340,-100,100,300"]), 2)
            with self.assertRaises(SystemExit):
                lt.main(["--address", "127.0.0.1", "--box-abs", "200,340,-100,100,300,500", "--box", "50,50,50,50,10,60"])
        finally:
            sys.stdout = real

    def test_reduced_mode_speed_limits_below_the_session_speeds_command_nothing(self) -> None:
        """Rig, 2026-10-12: reduced mode had been switched on (TCP 50 mm/s) and a 120 mm/s session ended in controller error 24."""
        fast = ["--v-max", "120", "--ctrl-speed", "150"]
        for prep, extra, why in ((lambda c: setattr(c, "reduced_tcp_mm_s", 50.0), fast, "TCP speed limit of 50 mm/s"),
                                 (lambda c: setattr(c, "reduced_joint_deg_s", 20.0), ["--rotation", "full", "--rot-speed", "20"], "joint speed limit of 20 deg/s"),
                                 (lambda c: setattr(c, "reduced_joint_deg_s", 20.0), ["--rotation", "yaw", "--rot-speed", "9"], "joint speed limit of 20 deg/s")):  # fmt: skip
            ctl = FakeController()
            prep(ctl)
            rc, res, _, _, _ = self.run_live(ctl, None, extra, duration=5.0)
            self.assertEqual((rc, res["result"]), (3, "aborted"), why)
            self.assertIn(why, res["abort_reason"])
            self.assertEqual(self.writes(ctl), [], why)
        ctl = FakeController()  # the same limits are fine for a session that stays below them
        ctl.reduced_tcp_mm_s, ctl.reduced_joint_deg_s = 50.0, 20.0
        rc, res, _, text, _ = self.run_live(ctl, None, ["--v-max", "40", "--ctrl-speed", "50", "--rotation", "full", "--rot-speed", "8"], duration=5.0)
        self.assertEqual((rc, res["before"]["reduced_max_tcp_speed_mm_s"], res["before"]["reduced_max_joint_speed_deg_s"]), (0, 50.0, 20.0))
        self.assertIn("reduced mode is ON: TCP limit 50.0 mm/s", text)
        ctl = FakeController()  # reduced mode off (operator's waiver): its limits do not apply
        ctl.reduced_on, ctl.reduced_tcp_mm_s = False, 50.0
        rc, res, _, _, _ = self.run_live(ctl, None, fast + ["--allow-reduced-mode-off"], duration=5.0)
        self.assertEqual((rc, res["before"]["reduced_mode_waived_by_operator"]), (0, True))

    def test_refusals_before_anything_is_commanded(self) -> None:
        import live_teleop as lt
        real, sys.stdout = sys.stdout, io.StringIO()
        try:
            self.assertEqual(lt.main(["--address", "127.0.0.1"]), 0)  # plan only
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--moves-real-arm"]), 2)  # no confirmation
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--v-max", "200"]), 2)
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--box", "50,50,50,50,10,500"]), 2)
            self.assertEqual(lt.main(["--address", "127.0.0.1", "--ctrl-speed", "250", "--v-max", "100"]), 2)
        finally:
            sys.stdout = real
        for prep, extra, why in ((lambda c: setattr(c, "reduced_on", False), [], "reduced mode"),
                                 (lambda c: setattr(c, "sim", True), [], "simulation"),
                                 (lambda c: setattr(c, "err", 22), [], "error 22"),
                                 (lambda c: None, ["--box", "350,50,50,50,10,60"], None)):  # fmt: skip
            ctl = FakeController()
            prep(ctl)
            rc, res, _, _, _ = self.run_live(ctl, None, extra, duration=5.0)
            if why is None:
                self.assertEqual(rc, 2)  # argument range
            else:
                self.assertEqual((rc, res["result"]), (3, "aborted"), why)
                self.assertIn(why, res["abort_reason"])
            self.assertEqual(self.writes(ctl), [], why)
        ctl = FakeController()
        ctl.reduced_on = False
        rc, res, _, _, _ = self.run_live(ctl, None, ["--allow-reduced-mode-off"], duration=5.0)
        self.assertEqual((rc, res["before"]["reduced_mode_waived_by_operator"], ctl.mode, ctl.state), (0, True, 0, 4))
        ctl = FakeController(pose=[190.0, 0.0, 400.0, POSE[3], POSE[4], POSE[5]])  # 30 mm from the fence x-min of 160
        rc, res, _, _, _ = self.run_live(ctl, None, [], duration=5.0)
        self.assertEqual(rc, 3)
        self.assertIn("fence", res["abort_reason"])
        self.assertEqual(self.writes(ctl), [])


if __name__ == "__main__":
    unittest.main()
