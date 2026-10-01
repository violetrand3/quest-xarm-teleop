"""Tests for the real-gripper path (gripctl client, adapter executor hook, live entry point).

NO REAL GRIPPER IS USED. The "gripctl" here is a SYNTHETIC HTTP server on loopback that records
every request and moves a pretend jaw instantly. It shows what our program sends; it shows
nothing about the real servo, its spring, its force or its dropouts.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import gripctl_client as gc  # noqa: E402
import quest_transport as qt  # noqa: E402
import teleop_adapter as ta  # noqa: E402
import teleop_backend as tb  # noqa: E402
import teleop_mapping as tm  # noqa: E402
from test_quest_pose_transport import feed, hello, pose_pkt, rec_obj, tracker  # noqa: E402
from test_quest_pose_v1 import syn_inputs, syn_pose  # noqa: E402
from test_teleop_adapter import CONFIG, P0, Rig  # noqa: E402


class FakeGripctl:
    """SYNTHETIC gripctl: GET /api/state, POST /api/do/grip. Everything else is recorded and refused."""

    def __init__(self, ticks: int = 1950):
        self.ticks, self.connected, self.fail_grip = ticks, True, False
        self.requests: list = []  # (monotonic s, method, path, body)
        outer = self

        class H(BaseHTTPRequestHandler):
            def _send(self, code: int, obj: dict) -> None:
                b = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

            def do_GET(self) -> None:  # noqa: N802
                outer.requests.append((time.monotonic(), "GET", self.path, None))
                if self.path == "/api/state":
                    self._send(200, {"ok": True, "age": 0.02, "status": {"connected": outer.connected},
                                     "values": {"position": outer.ticks if outer.connected else None, "goal_position": outer.ticks,
                                                "load": 0.0, "moving": 0, "torque_enable": 1}})  # fmt: skip
                else:
                    self._send(404, {"ok": False})

            def do_POST(self) -> None:  # noqa: N802
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
                outer.requests.append((time.monotonic(), "POST", self.path, body))
                if self.path != "/api/do/grip":
                    self._send(404, {"ok": False, "error": "unexpected"})
                elif outer.fail_grip or not outer.connected:
                    self._send(400, {"ok": False, "error": "not connected"})
                else:
                    src = outer.ticks
                    outer.ticks = int(round(3033 - 18.52 * body["value"]))
                    self._send(200, {"ok": True, "error": None, "grip": {"percent": body["value"], "target": outer.ticks, "from": src,
                                                                         "direction": "closing" if outer.ticks < src else "opening"}})  # fmt: skip

            def log_message(self, *a) -> None:  # type: ignore[no-untyped-def]
                pass

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_address[1]}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def posts(self) -> list:
        return [r for r in self.requests if r[1] == "POST"]

    def close(self) -> None:
        self.srv.shutdown()
        self.srv.server_close()


def prop(pct: float) -> tb.GripperProposal:
    return tb.GripperProposal(pct, tm.pct_closed_to_ticks(pct), 0.5)


def wait(cond, timeout: float = 2.0) -> bool:  # type: ignore[no-untyped-def]
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


class GripctlClientTests(unittest.TestCase):
    def setUp(self) -> None:
        self.g = FakeGripctl()
        self.addCleanup(self.g.close)

    def client(self, **kw) -> gc.GripctlClient:  # type: ignore[no-untyped-def]
        c = gc.GripctlClient(self.g.url, min_interval_s=kw.pop("min_interval_s", 0.05), state_period_s=0.01, **kw)
        self.addCleanup(c.close)
        return c

    def test_only_the_loopback_service_is_accepted(self) -> None:
        for bad in ("http://192.168.1.50:8088", "http://localhost:8088", "https://127.0.0.1:8088", "http://127.0.0.1:8088/api", "http://127.0.0.1"):
            with self.assertRaises(ValueError):
                gc.GripctlClient(bad)

    def test_state_units_and_feedback(self) -> None:
        c = self.client()
        c.refresh_state()
        fb = c.read()
        self.assertTrue(fb.connected and not fb.synthetic)
        self.assertAlmostEqual(fb.pct_closed, (3033 - 1950) / 1852 * 100, places=6)  # 58.48 % closed
        self.assertAlmostEqual(gc.ticks_to_pct_closed(3033), 0.0)
        self.assertAlmostEqual(gc.ticks_to_pct_closed(1181), 100.0)

    def test_sends_newest_only_rate_limited_and_with_a_minimum_step(self) -> None:
        events: list = []
        c = self.client(min_interval_s=0.15, on_event=lambda k, r: events.append((k, r)))
        c.start()
        self.assertTrue(wait(lambda: c.read().connected))
        for p in (60.0, 61.0, 62.0, 63.0):  # four proposals before the worker can send twice
            c.submit(prop(p))
        self.assertTrue(wait(lambda: len(self.g.posts()) >= 1))
        time.sleep(0.4)
        vals = [r[3]["value"] for r in self.g.posts()]
        self.assertEqual(vals, [63.0])  # the newest one, once; nothing queued
        c.submit(prop(63.4))  # below the 1 % step: not sent
        time.sleep(0.3)
        self.assertEqual(len(self.g.posts()), 1)
        c.submit(prop(70.0))
        self.assertTrue(wait(lambda: len(self.g.posts()) == 2))
        t = [r[0] for r in self.g.posts()]
        self.assertGreaterEqual(t[1] - t[0], 0.15)
        self.assertEqual({(r[1], r[2]) for r in self.g.requests}, {("GET", "/api/state"), ("POST", "/api/do/grip")})
        self.assertTrue(all(e[1]["ok"] for e in events if e[0] == "gripper_command"))

    def test_nothing_is_sent_when_the_servo_is_disconnected_or_state_unknown(self) -> None:
        c = self.client()
        c.submit(prop(70.0))  # worker not started, no state yet
        c._maybe_send()
        self.assertEqual(self.g.posts(), [])
        self.g.connected = False
        c.start()
        time.sleep(0.15)
        fb = c.read()
        self.assertEqual((fb.connected, fb.pct_closed), (False, None))
        c.submit(prop(70.0))
        time.sleep(0.2)
        self.assertEqual(self.g.posts(), [])
        self.g.connected = True  # the servo comes back: the old proposal was dropped, not sent late
        self.assertTrue(wait(lambda: c.read().connected))
        time.sleep(0.2)
        self.assertEqual(self.g.posts(), [])

    def test_cancel_and_close_send_nothing(self) -> None:
        c = self.client(min_interval_s=0.3)
        c.start()
        self.assertTrue(wait(lambda: c.read().connected))
        c.submit(prop(65.0))
        self.assertTrue(wait(lambda: len(self.g.posts()) == 1))
        c.submit(prop(80.0))  # waits for the rate limit...
        c.cancel()  # ...and is cancelled first
        time.sleep(0.5)
        self.assertEqual(len(self.g.posts()), 1)
        c.submit(prop(90.0))
        c.close()
        time.sleep(0.4)
        self.assertEqual(len(self.g.posts()), 1)

    def test_failed_command_marks_the_gripper_unavailable(self) -> None:
        c = self.client()
        c.refresh_state()
        self.g.fail_grip = True
        c.submit(prop(70.0))
        c._maybe_send()
        self.assertEqual((c.sent, c.send_errors), (0, 1))
        self.assertFalse(c.read().connected)

    def test_stale_state_is_not_connected(self) -> None:
        now = [10**9]
        c = gc.GripctlClient(self.g.url, clock=lambda: now[0], stale_s=0.4)
        c.refresh_state()
        self.assertTrue(c.read().connected)
        now[0] += 500_000_000
        self.assertFalse(c.read().connected)
        c.submit(prop(70.0))
        c._maybe_send()
        self.assertEqual(self.g.posts(), [])


class FakeExecutor:
    def __init__(self) -> None:
        self.submitted: list = []
        self.cancels = 0

    def submit(self, p) -> None:  # type: ignore[no-untyped-def]
        self.submitted.append(p.pct_closed)

    def cancel(self) -> None:
        self.cancels += 1


class AdapterExecutorTests(unittest.TestCase):
    def rig(self, jaws_pct: float = 58.5):  # type: ignore[no-untyped-def]
        r = Rig()
        ex = FakeExecutor()
        r.ad.gripper_executor = ex
        r.ad.follower = tm.GripperFollower(8.0, 50.0, 15.0, 85.0)
        fb = {"pct": jaws_pct, "connected": True}
        r.backend.read_gripper = lambda: tb.GripperFeedback(fb["pct"] if fb["connected"] else None, fb["connected"], r.now, True)  # type: ignore[method-assign]
        return r, ex, fb

    def test_default_adapter_has_no_executor(self) -> None:
        self.assertIsNone(Rig().ad.gripper_executor)

    def test_trigger_range_keeps_off_the_end_stops(self) -> None:
        self.assertEqual([tm.trigger_to_pct_closed(v, 15.0, 85.0) for v in (0.0, 0.5, 1.0)], [15.0, 50.0, 85.0])
        self.assertIsNone(tm.trigger_to_pct_closed(None, 15.0, 85.0))
        with self.assertRaises(tm.ConfigError):
            tm.GripperFollower(8.0, 50.0, 60.0, 40.0)

    def test_nothing_is_submitted_before_engagement_or_before_pickup(self) -> None:
        r, ex, _ = self.rig()
        for _ in range(10):
            r.step(P0, 0.0, trigger=0.62)  # clutch released: trigger at the jaw position, still nothing
        self.assertEqual(ex.submitted, [])
        r.engage()
        for _ in range(10):
            out = r.step(P0, 1.0, trigger=0.0)  # 15 % wanted, jaws at 58.5 %: far outside the pickup window
        self.assertEqual((out["gripper"], ex.submitted), ("waiting_pickup", []))

    def test_pickup_then_rate_limited_following_and_no_jump(self) -> None:
        r, ex, _ = self.rig()
        r.engage()
        out = r.step(P0, 1.0, trigger=0.62)  # 58.4 % wanted: pickup
        self.assertEqual(out["gripper"], "following")
        self.assertAlmostEqual(ex.submitted[0], 58.4, places=1)
        for _ in range(25):  # 0.5 s with the trigger fully pressed
            out = r.step(P0, 1.0, trigger=1.0)
        steps = [b - a for a, b in zip(ex.submitted, ex.submitted[1:])]
        self.assertLessEqual(max(steps), 50.0 * 0.02 + 1e-6)  # 50 %/s at 20 ms ticks
        self.assertLessEqual(max(ex.submitted), 85.0)
        self.assertAlmostEqual(ex.submitted[-1], 58.5 + 25.0, delta=1.5)
        self.assertEqual(out["gripper_fb_pct"], 58.5)

    def test_clutch_release_cancels_and_sends_nothing_more(self) -> None:
        r, ex, _ = self.rig()
        r.engage()
        r.step(P0, 1.0, trigger=0.62)
        r.step(P0, 1.0, trigger=0.8)
        n = len(ex.submitted)
        r.step(P0, 0.0, trigger=0.8)  # clutch released
        self.assertEqual(ex.cancels, 1)
        for _ in range(20):
            r.step(P0, 0.0, trigger=1.0)
        self.assertEqual(len(ex.submitted), n)

    def test_input_loss_cancels_and_recovery_with_clutch_held_does_not_resume(self) -> None:
        r, ex, _ = self.rig()
        r.engage()
        r.step(P0, 1.0, trigger=0.62)
        n = len(ex.submitted)
        r.step(P0, 1.0, trigger=0.9, state="VISIBLE", focused=False)  # focus lost
        self.assertEqual(ex.cancels, 1)
        for _ in range(20):
            r.step(P0, 1.0, trigger=0.9)  # input back, clutch still held
        self.assertEqual(len(ex.submitted), n)

    def test_gripper_dropout_stops_following_and_needs_a_new_pickup(self) -> None:
        r, ex, fb = self.rig()
        r.engage()
        r.step(P0, 1.0, trigger=0.62)
        r.step(P0, 1.0, trigger=0.7)
        n = len(ex.submitted)
        fb["connected"] = False  # SYNTHETIC USB dropout
        out = r.step(P0, 1.0, trigger=0.9)
        self.assertEqual((out["gripper"], out["state"], ex.cancels), ("no_gripper_feedback", ta.ACTIVE, 1))
        fb["connected"], fb["pct"] = True, 64.0  # back, jaws where the last command left them
        for _ in range(10):
            out = r.step(P0, 1.0, trigger=0.9)  # 78 % wanted, jaws at 64 %: no jump, waits for pickup
        self.assertEqual((out["gripper"], len(ex.submitted)), ("waiting_pickup", n))
        out = r.step(P0, 1.0, trigger=0.72)  # 65.4 %: within 8 % of the jaws
        self.assertEqual(out["gripper"], "following")


class LiveEntryPointTests(unittest.TestCase):
    def test_refuses_without_explicit_flags_and_sends_nothing(self) -> None:
        import live_gripper_test
        g = FakeGripctl()
        self.addCleanup(g.close)
        for extra in ([], ["--moves-real-gripper"], ["--human-confirmation", "someone said it is fine"]):
            self.assertEqual(live_gripper_test.main(["--config", str(CONFIG), "--gripctl", g.url, "--port", "0", "--duration", "0.2"] + extra), 2)
        self.assertEqual(g.requests, [])

    def test_dry_run_cannot_reach_the_gripper_client(self) -> None:
        dry = (ROOT / "scripts" / "teleop_dryrun.py").read_text()
        for name in ("gripctl_client", "live_gripper_test", "gripper_executor", "8088"):
            self.assertNotIn(name, dry)
        for f in sorted((ROOT / "scripts").glob("teleop_*.py")):
            self.assertNotIn("gripctl_client", f.read_text(), f.name)

    def test_trigger_drives_the_fake_jaws_end_to_end_and_restores_them(self) -> None:
        g = FakeGripctl(ticks=1950)
        self.addCleanup(g.close)
        with socket.socket() as s0:
            s0.bind(("127.0.0.1", 0))
            port = s0.getsockname()[1]
        with tempfile.TemporaryDirectory() as tmp:
            summ, log = Path(tmp) / "s.json", Path(tmp) / "a.jsonl"
            proc = subprocess.Popen([sys.executable, str(ROOT / "scripts" / "live_gripper_test.py"), "--config", str(CONFIG),
                                     "--moves-real-gripper", "--human-confirmation", "SYNTHETIC TEST, fake gripctl on loopback",
                                     "--gripctl", g.url, "--port", str(port), "--summary", str(summ), "--log", str(log)],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)  # fmt: skip
            t_release = None
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

                tr, seq, tq = tracker(), [0], [10**9]

                def pose(grip: float, trig: float) -> None:
                    seq[0] += 1
                    tq[0] += 33_000_000
                    obj = rec_obj(seq[0], t=tq[0], gr=syn_pose(P0), in_r=syn_inputs(True, squeeze=grip, trigger=trig, a=False, b=False))
                    feed(tr, obj, 0)
                    self.assertEqual(rpc(pose_pkt(tr, obj, seq[0]))["status"], "ok")
                    time.sleep(0.03)

                self.assertEqual(rpc(hello())["type"], "hello_ack")
                for _ in range(5):
                    pose(0.0, 0.0)  # clutch released
                for _ in range(8):
                    pose(1.0, 0.0)  # clutch engaged, trigger released: 15 % wanted, jaws at 58.5 % -> no command
                self.assertEqual(g.posts(), [])
                for i in range(50):
                    pose(1.0, min(1.0, i / 40))  # slow squeeze to the end
                for _ in range(40):
                    pose(1.0, 1.0)
                n_active = len(g.posts())
                t_release = time.monotonic()
                for _ in range(15):
                    pose(0.0, 1.0)  # clutch released with the trigger still pressed
                for _ in range(15):
                    pose(0.0, 0.0)
                time.sleep(0.3)
                n_after = len(g.posts())
                sock.close()
                time.sleep(0.2)
            finally:
                proc.send_signal(signal.SIGINT)
                out, _ = proc.communicate(timeout=15)
            o = json.loads(summ.read_text())
        self.assertEqual(proc.returncode, 0, out[-600:])
        vals = [r[3]["value"] for r in g.posts()]
        self.assertGreaterEqual(n_active, 5)
        self.assertLessEqual(abs(vals[0] - 58.48), 8.0 + 3.0)  # first command near the jaws: no jump
        self.assertEqual(vals[:n_active], sorted(vals[:n_active]))  # closing monotonically
        self.assertLessEqual(max(vals), 85.0)
        self.assertGreater(vals[n_active - 1], 80.0)
        self.assertLessEqual(n_after - n_active, 1)  # at most the command already in flight at the release
        self.assertFalse([r for r in g.posts()[:-1] if r[0] > t_release + 0.25])
        self.assertAlmostEqual(vals[-1], 58.48, places=1)  # the deliberate end-of-test restore
        self.assertEqual(g.ticks, 1950)
        self.assertEqual({(r[1], r[2]) for r in g.requests}, {("GET", "/api/state"), ("POST", "/api/do/grip")})
        self.assertEqual((o["robot_arm_connection_attempted"], o["gripper"]["restore"]["done"], o["gripper"]["final"]["ticks"]), (False, True, 1950))
        self.assertIn("GRIPPER LIVE", out)


if __name__ == "__main__":
    unittest.main()
