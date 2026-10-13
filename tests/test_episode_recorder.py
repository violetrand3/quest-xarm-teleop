"""Tests for episode recording (episode_recorder.py, episode_cameras.py, episode_inspect.py).

NO CAMERA, ARM OR GRIPPER IS USED. The "camera" is a SYNTHETIC MJPEG server on loopback that sends
small fake JPEGs; the tick records are made up here. This shows what our programs write and when;
it shows nothing about the real cameras' timing.
"""
from __future__ import annotations

import json
import math
import socket
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

import episode_inspect as ei  # noqa: E402
import episode_recorder as er  # noqa: E402
from teleop_rotation import rpy_to_q  # noqa: E402

DOWN = rpy_to_q(math.pi, 0.0, 0.0)  # tool pointing down: tool x = base x, tool y = -base y, tool z = -base z


class FakeCameras:
    """SYNTHETIC camera wall: /cam/<name> streams fake JPEGs with Content-Length, about 50 per second."""

    def __init__(self) -> None:
        self.on = True
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                n = 0
                try:
                    while outer.on:
                        n += 1
                        jpeg = b"\xff\xd8" + f"{self.path} frame {n} ".encode() * 20 + b"\xff\xd9"
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
                        time.sleep(0.02)
                except OSError:
                    pass

            def log_message(self, *a) -> None:  # type: ignore[no-untyped-def]
                pass

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        port = self.srv.server_address[1]
        self.cams = {"wrist": f"http://127.0.0.1:{port}/cam/wrist", "fixed": f"http://127.0.0.1:{port}/cam/fixed"}

    def close(self) -> None:
        self.on = False
        self.srv.shutdown()
        self.srv.server_close()


class EpisodeRecorderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.cam = FakeCameras()
        self.logs: list = []
        self.z = 400.0
        self.stale_arm = False
        self.rec: er.EpisodeRecorder | None = None

    def tearDown(self) -> None:
        if self.rec is not None and self.rec.child.poll() is None:
            self.rec.close()
        self.cam.close()
        self.tmp.cleanup()

    def make(self, **kw) -> er.EpisodeRecorder:  # type: ignore[no-untyped-def]
        def extras() -> dict:
            return {"arm": {"pose": (300.0, -70.0, self.z, math.pi, 0.0, 0.0), "joints": (0.1, 0.2, 0.3, 0.4, 0.5, 0.6), "state": 1,
                            "t": time.monotonic_ns(), "read_ms": 1.0},
                    "gripper": {"connected": True, "pct": 10.0 + (self.z - 400.0), "ticks": 600, "load": 3, "read_ns": time.monotonic_ns()}}  # fmt: skip

        self.rec = er.EpisodeRecorder(Path(self.tmp.name) / "sess", {"tool": "test"}, kw.pop("cams", self.cam.cams), extras,
                                      log=self.logs.append, **kw)  # fmt: skip
        end = time.monotonic() + 8.0
        while self.rec.status().split()[0] != "idle" and time.monotonic() < end:
            time.sleep(0.05)
        return self.rec

    def ticks(self, n: int, a=False, b=False, fb=True, state="ACTIVE") -> None:  # type: ignore[no-untyped-def]
        """n made-up 20 ms ticks; the pretend arm rises 0.5 mm per tick."""
        for _ in range(n):
            self.z += 0.5
            self.rec.on_record({"k": "tick", "t": time.monotonic_ns(), "state": state, "btn_a": a, "btn_b": b, "dt_ms": 20.0,
                                "fb_mm": [300.0, -70.0, self.z] if fb else None, "fb_q": list(DOWN) if fb else None,
                                "cmd_mm": [300.0, -70.0, self.z + 1.0], "cmd_ori": [math.pi, 0.0, 0.0], "gripper_pct": None,
                                "trig": 0.2, "grip": 1.0, "ctrl_mm": [0.0, 0.0, 0.0], "ctrl_q": [0.0, 0.0, 0.0, 1.0], "input_age_ms": 12.0})  # fmt: skip
            time.sleep(0.02)

    def press(self, button: str) -> None:
        self.ticks(2, **{button: True})
        self.ticks(2)

    def wait(self, word: str, s: float = 6.0) -> str:
        end = time.monotonic() + s
        while time.monotonic() < end and not self.rec.status().startswith(word):
            time.sleep(0.02)
        return self.rec.status()

    def test_button_a_starts_and_saves_an_episode_with_rows_and_video_on_one_clock(self) -> None:
        r = self.make()
        self.ticks(3)
        self.press("a")
        self.assertTrue(self.wait("rec#1").startswith("rec#1"), self.logs)
        r.on_record({"k": "transition", "t": time.monotonic_ns(), "old": "ARMED", "new": "ACTIVE", "reason": "valid_press_after_valid_release", "latch": []})
        self.ticks(50)
        self.press("a")
        self.assertIn("saved=1", self.wait("idle"))
        ep = Path(self.tmp.name) / "sess" / "episode_0001"
        meta = json.loads((ep / "meta.json").read_text())
        steps = ei.rows(ep / "steps.jsonl")
        self.assertEqual((meta["outcome"], meta["closed_by"], meta["problems"], meta["engagements_started"]), ("saved", "operator", [], 1))
        self.assertEqual(meta["steps"], len(steps))
        self.assertGreater(len(steps), 50)
        row = steps[10]
        self.assertEqual(set(row), set(er.FIELDS))
        self.assertEqual((row["tcp_mm"][:2], row["tcp_rpy"], row["joints"]), ([300.0, -70.0], [round(math.pi, 6), 0.0, 0.0], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))
        self.assertEqual(row["cmd_mm"][2], row["tcp_mm"][2] + 1.0)
        self.assertEqual(json.loads((ep / "events.jsonl").read_text().splitlines()[0])["new"], "ACTIVE")
        for name in ("wrist", "fixed"):
            fr = ei.rows(ep / f"cam_{name}.frames.jsonl")
            self.assertGreater(len(fr), 40)
            self.assertTrue(all(meta["t0"] <= f["t"] <= meta["t1"] for f in fr))
            self.assertLess(fr[0]["t"] - meta["t0"], 100e6)  # frames from before the start command reached the child are kept
            data = (ep / f"cam_{name}.mjpeg").read_bytes()
            self.assertTrue(all(data[f["off"]:f["off"] + 2] == b"\xff\xd8" and data[f["off"] + f["len"] - 2:f["off"] + f["len"]] == b"\xff\xd9" for f in fr))
            self.assertIn(f"/cam/{name} frame".encode(), data)
        out, problems, _, _ = ei.check(ep)
        self.assertEqual(problems, [])
        self.assertEqual(out["joints"], len(steps))
        sess = json.loads((Path(self.tmp.name) / "sess" / "session.json").read_text())
        self.assertEqual((sess["format"], sess["tool"], set(sess["steps_fields"])), (er.FORMAT, "test", set(er.FIELDS)))

    def test_a_held_or_missing_button_never_starts_an_episode(self) -> None:
        self.make()
        self.ticks(10, a=True)  # held from the start: no release was seen
        self.ticks(3, a=None)
        self.ticks(10, a=True)  # back after a gap, still held
        time.sleep(0.3)
        self.assertTrue(self.rec.status().startswith("idle saved=0"), self.rec.status())
        self.ticks(2)
        self.press("a")
        self.assertTrue(self.wait("rec#1").startswith("rec#1"))

    def test_button_b_discards_and_the_number_is_used_again(self) -> None:
        self.make()
        self.ticks(2)
        self.press("a")
        self.wait("rec#1")
        self.ticks(15)
        self.press("b")
        self.assertIn("saved=0 disc=1", self.wait("idle"))
        self.assertEqual([p.name for p in (Path(self.tmp.name) / "sess").glob("episode_*")], [])
        self.press("b")  # nothing is being recorded: ignored
        self.press("a")
        self.assertTrue(self.wait("rec#1").startswith("rec#1"))
        self.ticks(10)
        self.rec.request("stop")
        self.assertIn("saved=1 disc=1", self.wait("idle"))

    def test_no_episode_starts_without_cameras_or_disk_space(self) -> None:
        self.make(min_free_gb=1e9)
        self.ticks(2)
        self.press("a")
        time.sleep(0.3)
        self.assertIn("ref=1", self.rec.status())
        self.assertIn("GB free", self.rec.last_refusal)
        self.rec.close()
        self.tmp.cleanup()
        self.tmp = tempfile.TemporaryDirectory()
        with socket.socket() as s0:  # a loopback port with nothing listening
            s0.bind(("127.0.0.1", 0))
            dead = {"wrist": f"http://127.0.0.1:{s0.getsockname()[1]}/cam/none"}
        self.rec = er.EpisodeRecorder(Path(self.tmp.name) / "sess", {}, dead, None, log=self.logs.append)
        time.sleep(2.5)
        self.assertTrue(self.rec.status().startswith("nocam"), self.rec.status())
        self.ticks(2)
        self.press("a")
        time.sleep(0.3)
        self.assertIn("ref=1", self.rec.status())
        self.assertIn("not delivering", self.rec.last_refusal)
        with self.assertRaises(ValueError):
            er.EpisodeRecorder(Path(self.tmp.name) / "other", {}, {"x": "http://10.0.0.1:80/cam"}, None)

    def test_an_episode_ends_by_itself_at_the_time_limit_and_at_session_end(self) -> None:
        self.make(max_episode_s=0.5)
        self.ticks(2)
        self.press("a")
        self.wait("rec#1")
        self.ticks(40)
        self.assertIn("saved=1", self.wait("idle"))
        self.press("a")
        self.wait("rec#2")
        self.ticks(5)
        out = self.rec.close()
        self.assertEqual([(e["episode"], e["closed_by"]) for e in out["episodes"]], [(1, "max_duration"), (2, "session_end")])
        self.assertEqual((out["saved"], out["camera_recorder_exit"], out["error"]), (2, 0, None))
        self.assertTrue((Path(self.tmp.name) / "sess" / "episode_0002" / "meta.json").exists())

    def test_a_failed_arm_read_is_recorded_as_missing_not_as_the_previous_pose(self) -> None:
        self.make()
        self.ticks(2)
        self.press("a")
        self.wait("rec#1")
        self.ticks(5)
        self.ticks(3, fb=False)
        self.ticks(5)
        self.rec.request("stop")
        self.wait("idle")
        ep = Path(self.tmp.name) / "sess" / "episode_0001"
        steps = ei.rows(ep / "steps.jsonl")
        self.assertEqual(sum(r["tcp_mm"] is None and r["joints"] is None and r["tcp_rpy"] is None for r in steps), 3)
        self.assertIn("3 rows without an arm pose", json.loads((ep / "meta.json").read_text())["problems"])
        self.assertIn("3 rows without a pose", ei.check(ep)[1])

    def test_labels_are_the_pose_change_in_the_tool_frame(self) -> None:
        self.make()
        self.ticks(2)
        self.press("a")
        self.wait("rec#1")
        self.ticks(60)  # the pretend arm rises along base +z with the tool pointing down
        self.rec.request("stop")
        self.wait("idle")
        ep = Path(self.tmp.name) / "sess" / "episode_0001"
        self.assertEqual(ei.main([str(ep.parent), "--labels", "5", "--ref-cam", "wrist"]), 0)
        lab = json.loads((ep / "labels_k5.json").read_text())
        self.assertEqual(len(lab["actions"]), lab["n"] - 5)
        self.assertGreater(lab["n"], 30)
        for i in (5, len(lab["actions"]) - 5):
            act, f0, f1 = lab["actions"][i], lab["frames"][i], lab["frames"][i + 5]
            dz_base = f1["xyz_m"][2] - f0["xyz_m"][2]
            self.assertGreater(dz_base, 0.0005)
            self.assertAlmostEqual(act[2], -dz_base, places=5)  # base +z is tool -z when the tool points down
            self.assertTrue(all(abs(v) < 1e-5 for v in act[:2] + act[3:6]))
            self.assertEqual(act[6], f1["aperture"])
        self.assertLess(lab["frames"][-1]["aperture"], lab["frames"][0]["aperture"])  # the pretend jaws close as the arm rises


class RelativeActionTests(unittest.TestCase):
    def test_translation_and_rotation_are_expressed_in_the_first_pose(self) -> None:
        yaw90 = rpy_to_q(0.0, 0.0, math.pi / 2)  # tool x points along base +y
        act = ei.relative_action((0.3, 0.0, 0.4), yaw90, (0.3, 0.05, 0.4), rpy_to_q(0.0, 0.0, math.pi / 2 + 0.2))
        for got, want in zip(act, (0.05, 0.0, 0.0, 0.0, 0.0, 0.2)):
            self.assertAlmostEqual(got, want, places=9)


if __name__ == "__main__":
    unittest.main()
