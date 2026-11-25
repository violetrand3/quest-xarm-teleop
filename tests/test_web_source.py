#!/usr/bin/env python3
"""The headset web page as the pose source: its records (scripts/web/records.js, run with node) are the app's records,
and the page's server (quest_web_source.py) hands them to the sender's slot. Runs anywhere; no headset, no arm."""

from __future__ import annotations

import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import quest_pose_transport as pt  # noqa: E402
import quest_pose_v1 as qp  # noqa: E402
from quest_pose_stream_sender import PoseSlot  # noqa: E402
from quest_web_source import WEB_BUILD, WebAcquisition, ws_accept  # noqa: E402
from yam_input import hand_reasons  # noqa: E402

NODE = shutil.which("node")
JS = r"""
const R = require(process.argv[1]);
const s = R.makeStream("0123456789abcdef");
const btn = (v) => ({pressed: v > 0.5, touched: v > 0, value: v});
const pad = (sq, tr, a, b) => ({buttons: [btn(tr), btn(sq), btn(0), btn(0), btn(a), btn(b), btn(0)], axes: [0, 0, 0.5, -0.25]});
const pose = (x, emulated) => ({p: [x, 1.1, -0.3], q: [0, 0.7071068, 0, 0.7071068], emulated: !!emulated});
const out = [];
out.push(s.event("session_begin", 0));
out.push(s.event("session_state", 10, "visible"));
out.push(s.pose({tMs: 20.5, visibility: "visible", head: pose(0), left: null, right: null}));                              // 3 no controllers
out.push(s.pose({tMs: 31, visibility: "visible", head: pose(0), left: {pose: pose(-0.2), gamepad: pad(0, 0, 0, 0)},
                 right: {pose: pose(0.2), gamepad: pad(1, 0.4, 1, 0)}}));                                                    // 4 both seen
out.push(s.pose({tMs: 42, visibility: "visible", head: pose(0), left: null, right: {pose: pose(0.2, true), gamepad: pad(0, 0, 0, 0)}}));  // 5 position guessed
out.push(s.pose({tMs: 53, visibility: "visible", head: pose(0), left: null, right: {pose: null, gamepad: pad(0, 0, 0, 0)}}));             // 6 not located
out.push(s.pose({tMs: 64, visibility: "visible-blurred", head: pose(0), left: null, right: {pose: pose(0.2), gamepad: pad(0, 0, 0, 0)}})); // 7 a system window
out.push(s.spaceChanged(70));                                                                                                // 8 recentre
out.push(s.pose({tMs: 75, visibility: "visible", head: pose(0), left: null, right: {pose: pose(0.2), gamepad: pad(0, 0, 0, 1)}}));         // 9
out.push(s.pose({tMs: 86, visibility: "visible", head: {p: [NaN, 0, 0], q: [0, 0, 0, 1]}, left: null,
                 right: {pose: {p: [0, 0, 0], q: [0, 0, 0, 0]}, gamepad: null}}));                                           // 10 bad numbers
out.push(s.event("session_end", 90));
console.log(JSON.stringify(out));
"""


def page_records() -> list[str]:
    r = subprocess.run([NODE, "-e", JS, str(ROOT / "scripts/web/records.js")], capture_output=True, text=True, timeout=30)
    if r.returncode != 0:
        raise AssertionError(r.stderr)
    return json.loads(r.stdout)


@unittest.skipUnless(NODE, "node is not installed")
class RecordTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.raw = page_records()
        cls.recs = [qp.decode_record(t) for t in cls.raw]  # every record passes the app's strict decoder

    def test_the_records_are_the_apps_records(self) -> None:
        self.assertEqual([r.seq for r in self.recs], list(range(1, 12)))  # one counter for poses and events
        self.assertTrue(all(r.build == WEB_BUILD and r.sid == "0123456789abcdef" for r in self.recs))
        self.assertTrue(all(len(t.encode()) < qp.MAX_RECORD_BYTES for t in self.raw))
        self.assertEqual([type(r).__name__ for r in self.recs],
                         ["EventRecord"] * 2 + ["PoseRecord"] * 5 + ["EventRecord"] + ["PoseRecord"] * 2 + ["EventRecord"])  # fmt: skip
        # the same keys, in the same places, as a record of the app (a real one, from a session of 2026-11-19)
        app = json.loads((ROOT / "tests" / "app_pose_record.json").read_text())
        ours = json.loads(self.raw[3])
        self.assertEqual(set(app), set(ours))
        for k in ("head", "grip_l", "grip_r", "in_l", "in_r"):
            self.assertEqual(set(app[k]), set(ours[k]), k)

    def test_poses_and_buttons(self) -> None:
        r = self.recs[3]
        self.assertEqual(r.t_query_ns, 31_000_000)
        self.assertEqual(self.recs[2].t_query_ns, 20_500_000)
        self.assertEqual([r.poses[k].status for k in qp.POSE_KEYS], [qp.TRACKED] * 3)
        self.assertEqual(r.poses["grip_r"].p, (0.2, 1.1, -0.3))
        self.assertEqual(r.inputs["in_r"], {"trigger": 0.4, "squeeze": 1.0, "thumbstick": (0.5, 0.25), "thumbstick_click": False,
                                            "thumbstick_touch": False, "trigger_touch": True, "thumbrest_touch": False, "a": True, "b": False})  # fmt: skip
        self.assertEqual(r.inputs["in_l"]["menu"], False)
        self.assertTrue(self.recs[8].inputs["in_r"]["b"])
        # a controller that is not there: inactive, no pose, and its buttons absent (null), never zero
        self.assertEqual(self.recs[2].poses["grip_r"].status, qp.MISSING)
        self.assertTrue(all(v is None for v in self.recs[2].inputs["in_r"].values()))

    def test_what_makes_a_hand_unavailable(self) -> None:
        why = [pt.right_input_reasons(r) for r in self.recs if isinstance(r, qp.PoseRecord)]
        self.assertEqual(why[0][:1], ["pose_action_inactive"])
        self.assertEqual(why[1], [])  # both seen: available
        self.assertEqual(why[2], ["pos_untracked"])  # the browser guessed the position
        self.assertEqual(why[3], ["pos_invalid", "ori_invalid"])
        self.assertEqual(why[4], ["unfocused"])  # a system window in front
        self.assertEqual(why[5], [])
        self.assertIn("pos_invalid", why[6])  # non-finite and zero-length numbers are never sent as a pose
        self.assertIsNone(self.recs[9].poses["head"].p)
        self.assertEqual(hand_reasons(self.recs[3], "left"), [])
        self.assertNotEqual(hand_reasons(self.recs[2], "left"), [])

    def test_continuity_as_the_sender_sees_it(self) -> None:
        tr = pt.ContinuityTracker(expect_build=WEB_BUILD)
        tr.new_generation(0)
        seen = []
        for i, r in enumerate(self.recs):
            gen = tr.continuity_gen
            kind = tr.on_record(r, (i + 1) * 10_000_000)
            seen.append((kind, tr.available, tr.continuity_gen > gen))
        self.assertEqual([s[1] for s in seen], [False, False, False, True, False, False, False, False, True, False, False])
        self.assertTrue(seen[0][2])  # session_begin
        self.assertFalse(seen[1][2])  # FOCUSED: no break
        self.assertTrue(seen[7][2])  # the recentre is a break ...
        self.assertFalse(seen[8][2])  # ... and the pose after it agrees with it (no second one)
        self.assertEqual(tr.break_counts.get("local_space_change"), 1)
        self.assertNotIn("ref_counter_changed_event_absent", tr.break_counts)
        self.assertEqual(tr.source_gap_records, 0)


def ws_connect(port: int, path: str = "/ws?v=2&p=0123456789abcdef") -> tuple[socket.socket, str, str]:
    s = socket.create_connection(("127.0.0.1", port), timeout=5)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall(f"GET {path} HTTP/1.1\r\nHost: localhost:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
              f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())  # fmt: skip
    head = b""
    while b"\r\n\r\n" not in head:
        c = s.recv(1)
        if not c:
            break
        head += c
    return s, head.decode(), key


def ws_send(s: socket.socket, text: str) -> None:
    data, mask = text.encode(), os.urandom(4)
    n = len(data)
    head = bytes([0x81, 0x80 | n]) if n < 126 else bytes([0x81, 0x80 | 126]) + struct.pack(">H", n)
    s.sendall(head + mask + bytes(c ^ mask[i & 3] for i, c in enumerate(data)))


def ws_recv_text(s: socket.socket) -> str:
    b = s.recv(2)
    n = b[1] & 0x7F
    if n == 126:
        n = struct.unpack(">H", s.recv(2))[0]
    out = b""
    while len(out) < n:
        out += s.recv(n - len(out))
    return out.decode()


def wait(cond, s: float = 3.0) -> bool:  # noqa: ANN001
    end = time.monotonic() + s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@unittest.skipUnless(NODE, "node is not installed")
class ServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.status = Path(self.tmp.name) / "status.txt"
        self.status.write_text("left: ARMED idle | right: ARMED idle\n")
        self.slot = PoseSlot(WEB_BUILD)
        self.events: list = []
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self.acq = WebAcquisition(self.slot, self.events.append, self.port, "http://127.0.0.1:9", ["left_wrist", "rs_color"],
                                  ["left_wrist"], self.status)  # fmt: skip
        self.acq.start_generation("test")
        self.acq.start_poll()
        self.raw = page_records()

    def tearDown(self) -> None:
        self.acq.stop()
        self.tmp.cleanup()

    def st(self) -> dict:
        return self.slot.status(time.monotonic_ns())

    def test_page_and_config(self) -> None:
        get = lambda p: urllib.request.urlopen(f"http://127.0.0.1:{self.port}{p}", timeout=5).read().decode()  # noqa: E731
        self.assertIn("/records.js", get("/"))
        self.assertIn("makeStream", get("/records.js"))
        self.assertEqual(json.loads(get("/config.json")), {"build": WEB_BUILD, "views": ["left_wrist", "rs_color"], "big": ["left_wrist"], "stereo": [], "options": {"memory": False}, "depth": None})
        for bad in ("/cam/other", "/cam/../x", "/nothing"):
            with self.assertRaises(urllib.error.HTTPError) as e:
                get(bad)
            self.assertEqual(e.exception.code, 404)
        with self.assertRaises(urllib.error.HTTPError) as e:  # no page holds the link: no pictures
            get("/cam/rs_color?p=abcdef0123")
        self.assertEqual(e.exception.code, 409)
        s, _, _ = ws_connect(self.port, "/ws?v=2&p=abcdef0123")
        self.assertTrue(wait(lambda: self.acq._page == "abcdef0123"))
        with self.assertRaises(urllib.error.HTTPError) as e:  # another tab
            get("/cam/rs_color?p=other")
        self.assertEqual(e.exception.code, 409)
        with self.assertRaises(urllib.error.HTTPError) as e:  # the camera view is not running: said, not hung
            get("/cam/rs_color?p=abcdef0123")
        self.assertEqual(e.exception.code, 502)
        s.close()

    def test_records_reach_the_slot_and_the_state_reaches_the_page(self) -> None:
        self.assertFalse(self.st()["source_up"])
        s, head, key = ws_connect(self.port)
        self.assertIn("101", head)
        self.assertIn(ws_accept(key), head)
        self.assertTrue(wait(lambda: self.st()["source_up"]))
        self.assertEqual(json.loads(ws_recv_text(s)), {"status": "left: ARMED idle | right: ARMED idle"})
        for t in self.raw[:4]:
            ws_send(s, t)
        self.assertTrue(wait(lambda: self.st()["available"]))
        self.assertEqual(self.st()["last_source_seq"], 4)
        _events, latest, _ = self.slot.take()
        self.assertEqual(latest["rec"].poses["grip_r"].p, (0.2, 1.1, -0.3))
        self.status.write_text("STOPPED\n")
        self.assertEqual(json.loads(ws_recv_text(s)), {"status": "STOPPED"})
        # a page from before "the newest page takes the link" is refused
        for early in ("/ws", "/ws?v=2"):
            s2, head2, _ = ws_connect(self.port, early)
            self.assertIn("409", head2)
            s2.close()
        # something that is not a record: the input is lost, never kept
        ws_send(s, "{not json")
        self.assertTrue(wait(lambda: not self.st()["available"]))
        ws_send(s, self.raw[4])
        s.close()
        self.assertTrue(wait(lambda: not self.st()["source_up"]))
        self.assertIn("not connected", self.st()["source_note"])

    def test_the_newest_page_takes_the_link(self) -> None:
        old, _, _ = ws_connect(self.port)
        self.assertTrue(wait(lambda: self.st()["source_up"]))
        ws_recv_text(old)  # the status
        new, head, _ = ws_connect(self.port)
        self.assertIn("101", head)
        self.assertEqual(old.recv(4), bytes([0x88, 2]) + struct.pack(">H", 4001))  # told why, then closed
        self.assertEqual(old.recv(1), b"")
        time.sleep(0.3)  # the old page's end must not mark the source down: the new page has it
        self.assertTrue(self.st()["source_up"])
        ws_recv_text(new)
        for t in self.raw[:4]:
            ws_send(new, t)
        self.assertTrue(wait(lambda: self.st()["available"]))
        new.close()
        self.assertTrue(wait(lambda: not self.st()["source_up"]))
        old.close()

    def test_an_aborted_generation_takes_nothing(self) -> None:
        s, _, _ = ws_connect(self.port)
        self.assertTrue(wait(lambda: self.st()["source_up"]))
        self.acq.abort_generation("test")
        for t in self.raw[:4]:
            ws_send(s, t)
        time.sleep(0.3)
        self.assertFalse(self.st()["available"])
        self.assertIsNone(self.st()["sid"])
        self.acq.start_generation("again")
        self.assertTrue(self.st()["source_up"])  # the page stayed connected
        for t in self.raw[:4]:
            ws_send(s, t)
        self.assertTrue(wait(lambda: self.st()["available"]))
        s.close()


if __name__ == "__main__":
    unittest.main()
