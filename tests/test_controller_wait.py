#!/usr/bin/env python3
"""The start-of-session wait for the right controller: the reason it names, and when it ends."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import controller_wait as cw  # noqa: E402

# A real pose record (headset app build s4a-ff0da2692500), right controller seen and focused.
SEEN = json.loads(
    '{"v":1,"type":"pose","build":"s4a-ff0da2692500","sid":"0123456789abcdef","seq":111,"space":"LOCAL",'
    '"t_query_ns":22023300806096,"t_query_kind":"predicted_display_time","session_state":"FOCUSED","focused":true,'
    '"local_epoch":0,"local_change_time_ns":0,'
    '"head":{"queried":true,"result":0,"flags":15,"pos_valid":true,"ori_valid":true,"pos_tracked":true,"ori_tracked":true,'
    '"p":[-0.000070,-0.000291,-0.000290],"q":[-0.0017141,-0.0000953,-0.0027485,0.9999948]},'
    '"grip_l":{"queried":true,"result":0,"active":false,"flags":0,"pos_valid":false,"ori_valid":false,"pos_tracked":false,'
    '"ori_tracked":false,"p":null,"q":null},'
    '"grip_r":{"queried":true,"result":0,"active":true,"flags":15,"pos_valid":true,"ori_valid":true,"pos_tracked":true,'
    '"ori_tracked":true,"p":[0.103668,-0.018958,-0.168738],"q":[-0.2728395,0.0159743,-0.5521146,-0.7877011]},'
    '"in_l":{"trigger":null,"squeeze":null,"thumbstick":null,"thumbstick_click":null,"thumbstick_touch":null,'
    '"trigger_touch":null,"thumbrest_touch":null,"x":null,"y":null,"menu":null},'
    '"in_r":{"trigger":0.0,"squeeze":0.0,"thumbstick":[0.0,0.0],"thumbstick_click":false,"thumbstick_touch":false,'
    '"trigger_touch":false,"thumbrest_touch":false,"a":false,"b":false}}'
)
NOT_ACTIVE = {"queried": True, "result": 0, "active": False, "flags": 0, "pos_valid": False, "ori_valid": False,
              "pos_tracked": False, "ori_tracked": False, "p": None, "q": None}  # fmt: skip
NO_BUTTONS = {k: None for k in SEEN["in_r"]}


def raw(**change) -> str:
    rec = json.loads(json.dumps(SEEN))
    for k, v in change.items():
        if isinstance(v, dict) and isinstance(rec.get(k), dict) and k != "grip_r":
            rec[k].update(v)
        else:
            rec[k] = v
    return json.dumps(rec)


def untracked() -> dict:
    g = dict(SEEN["grip_r"])
    g["pos_tracked"] = g["ori_tracked"] = False
    g["flags"] = 3  # valid but not tracked (the tracked bits must agree with the flags)
    return g


class CauseTests(unittest.TestCase):
    def test_seen(self):
        self.assertIsNone(cw.cause(raw(), 0))

    def test_no_records_or_silent_app(self):
        self.assertEqual(cw.cause(None, None), "starting")
        self.assertEqual(cw.cause(raw(), cw.SILENT_NS + 1), "silent")

    def test_not_in_focus(self):
        # The headset reports the controller inactive too when unfocused: focus is the cause to name.
        self.assertEqual(cw.cause(raw(session_state="READY", focused=False, grip_r=NOT_ACTIVE, in_r=NO_BUTTONS), 0), "asleep")
        self.assertEqual(cw.cause(raw(session_state="VISIBLE", focused=False), 0), "overlay")
        self.assertEqual(cw.cause(raw(session_state="STOPPING", focused=False), 0), "closing")

    def test_controller_not_connected(self):
        self.assertEqual(cw.cause(raw(grip_r=NOT_ACTIVE, in_r=NO_BUTTONS), 0), "inactive")

    def test_controller_not_seen(self):
        self.assertEqual(cw.cause(raw(grip_r=untracked()), 0), "untracked")

    def test_buttons_only(self):
        self.assertEqual(cw.cause(raw(in_r=NO_BUTTONS), 0), "buttons")

    def test_a_normal_start_is_not_reported(self):
        # Recorded starts on 2026-10-12 showed the controller "not connected" for 1.3-3.9 s before it was seen.
        self.assertGreaterEqual(cw.SETTLE_S, 3.0)

    def test_every_cause_has_a_text(self):
        self.assertEqual(set(cw.TEXT), {"starting", "silent", "asleep", "overlay", "closing", "inactive", "untracked", "buttons"})


class WaitRunTests(unittest.TestCase):
    """The program itself, fed a source log written the way the sender writes it."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.src = Path(self.dir.name) / "src.jsonl"
        self.console = Path(self.dir.name) / "console.txt"
        self.src.write_text("")
        self.console.write_text("")
        self.stop = threading.Event()

    def tearDown(self):
        self.stop.set()
        self.dir.cleanup()

    def feed(self, plan):
        """plan: list of (seconds, raw, available) written every 20 ms."""

        def run():
            with self.src.open("a", buffering=1) as f:
                for secs, r, avail in plan:
                    end = time.monotonic() + secs
                    while time.monotonic() < end and not self.stop.is_set():
                        f.write(json.dumps({"host_ns": time.monotonic_ns(), "gen": 1, "kind": "pose", "available": avail, "raw": r}) + "\n")
                        time.sleep(0.02)

        threading.Thread(target=run, daemon=True).start()

    def wait(self, *extra, timeout=20):
        return subprocess.run([sys.executable, str(ROOT / "scripts" / "controller_wait.py"), str(self.src), "--need", "0.5", "--settle", "1", *extra],
                              capture_output=True, text=True, timeout=timeout)  # fmt: skip

    def test_names_each_cause_then_ends_when_seen(self):
        self.feed([(1.6, raw(session_state="VISIBLE", focused=False), False),
                   (1.6, raw(grip_r=NOT_ACTIVE, in_r=NO_BUTTONS), False),
                   (5, raw(), True)])  # fmt: skip
        r = self.wait()
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        lines = r.stdout.splitlines()
        self.assertEqual(len(lines), 2, r.stdout)
        self.assertIn("system window", lines[0])
        self.assertIn("press any button", lines[1])

    def test_a_short_blip_is_not_reported(self):
        self.feed([(0.3, raw(grip_r=untracked()), False), (5, raw(), True)])
        r = self.wait()
        self.assertEqual(r.returncode, 0)
        self.assertEqual(r.stdout, "")

    def test_no_limit_by_default_but_stops_when_the_arm_program_ends(self):
        self.feed([(30, raw(grip_r=untracked()), False)])
        t = threading.Timer(2.5, lambda: self.console.write_text("x | ARMED\nENDING: stopped from another terminal\n"))
        t.start()
        t0 = time.monotonic()
        r = self.wait("--console", str(self.console))
        t.cancel()
        self.assertEqual(r.returncode, 1)
        self.assertGreater(time.monotonic() - t0, 2.4)
        self.assertIn("cannot see it", r.stdout)
        self.assertIn("arm program has ended", r.stdout)

    def test_silent_app(self):
        r = self.wait("--max", "1.5")
        self.assertEqual(r.returncode, 1)
        self.assertIn("has not sent anything yet", r.stdout)


if __name__ == "__main__":
    unittest.main()
