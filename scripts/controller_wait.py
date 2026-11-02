#!/usr/bin/env python3
"""Wait for the right controller at the start of a session, saying WHY it is not seen yet.

Reads the sender's source log (src_<name>.jsonl). Every pose record from the headset app says what
is missing (not in focus / controller not active / not tracked); this turns that into one plain
line with what fixes it, printed when it changes and repeated every REMIND_S. While the app is not
in focus the headset is told again to stay awake. Exits 0 once the controller has been seen
continuously for --need seconds; 1 after --max seconds (0 = no limit) or when the arm program's
console (--console) shows it has ended; 130 on Ctrl-C.
Runs on the operator's PC before any arm motion; it only reads the log and sends that broadcast.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

import quest_pose_v1 as qp
from quest_pose_transport import right_input_reasons

FRESH_NS = 300_000_000  # a record older than this is not "now"
SILENT_NS = 3_000_000_000  # no record at all for this long: the app is not producing readings
SETTLE_S = 3.0  # a reason must hold this long before it is printed: a normal start shows 1-4 s of "not connected"
REMIND_S = 30.0
WAKE_EVERY_S = 10.0
WAKE = ["shell", "am", "broadcast", "-a", "com.oculus.vrpowermanager.prox_close"]

TEXT = {
    "starting": "the headset app has not sent anything yet (it is starting)",
    "silent": "the headset app is not sending anything: the headset is probably asleep. "
    "Press its power button briefly or put it on for a moment",
    "asleep": "the headset is not showing the app (asleep or screen off). "
    "Put it on for a moment or press its power button briefly",
    "overlay": "a system window is in front of the app in the headset (boundary, tracking or battery warning, menu). "
    "Put the headset on and close it; a stationary boundary avoids most of these",
    "closing": "the headset app is closing",
    "inactive": "the right controller is not connected: asleep, flat battery, or the headset switched to hand tracking. "
    "Pick it up and press any button",
    "untracked": "the right controller is on but the headset cannot see it. "
    "Hold it in front of the headset, in a lit place",
    "buttons": "the right controller's buttons are not reported yet. Press any button on it",
}


def cause(raw: str | None, age_ns: int | None) -> str | None:
    """Key into TEXT for the newest record, or None when nothing is missing."""
    if age_ns is None:
        return "starting"
    if age_ns > SILENT_NS:
        return "silent"
    if raw is None:
        return "starting"
    try:
        rec = qp.decode_record(raw)
    except Exception:
        return None  # not ours to judge; 'available' in the log decides
    if not isinstance(rec, qp.PoseRecord):
        return None
    why = right_input_reasons(rec)
    if not why:
        return None
    if "unfocused" in why:
        s = rec.session_state
        return "overlay" if s == "VISIBLE" else "closing" if s in ("STOPPING", "LOSS_PENDING", "EXITING") else "asleep"
    if "locate_failed" in why or "pose_action_inactive" in why:
        return "inactive"
    if any(r in why for r in ("pos_invalid", "ori_invalid", "pos_untracked", "ori_untracked")):
        return "untracked"
    return "buttons"


def newest(path: str) -> tuple[dict | None, str | None]:
    """(newest record, raw text of the newest pose record) from the end of the log."""
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 20000))
            lines = f.read().decode("utf-8", "replace").splitlines()[1:]
    except OSError:
        return None, None
    last = raw = None
    for line in reversed(lines):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        last = last or r
        if r.get("kind") == "pose":
            raw = r.get("raw")
            break
    return last, raw


def ended(console: str) -> bool:
    try:
        with open(console, encoding="utf-8", errors="replace") as f:
            return any(line.startswith(("ENDING:", '{"result"')) for line in f)
    except OSError:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src_log")
    ap.add_argument("--need", type=float, default=2.0)
    ap.add_argument("--max", type=float, default=0.0)
    ap.add_argument("--serial", help="headset to keep awake while waiting")
    ap.add_argument("--settle", type=float, default=SETTLE_S)
    ap.add_argument("--adb", default="adb")
    ap.add_argument("--console", help="the arm program's console copy: stop waiting if it has ended")
    a = ap.parse_args()

    def say(text: str) -> None:
        print(time.strftime("[%H:%M:%S] ") + text, flush=True)

    t0 = time.monotonic()
    since = shown = pending = None
    pending_t = last_said = last_wake = last_check = 0.0
    while not a.max or time.monotonic() - t0 < a.max:
        now = time.monotonic()
        if a.console and now - last_check >= 1.0:
            last_check = now
            if ended(a.console):
                say("  The arm program has ended, so the wait for the controller stops.")
                return 1
        rec, raw = newest(a.src_log)
        age = None if rec is None else time.monotonic_ns() - rec["host_ns"]
        if rec is not None and rec.get("available") is True and age < FRESH_NS:
            since = since or now
            if now - since >= a.need:
                return 0
        else:
            since = None
            c = cause(raw, age) or pending
            if c != pending:
                pending, pending_t = c, now
            if c and now - pending_t >= a.settle and (c != shown or now - last_said >= REMIND_S):
                waited = f" ({now - t0:.0f} s; Ctrl-C ends)" if c == shown else ""
                say(f"  Waiting for the controller{waited}: {TEXT[c]}.")
                shown, last_said = c, now
            if a.serial and c in ("silent", "asleep") and now - last_wake >= WAKE_EVERY_S:
                last_wake = now
                subprocess.run([a.adb, "-s", a.serial, *WAKE], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10, check=False)
        time.sleep(0.05)
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
