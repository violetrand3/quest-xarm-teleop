#!/usr/bin/env python3
"""SYNTHETIC stand-in for `adb` used only by offline tests (no device, no real adb).

  fake_adb.py -s SERIAL shell date +%s.%N          -> prints the host epoch
  fake_adb.py -s SERIAL shell pidof <pkg>          -> prints a fake pid
  fake_adb.py -s SERIAL logcat ...                 -> streams synthetic QuestXarmPoseV1 lines (~100 Hz)
The session id comes from FAKE_ADB_SID so a test can simulate an app restart.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
from test_quest_pose_v1 import syn_inputs, syn_record  # noqa: E402

args = sys.argv[1:]
if "shell" in args and "date" in args:
    print("%.9f" % time.time())
elif "shell" in args and "pidof" in args:
    print("4242")
elif "logcat" in args:
    sid = os.environ.get("FAKE_ADB_SID", "00000000deadbeef")
    seq = int(os.environ.get("FAKE_ADB_START_SEQ", "0"))
    while True:
        seq += 1
        rec = syn_record(seq=seq, sid=sid, t=seq * 13_888_888, in_r=syn_inputs(True, squeeze=1.0))
        rec["build"] = "s4a-test"
        sys.stdout.write("         %.6f  4242  4242 I QuestXarmPoseV1: %s\n" % (time.time(), json.dumps(rec, separators=(",", ":"))))
        sys.stdout.flush()
        time.sleep(0.01)
