"""SYNTHETIC tests for scripts/analyze_step04b_test5.py (LOCAL epoch classification).

Synthetic only: no headset, no OpenXR runtime. They do not show that a real recenter
produces these events.
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import analyze_step04b_test5 as a5  # noqa: E402
import quest_pose_v1 as qp  # noqa: E402
from test_quest_pose_v1 import syn_inputs, syn_record  # noqa: E402

SID = "00000000deadbeef"


def ev(seq: int, name: str, t: int, sid: str = SID, pose_valid: bool | None = None, prev: dict | None = None) -> dict:
    d = {"v": 1, "type": "event", "build": "s4a-test", "sid": sid, "seq": seq, "event": name, "space": "LOCAL",
         "local_epoch": 0, "time_ns": t}  # fmt: skip
    if pose_valid is not None:
        d["pose_valid"] = pose_valid
        d["pose_in_previous_space"] = prev if pose_valid else None
    return d


def pose(seq: int, t: int, sid: str = SID) -> dict:
    return syn_record(seq=seq, sid=sid, t=t, in_r=syn_inputs(True, squeeze=1.0))


class EpochTests(unittest.TestCase):
    def changes(self, events: list[dict]) -> list[a5.LocalChange]:
        return a5.local_changes([(1000 + i, qp.decode_record(json.dumps(e)), e) for i, e in enumerate(events)])  # type: ignore[misc]

    def test_before_at_after_change_time(self) -> None:
        ch = self.changes([ev(2, "local_space_change_pending", 5000, pose_valid=True,
                              prev={"p": [0, 0, 0], "q": [0, 0, 0, 1]})])  # fmt: skip
        for t, want in ((4999, 0), (5000, 1), (5001, 1)):
            rec = qp.decode_record(json.dumps(pose(10, t)))
            self.assertEqual(a5.epoch_of(rec, ch), want, t)  # type: ignore[arg-type]

    def test_event_received_after_post_change_samples(self) -> None:
        # Pose queried after changeTime but received BEFORE the event: classification is by query time.
        lines = [{"host_ns": 100, "raw": json.dumps(pose(1, 6000))},
                 {"host_ns": 200, "raw": json.dumps(ev(2, "local_space_change_pending", 5000, pose_valid=False))},
                 {"host_ns": 300, "raw": json.dumps(pose(3, 4000))}]  # fmt: skip
        with tempfile.TemporaryDirectory() as d:
            base = Path(d) / "x"
            base.with_suffix(".jsonl").write_text("\n".join(json.dumps(dict(l, type="x")) for l in lines) + "\n")
            res = a5.analyze(base)
        c = res["local_changes"][0]
        self.assertEqual((c["poses_before"], c["poses_after"]), (1, 1))
        self.assertEqual(c["received_before_event_but_new_epoch"], 1)
        self.assertEqual(c["received_after_event_but_old_epoch"], 1)
        self.assertIsNone(c["pose_in_previous_space"])  # poseValid false -> unknown, not identity

    def test_unrelated_space_type_ignored(self) -> None:
        ch = self.changes([ev(2, "other_space_change_pending", 5000)])
        self.assertEqual(ch, [])
        rec = qp.decode_record(json.dumps(pose(10, 9000)))
        self.assertEqual(a5.epoch_of(rec, ch), 0)  # type: ignore[arg-type]

    def test_pose_valid_false_keeps_previous_unknown(self) -> None:
        ch = self.changes([ev(2, "local_space_change_pending", 5000, pose_valid=False)])
        self.assertIs(ch[0].pose_valid, False)
        self.assertIsNone(ch[0].pose_in_previous)

    def test_session_separation_and_multiple_epochs(self) -> None:
        ch = self.changes([ev(2, "local_space_change_pending", 5000, pose_valid=False),
                           ev(5, "local_space_change_pending", 8000, pose_valid=False),
                           ev(3, "local_space_change_pending", 1000, sid="1111111111111111", pose_valid=False)])  # fmt: skip
        a = qp.decode_record(json.dumps(pose(10, 9000)))
        b = qp.decode_record(json.dumps(pose(11, 6000)))
        other = qp.decode_record(json.dumps(pose(1, 9000, sid="1111111111111111")))
        self.assertEqual(a5.epoch_of(a, ch), 2)  # type: ignore[arg-type]
        self.assertEqual(a5.epoch_of(b, ch), 1)  # type: ignore[arg-type]
        self.assertEqual(a5.epoch_of(other, ch), 1)  # only its own sid's change counts  # type: ignore[arg-type]

    def test_large_integer_boundary_no_float_loss(self) -> None:
        big = 17_186_083_877_193  # realistic XrTime (ns); adjacent integers must stay distinct
        ch = self.changes([ev(2, "local_space_change_pending", big, pose_valid=False)])
        self.assertEqual(a5.epoch_of(qp.decode_record(json.dumps(pose(9, big - 1))), ch), 0)  # type: ignore[arg-type]
        self.assertEqual(a5.epoch_of(qp.decode_record(json.dumps(pose(10, big))), ch), 1)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
