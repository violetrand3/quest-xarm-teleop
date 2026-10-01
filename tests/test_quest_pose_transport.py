"""Offline tests for the NEW-reader diagnostic transport (protocol v2, quest-pose-link).

ALL DATA HERE IS SYNTHETIC (including the fake `adb` used for the acquisition tests). These
tests involve no Quest headset, no real adb, no SSH and no robot, and do not count as physical
tests.

PoseProtocolTests, ContinuityTests, PoseReceiverTests and PoseServerTests need only the files
deployed to the lab (run there on Python 3.10). PoseSenderTests/AcquisitionTests also need
quest_pose_stream_sender.py and run locally.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))

import quest_pose_stream_receiver as prx  # noqa: E402
import quest_pose_transport as pt  # noqa: E402
import quest_pose_v1 as qp  # noqa: E402
import quest_stream_receiver as rx1  # noqa: E402
import quest_transport as qt  # noqa: E402
from test_quest_pose_v1 import ALL, syn_inputs, syn_pose, syn_record  # noqa: E402

MS = 1_000_000
BUILD = "s4a-test"


def rec_obj(seq: int, **kw) -> dict:  # type: ignore[no-untyped-def]
    """SYNTHETIC source pose record with the grip pressed (available right input)."""
    kw.setdefault("in_r", syn_inputs(True, squeeze=1.0, trigger=0.25, a=True, b=False))
    return syn_record(seq=seq, **kw)


def ev_obj(seq: int, name: str, sid: str = "00000000deadbeef", t: int = 0, state: str | None = None, epoch: int = 0) -> dict:
    d = {"v": 1, "type": "event", "build": BUILD, "sid": sid, "seq": seq, "event": name, "space": "LOCAL",
         "local_epoch": epoch, "time_ns": t}  # fmt: skip
    if state:
        d["session_state"] = state
    return d


def feed(tr: pt.ContinuityTracker, obj: dict, now: int) -> str:
    return tr.on_record(qp.decode_record(json.dumps(obj)), now)


def tracker() -> pt.ContinuityTracker:
    tr = pt.ContinuityTracker(expect_build=BUILD, stale_ns=500 * MS)
    tr.new_generation(0, "test")
    tr.set_source(True, "synthetic", 0)
    return tr


def pose_pkt(tr: pt.ContinuityTracker, obj: dict, seq: int, session: str = "sessA", run: str = "runA",
             decimated: int = 0, recv: int = 0) -> dict:  # fmt: skip
    rec = qp.decode_record(json.dumps(obj))
    pkt = pt.pose_packet(run, tr.acq_gen, tr.session_gen, rec, obj, recv, "0.0", tr.continuity_fields(), decimated,
                         tr.source_up, tr.stale)  # fmt: skip
    pkt.update(v=2, session=session, seq=seq, send_mono_ns=recv + 1)
    return json.loads(json.dumps(pkt))  # as received over the wire


def hello(session: str = "sessA", run: str = "runA", **over) -> dict:  # type: ignore[no-untyped-def]
    h = {"v": 2, "type": "hello", "session": session, "sender_run": run, "protocol": pt.PROTOCOL,
         "source_format": pt.SOURCE_FORMAT, "source_schema": pt.SOURCE_SCHEMA, "frame": pt.FRAME}  # fmt: skip
    h.update(over)
    return h


def status_pkt(tr: pt.ContinuityTracker, seq: int, session: str = "sessA") -> dict:
    return {"v": 2, "type": "status", "session": session, "seq": seq, "acq_gen": tr.acq_gen, "source_up": tr.source_up,
            "source_note": tr.source_note or "-", "stale": tr.stale, "continuity": tr.continuity_fields()}  # fmt: skip


class PoseProtocolTests(unittest.TestCase):
    def test_versions_are_not_interchangeable(self) -> None:
        with self.assertRaises(qt.ProtocolError):
            pt.decode(b'{"v":1,"type":"hello"}')  # old protocol into the new endpoint
        with self.assertRaises(qt.ProtocolError):
            qt.decode(b'{"v":2,"type":"pose"}')  # new packet into the old endpoint
        with self.assertRaises(qt.ProtocolError):
            pt.decode(b'{"v":2,"type":"sample"}')  # v1 packet type is not a v2 type
        with self.assertRaises(qt.ProtocolError):
            pt.decode(b'{"v":2,"type":"pose","x":NaN}')

    def test_new_telemetry_and_flags_preserved(self) -> None:
        tr = tracker()
        obj = rec_obj(1, gl=syn_pose(flags=0, active=False),  # left missing
                      gr=syn_pose((0.2, 1.0, -0.3), (0.0, 0.3826834, 0.0, 0.9238795)),
                      in_r=syn_inputs(True, squeeze=0.75, trigger=None, a=True, b=False))
        feed(tr, obj, 10)
        v = pt.validate_pose(pose_pkt(tr, obj, 1))
        self.assertTrue(v.available)
        self.assertEqual(v.record.poses["grip_r"].flags, ALL)
        self.assertEqual(v.record.inputs["in_r"]["squeeze"], 0.75)
        self.assertIsNone(v.record.inputs["in_r"]["trigger"])  # null preserved, not 0
        self.assertIs(v.record.inputs["in_r"]["a"], True)
        self.assertIsNone(v.matrices["grip_l"])  # missing pose: no identity fallback
        self.assertEqual(v.matrices["grip_r"], qp.pose_matrix((0.2, 1.0, -0.3), (0.0, 0.3826834, 0.0, 0.9238795)))
        self.assertEqual((v.record.t_query_ns, v.source_seq, v.build), (1_000_000_000, 1, BUILD))

    def test_tampered_or_inconsistent_pose_rejected(self) -> None:
        tr = tracker()
        obj = rec_obj(1)
        feed(tr, obj, 10)
        good = pose_pkt(tr, obj, 1)
        pt.validate_pose(json.loads(json.dumps(good)))
        for mutate in (
            lambda p: p["matrices"]["grip_r"][0].__setitem__(3, 9.9),  # not the unmodified LOCAL pose
            lambda p: p["matrices"].__setitem__("grip_l", None),  # presence disagrees
            lambda p: p["raw_right"].__setitem__("available", False),
            lambda p: p["source"].__setitem__("sid", "ffffffffffffffff"),
            lambda p: p.__setitem__("frame", "headset_relative_observed"),  # v1 frame label
            lambda p: p["record"]["grip_r"].__setitem__("pos_tracked", False),  # flag/field mismatch
        ):
            bad = json.loads(json.dumps(good))
            mutate(bad)
            with self.assertRaises(qt.ProtocolError):
                pt.validate_pose(bad)

    def test_single_tracked_bit_makes_input_unavailable(self) -> None:
        for flags, why in ((ALL & ~qp.POSITION_TRACKED, "pos_untracked"), (ALL & ~qp.ORIENTATION_TRACKED, "ori_untracked")):
            rec = qp.decode_record(json.dumps(rec_obj(1, gr=syn_pose((0.2, 1.0, -0.3), flags=flags))))
            self.assertEqual(pt.right_input_reasons(rec), [why])  # type: ignore[arg-type]
        rec = qp.decode_record(json.dumps(rec_obj(1, in_r=syn_inputs(True, squeeze=None))))
        self.assertEqual(pt.right_input_reasons(rec), ["grip_action_inactive"])  # type: ignore[arg-type]


class ContinuityTests(unittest.TestCase):
    def test_gap_is_a_break_and_distinct_from_decimation(self) -> None:
        tr = tracker()
        feed(tr, rec_obj(1), 10)
        g0 = tr.continuity_gen
        for s in (2, 3, 4):  # consecutive records: no break (decimation is the sender's choice, not a gap)
            feed(tr, rec_obj(s), 10 + s)
        self.assertEqual((tr.continuity_gen, tr.source_gap_records), (g0, 0))
        feed(tr, rec_obj(7), 30)  # 5 and 6 never arrived: type unknown, never assumed harmless
        self.assertGreater(tr.continuity_gen, g0)
        self.assertEqual(tr.source_gap_records, 2)
        self.assertIn("source_gap", tr.break_counts)

    def test_brief_loss_between_poses_raises_generation(self) -> None:
        tr = tracker()
        feed(tr, rec_obj(1), 10)
        g0, l0 = tr.continuity_gen, tr.loss_count
        feed(tr, rec_obj(2, gr=syn_pose(flags=0, active=False), in_r=syn_inputs(True)), 20)  # one-frame loss
        feed(tr, rec_obj(3), 30)  # valid again
        self.assertTrue(tr.available)
        self.assertEqual(tr.loss_count, l0 + 1)
        self.assertGreater(tr.continuity_gen, g0)  # persists after recovery

    def test_changed_reference_counter_with_event_absent(self) -> None:
        tr = tracker()
        feed(tr, rec_obj(1, epoch=1, change=5000), 10)
        g0 = tr.continuity_gen
        feed(tr, rec_obj(2, epoch=2, change=9000), 20)  # the LOCAL change event itself was never received
        self.assertGreater(tr.continuity_gen, g0)
        self.assertIn("ref_counter_changed_event_absent", tr.break_counts)
        g1 = tr.continuity_gen
        feed(tr, rec_obj(3, epoch=2, change=9999), 30)  # only the change time differs
        self.assertGreater(tr.continuity_gen, g1)

    def test_reference_and_session_events_break(self) -> None:
        tr = tracker()
        feed(tr, rec_obj(1), 10)
        for i, (name, state) in enumerate((("other_space_change_pending", None), ("local_space_change_pending", None),
                                           ("events_lost", None), ("session_state", "VISIBLE"), ("session_end", None),
                                           ("session_begin", None))):  # fmt: skip
            g = tr.continuity_gen
            feed(tr, ev_obj(2 + i, name, state=state, t=123), 20 + i)
            self.assertGreater(tr.continuity_gen, g, name)
        self.assertFalse(tr.available)  # an event alone never restores input
        g = tr.continuity_gen
        feed(tr, ev_obj(8, "session_state", state="FOCUSED"), 50)
        self.assertEqual(tr.continuity_gen, g)
        self.assertFalse(tr.available)

    def test_restart_generations_and_app_run(self) -> None:
        tr = tracker()
        feed(tr, rec_obj(5), 10)
        g = tr.continuity_gen
        feed(tr, rec_obj(1, sid="1111111111111111"), 20)  # app restarted: new run id, seq restarts
        self.assertGreater(tr.continuity_gen, g)
        self.assertIn("app_run_changed", tr.break_counts)
        self.assertEqual(tr.source_gap_records, 0)  # a restart is not counted as a sequence gap
        g, a = tr.continuity_gen, tr.acq_gen
        tr.new_generation(30, "explicit")
        self.assertEqual(tr.acq_gen, a + 1)
        self.assertGreater(tr.continuity_gen, g)
        self.assertFalse(tr.available)
        self.assertIsNone(tr.sid)

    def test_watchdog_detects_stale_without_new_records(self) -> None:
        tr = tracker()
        feed(tr, rec_obj(1), 100 * MS)
        self.assertFalse(tr.watchdog(500 * MS))
        g = tr.continuity_gen
        self.assertTrue(tr.watchdog(601 * MS))  # no records arrived; the caller's clock decides
        self.assertFalse(tr.available)
        self.assertGreater(tr.continuity_gen, g)
        self.assertEqual(tr.stale_detections[0]["delay_ms"], 501.0)
        g = tr.continuity_gen
        tr.watchdog(900 * MS)
        self.assertEqual(tr.continuity_gen, g)  # one break per stale episode
        feed(tr, rec_obj(2), 1000 * MS)
        self.assertFalse(tr.stale)
        self.assertTrue(tr.available)

    def test_foreign_build_and_duplicates_ignored(self) -> None:
        tr = tracker()
        other = rec_obj(1)
        other["build"] = "s4a-9e6f1f24dbed"
        self.assertEqual(feed(tr, other, 1), "foreign")
        self.assertIsNone(tr.sid)
        feed(tr, rec_obj(3), 2)
        self.assertEqual(feed(tr, rec_obj(3), 3), "duplicate")
        self.assertEqual(feed(tr, rec_obj(2), 4), "duplicate")


class PoseReceiverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.s = prx.PoseReceiverState(500, 500)
        self.s.open_connection(0)
        self.assertEqual(self.s.process(hello(), 0)["type"], "hello_ack")
        self.tr = tracker()

    def send(self, obj: dict, seq: int, now: int, **kw) -> dict:  # type: ignore[no-untyped-def]
        feed(self.tr, obj, now)
        return self.s.process(pose_pkt(self.tr, obj, seq, **kw), now)  # type: ignore[return-value]

    def test_incompatible_source_rejected(self) -> None:
        for over in ({"source_format": "wE9ryARX"}, {"source_schema": 2}, {"protocol": "other"}, {"frame": "headset_relative_observed"}):
            s = prx.PoseReceiverState()
            s.open_connection(0)
            self.assertEqual(s.process(hello(**over), 0)["type"], "error")
            self.assertIsNone(s.session)

    def test_end_to_end_values_and_continuity(self) -> None:
        a1 = self.send(rec_obj(1), 1, 10 * MS)
        a2 = self.send(rec_obj(2), 2, 20 * MS)
        self.assertEqual((a1["status"], a1["continuous"], a2["continuous"]), ("ok", False, True))
        snap = self.s.snapshot(25 * MS)
        self.assertEqual((snap.transport, snap.input), ("LIVE", "PRESENT"))
        line = prx.format_snapshot(snap)
        self.assertIn("grip=1.00", line)
        self.assertIn("trig=0.25", line)
        self.assertIn("A=1", line)

    def test_brief_loss_between_transmitted_poses_is_not_erased(self) -> None:
        self.send(rec_obj(1), 1, 10 * MS)
        self.send(rec_obj(2), 2, 20 * MS)
        # records 3 (input lost for one frame) and 4 are processed by the tracker but NOT transmitted
        feed(self.tr, rec_obj(3, gr=syn_pose(flags=0, active=False), in_r=syn_inputs(True)), 30 * MS)
        feed(self.tr, rec_obj(4), 40 * MS)
        a = self.send(rec_obj(5), 3, 50 * MS, decimated=2)
        self.assertTrue(a["available"])
        self.assertFalse(a["continuous"])  # the next transmitted pose is valid but NOT continuous
        st = self.s.sessions[-1]
        self.assertEqual(st.generation_changes_seen, 1)
        self.assertIn("input_lost", st.break_reasons)
        self.assertEqual((st.decimated_by_sender, st.source_gap_records), (2, 0))  # skips are not gaps
        self.assertTrue(self.send(rec_obj(6), 4, 60 * MS)["continuous"])

    def test_identity_change_without_generation_change_is_caught(self) -> None:
        self.send(rec_obj(1, epoch=1, change=5000), 1, 10 * MS)
        pkt = pose_pkt(self.tr, rec_obj(2, epoch=2, change=9000), 2)  # tracker NOT fed: gen unchanged in packet
        a = self.s.process(pkt, 20 * MS)
        self.assertFalse(a["continuous"])
        self.assertEqual(self.s.sessions[-1].identity_changed_without_generation, 1)

    def test_status_heartbeats_do_not_refresh_input(self) -> None:
        self.send(rec_obj(1), 1, 0)
        for i, t in enumerate(range(100, 1100, 100)):
            self.assertEqual(self.s.process(status_pkt(self.tr, 2 + i), t * MS)["status"], "ok")
        snap = self.s.snapshot(1050 * MS)
        self.assertEqual((snap.transport, snap.input, snap.valid_total), ("LIVE", "STALE", 1))
        self.assertIsNone(snap.current)  # stale values are not presented
        self.assertNotIn("grip=1.00", prx.format_snapshot(snap))

    def test_sender_reported_stale_or_down_clears_immediately(self) -> None:
        self.send(rec_obj(1), 1, 0)
        self.tr.watchdog(600 * MS)
        self.s.process(status_pkt(self.tr, 2), 601 * MS)
        self.assertEqual(self.s.snapshot(602 * MS).input, "STALE")
        self.tr.set_source(False, "app not running", 700 * MS)
        self.s.process(status_pkt(self.tr, 3), 701 * MS)
        snap = self.s.snapshot(702 * MS)
        self.assertEqual(snap.input, "SOURCE_DISCONNECTED")
        self.assertIsNone(snap.current)

    def test_restart_no_cached_pose_and_old_identity_rejected(self) -> None:
        self.send(rec_obj(7), 1, 10 * MS)
        old = pose_pkt(self.tr, rec_obj(8), 3)  # built now, delivered late
        feed(self.tr, rec_obj(1, sid="1111111111111111"), 20 * MS)  # app restarted
        a = self.s.process(pose_pkt(self.tr, rec_obj(1, sid="1111111111111111"), 2), 20 * MS)
        self.assertEqual((a["status"], a["continuous"]), ("ok", False))
        r = self.s.process(old, 30 * MS)  # late pose from the OLD app run
        self.assertEqual(r["status"], "rejected")
        self.assertIn("retired", r["reason"])
        self.assertEqual(self.s.snapshot(31 * MS).current.sid, "1111111111111111")  # type: ignore[union-attr]
        self.s.close_connection(40 * MS, "EOF")
        self.assertIsNone(self.s.snapshot(41 * MS).current)
        self.s.open_connection(50 * MS)
        self.s.process(hello("sessB", "runB"), 50 * MS)
        snap = self.s.snapshot(51 * MS)
        self.assertEqual((snap.input, snap.valid_total, snap.continuity_gen), ("MISSING", 0, None))

    def test_unavailable_pose_clears_and_values_hidden(self) -> None:
        self.send(rec_obj(1), 1, 10 * MS)
        a = self.send(rec_obj(2, state="VISIBLE", focused=False), 2, 20 * MS)
        self.assertFalse(a["available"])
        snap = self.s.snapshot(21 * MS)
        self.assertEqual(snap.input, "MISSING")
        self.assertIn("unfocused", snap.input_reason)
        self.assertIn("R.pos=-", prx.format_snapshot(snap))


def _recv_lines(sock: socket.socket, n: int, timeout: float = 2.0) -> list[dict]:
    sock.settimeout(timeout)
    f, out = qt.LineFramer(), []
    while len(out) < n:
        data = sock.recv(4096)
        if not data:
            break
        out += [json.loads(x) for x in f.feed(data)]
    return out


class PoseServerTests(unittest.TestCase):
    """Real loopback sockets, SYNTHETIC packets."""

    def setUp(self) -> None:
        self.state = prx.PoseReceiverState(150, 150)  # test-only thresholds
        self.server = rx1.DiagnosticServer(self.state, port=0, decode_fn=pt.decode, proto_version=pt.PROTOCOL_VERSION,
                                           log_fields=prx.log_fields)  # fmt: skip
        self.server.start()

    def tearDown(self) -> None:
        self.server.stop()

    def test_handshake_pose_ack_and_watchdog(self) -> None:
        self.assertEqual(self.server._sock.getsockname()[0], "127.0.0.1")
        c = socket.create_connection(("127.0.0.1", self.server.port), timeout=2)
        tr = tracker()
        feed(tr, rec_obj(1), 0)
        c.sendall(qt.encode(hello()) + qt.encode(pose_pkt(tr, rec_obj(1), 1)))
        replies = _recv_lines(c, 2)
        self.assertEqual([r["type"] for r in replies], ["hello_ack", "ack"])
        self.assertEqual((replies[0]["v"], replies[0]["robot_connected"], replies[0]["protocol"]), (2, False, pt.PROTOCOL))
        self.assertEqual(self.state.snapshot(time.monotonic_ns()).input, "PRESENT")
        time.sleep(0.3)  # handler blocked in recv; the watchdog view still goes stale
        snap = self.state.snapshot(time.monotonic_ns())
        self.assertEqual((snap.transport, snap.input), ("STALE", "STALE"))
        c.close()

    def test_v1_client_is_not_accepted(self) -> None:
        c = socket.create_connection(("127.0.0.1", self.server.port), timeout=2)
        c.sendall(qt.encode({"v": 1, "type": "hello", "session": "s", "source_session": "x"}))
        time.sleep(0.3)
        self.assertIsNone(self.state.session)  # no session for the old protocol
        self.assertEqual(self.state.errors_total, 1)
        c.close()


class PoseSenderTests(unittest.TestCase):
    """Sender slot + session against a real loopback receiver; SYNTHETIC log lines."""

    LINE = "         %.6f  4242  4242 I QuestXarmPoseV1: %s"

    @classmethod
    def setUpClass(cls) -> None:
        import quest_pose_stream_sender as ptx

        cls.ptx = ptx

    def setUp(self) -> None:
        self.state = prx.PoseReceiverState()
        self.server = rx1.DiagnosticServer(self.state, port=0, decode_fn=pt.decode, proto_version=pt.PROTOCOL_VERSION)
        self.server.start()
        self.slot = self.ptx.PoseSlot(BUILD, stale_ms=200)
        self.gen = self.slot.new_generation(time.monotonic_ns(), "test")
        self.slot.set_source(True, "synthetic", time.monotonic_ns())
        self.sess = self.ptx.PoseSenderSession(self.ptx.connect(self.server.port), self.slot, "runT", max_rate_hz=50)
        self.assertIs(self.sess.handshake()["robot_connected"], False)
        self.sess.start_reader()

    def tearDown(self) -> None:
        self.sess.close(say_bye=True)
        self.server.stop()

    def line(self, obj: dict, gen: int | None = None) -> None:
        self.slot.on_line(self.LINE % (time.time(), json.dumps(obj)), time.monotonic_ns(), self.gen if gen is None else gen)

    def wait_acks(self, n: int) -> None:
        deadline = time.time() + 3
        while time.time() < deadline and self.sess.counts["acks_ok"] + self.sess.counts["acks_rejected"] < n:
            time.sleep(0.01)

    def test_every_record_processed_before_decimation(self) -> None:
        for s in (1, 2, 3):
            self.line(rec_obj(s))
        self.sess.tick()  # sends only #3; #1 and #2 are intentional decimation
        self.line(rec_obj(4, gr=syn_pose(flags=0, active=False), in_r=syn_inputs(True)))  # one-frame loss...
        self.line(rec_obj(5))
        self.line(rec_obj(6))  # ...hidden between two transmitted poses
        self.sess.tick()
        self.line(rec_obj(8))  # 7 missing at the source
        self.sess.tick()
        self.wait_acks(3)
        st = self.state.sessions[-1]
        self.assertEqual((st.poses, st.poses_available, st.continuous_poses), (3, 3, 0))
        self.assertEqual(self.slot.decimated_total, 4)
        self.assertEqual(st.decimated_by_sender, 4)  # 2 + 2, reported separately from...
        self.assertEqual(st.source_gap_records, 1)  # ...the one missing source record
        self.assertIn("input_lost", st.break_reasons)
        self.assertIn("source_gap", st.break_reasons)
        self.assertEqual(self.slot.tracker.counts["pose"], 7)  # all seven were processed

    def test_stale_heartbeats_continue_without_refreshing_input(self) -> None:
        self.line(rec_obj(1))
        self.sess.tick()
        time.sleep(0.35)  # no new source lines: the sender's own watchdog must flag it
        for _ in range(4):
            self.sess.tick()
            time.sleep(0.25)
        self.wait_acks(3)
        st = self.state.sessions[-1]
        self.assertGreaterEqual(st.status_packets, 2)
        self.assertEqual(st.poses_available, 1)
        self.assertIn("source_stale", st.break_reasons)
        snap = self.state.snapshot(time.monotonic_ns())
        self.assertEqual((snap.transport, snap.input), ("LIVE", "STALE"))
        self.assertEqual(len(self.slot.tracker.stale_detections), 1)

    def test_generation_restart_discards_old_lines_and_cached_pose(self) -> None:
        self.line(rec_obj(1))  # cached, not yet sent
        old = self.gen
        self.gen = self.slot.new_generation(time.monotonic_ns(), "restart")
        self.sess.tick()
        self.line(rec_obj(2), gen=old)  # a late line from the aborted generation
        self.sess.tick()
        self.wait_acks(1)
        self.assertEqual(self.state.sessions[-1].poses, 0)  # neither the cached nor the stale-generation pose was sent
        self.assertEqual(self.slot.stale_generation_lines, 1)
        self.line(rec_obj(1, sid="2222222222222222"))
        self.sess.tick()
        self.wait_acks(2)
        st = self.state.sessions[-1]
        self.assertEqual((st.poses, st.source_identities[0]["sid"], st.source_identities[0]["acq_gen"]), (1, "2222222222222222", self.gen))


class AcquisitionTests(unittest.TestCase):
    """Owned-child lifecycle against a SYNTHETIC fake adb (no device)."""

    @classmethod
    def setUpClass(cls) -> None:
        import quest_pose_stream_sender as ptx

        cls.ptx = ptx
        cls.fake = str(ROOT / "tests" / "fixtures" / "fake_adb.py")

    def test_suspended_child_is_detected_killed_and_replaced(self) -> None:
        slot = self.ptx.PoseSlot(BUILD, stale_ms=300)
        events: list[dict] = []
        acq = self.ptx.OwnedLogcatAcquisition(self.fake, "SYNTH", slot, events.append)
        try:
            g1 = acq.start_generation("test")
            pid = acq.child_pid
            assert pid is not None
            with open(f"/proc/{pid}/stat") as f:
                self.assertEqual(int(f.read().split()[3]), os.getpid())  # the child is OURS (ppid == this process)
            deadline = time.time() + 5
            while time.time() < deadline and not slot.tracker.available:
                time.sleep(0.02)
            self.assertTrue(slot.tracker.available)
            os.kill(pid, signal.SIGSTOP)  # induced acquisition-path pause (only our own child)
            t_stop = time.monotonic_ns()
            detected = None
            while time.monotonic_ns() - t_stop < 2_000_000_000:
                if slot.watchdog(time.monotonic_ns()):  # the independent watchdog, not a manual flag
                    detected = time.monotonic_ns()
                    break
                time.sleep(0.01)
            self.assertIsNotNone(detected)
            self.assertFalse(slot.tracker.available)
            det = slot.tracker.stale_detections[-1]
            self.assertGreaterEqual(det["delay_ms"], 300.0)
            self.assertLess(det["delay_ms"], 1000.0)
            info = acq.abort_generation("test pause")  # kill + reap the STOPPED child; never resumed
            self.assertTrue(info["reaped"])
            self.assertTrue(info["reader_joined"])
            self.assertFalse(os.path.exists(f"/proc/{pid}"))  # nothing left behind
            self.assertIsNone(acq.child_pid)
            before = slot.tracker.counts["pose"]
            os.environ["FAKE_ADB_SID"] = "3333333333333333"
            g2 = acq.start_generation("explicit restart")
            self.assertGreater(g2, g1)
            deadline = time.time() + 5
            while time.time() < deadline and not slot.tracker.available:
                time.sleep(0.02)
            self.assertTrue(slot.tracker.available)
            self.assertEqual(slot.tracker.sid, "3333333333333333")  # fresh source identity
            self.assertGreater(slot.tracker.counts["pose"], before)
            self.assertIn("acquisition_restart", slot.tracker.break_counts)
        finally:
            os.environ.pop("FAKE_ADB_SID", None)
            acq.stop()
        self.assertIsNone(acq.child_pid)


if __name__ == "__main__":
    unittest.main()
