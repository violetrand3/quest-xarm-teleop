"""Offline tests for the Step 3 diagnostic transport.

ALL DATA HERE IS SYNTHETIC. These tests do not involve a Quest headset, adb,
SSH or any robot, and do not count as a physical Quest test.

Protocol/receiver tests (ProtocolTests, ReceiverStateTests, ServerTests) need
only quest_transport.py, quest_stream_receiver.py and quest_input_monitor.py,
so they can run on the lab computer. SenderTests additionally need
quest_stream_sender.py and are run locally.
"""

from __future__ import annotations

import copy
import json
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import quest_stream_receiver as rx  # noqa: E402
import quest_transport as qt  # noqa: E402

MS = 1_000_000
SYN_R = [[1.0, 0.0, 0.0, 0.1], [0.0, 1.0, 0.0, 0.2], [0.0, 0.0, 1.0, 0.3], [0.0, 0.0, 0.0, 1.0]]
SYN_L = [[1.0, 0.0, 0.0, -0.1], [0.0, 1.0, 0.0, 0.2], [0.0, 0.0, 1.0, 0.3], [0.0, 0.0, 0.0, 1.0]]
SYN_BUTTONS = {"A": True, "B": False, "rightTrig": [0.75], "rightJS": [0.0, -0.5]}  # rightGrip deliberately absent


def syn_sample(seq: int, source_seq: int, session: str = "sess1", source_session: str = "src1",
               r: object = SYN_R, l: object = SYN_L, buttons: dict | None = None) -> dict:  # fmt: skip
    """Build a SYNTHETIC sample packet."""
    poses = {"r": copy.deepcopy(r), "l": copy.deepcopy(l)}
    return {
        "v": 1, "type": "sample", "session": session, "seq": seq, "source_session": source_session,
        "source_seq": source_seq, "source_recv_mono_ns": 1000 + source_seq, "send_mono_ns": 2000 + source_seq,
        "device_log_time": "0.0", "frame": qt.FRAME, "tracking_validity": "unknown",
        "poses": poses, "pose_present": {k: v is not None for k, v in poses.items()},
        "pose_validation": {}, "buttons": copy.deepcopy(SYN_BUTTONS if buttons is None else buttons),
        "input_state": qt.compute_input_state(poses), "source": {"up": True, "note": "synthetic"},
    }  # fmt: skip


def hello(session: str = "sess1", source_session: str = "src1") -> dict:
    return {"v": 1, "type": "hello", "session": session, "source_session": source_session}


def hb(seq: int, session: str = "sess1", up: bool = True) -> dict:
    return {"v": 1, "type": "heartbeat", "session": session, "seq": seq, "source": {"up": up}}


class ProtocolTests(unittest.TestCase):
    def test_partial_and_combined_reads(self) -> None:
        f = qt.LineFramer()
        a, b = qt.encode(hello()), qt.encode(syn_sample(1, 1))
        data = a + b
        self.assertEqual(f.feed(data[:5]), [])
        self.assertEqual(f.feed(data[5:len(a) + 3]), [a[:-1]])
        self.assertEqual(f.feed(data[len(a) + 3:]), [b[:-1]])
        self.assertEqual(len(qt.LineFramer().feed(a + b + a)), 3)  # several frames in one read

    def test_oversized_frames(self) -> None:
        f = qt.LineFramer(max_bytes=100)
        with self.assertRaises(qt.FrameTooLarge):
            f.feed(b"x" * 150)  # unterminated
        with self.assertRaises(qt.FrameTooLarge):
            qt.LineFramer(max_bytes=100).feed(b"y" * 120 + b"\n")
        big = syn_sample(1, 1, buttons={f"k{i}": [1.0] for i in range(10)})
        with self.assertRaises(qt.FrameTooLarge):
            qt.encode(big, max_bytes=200)

    def test_non_finite_rejected(self) -> None:
        for text in (b'{"v":1,"type":"bye","x":NaN}', b'{"v":1,"type":"bye","x":Infinity}',
                     b'{"v":1,"type":"bye","x":-Infinity}', b'{"v":1,"type":"bye","x":1e999}'):  # fmt: skip
            with self.subTest(text=text):
                with self.assertRaises(qt.ProtocolError):
                    qt.decode(text)
        with self.assertRaises(qt.ProtocolError):
            qt.encode({"x": float("nan")})

    def test_garbage_rejected(self) -> None:
        for text in (b"not json", b"[1,2]", b'{"v":2,"type":"bye"}', b'{"v":1,"type":"exec"}', b"\xff\xfe"):
            with self.subTest(text=text):
                with self.assertRaises(qt.ProtocolError):
                    qt.decode(text)

    def test_sample_validation(self) -> None:
        v = qt.validate_sample(syn_sample(1, 1))
        self.assertEqual(v.input_state, qt.INPUT_PRESENT)
        self.assertEqual(v.buttons["rightTrig"], (0.75,))
        self.assertNotIn("rightGrip", v.buttons)  # missing stays missing
        bad_mats = [
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[1, 0, 0, 0]] * 3 + [[0, 0, 0, "x"]],
            [[1, 0, 0, 0]] * 3 + [[0, 0, 0, True]],
        ]
        for m in bad_mats:
            with self.subTest(m=str(m)[:30]):
                with self.assertRaises(qt.ProtocolError):
                    qt.validate_sample(syn_sample(1, 1, r=m))
        pkt = syn_sample(1, 1)
        pkt["input_state"] = "MISSING"  # disagrees with poses
        with self.assertRaises(qt.ProtocolError):
            qt.validate_sample(pkt)
        pkt = syn_sample(1, 1)
        del pkt["poses"]["l"]  # missing must be explicit null, not absent
        with self.assertRaises(qt.ProtocolError):
            qt.validate_sample(pkt)


class ReceiverStateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.s = rx.ReceiverState(transport_stale_ms=500, input_stale_ms=500)
        self.s.open_connection(0)
        self.assertEqual(self.s.process(hello(), 0)["type"], "hello_ack")

    def test_valid_delivery_preserves_buttons(self) -> None:
        ack = self.s.process(syn_sample(1, 10), 10 * MS)
        self.assertEqual(ack["status"], "ok")
        snap = self.s.snapshot(20 * MS)
        self.assertEqual((snap.transport, snap.input), ("LIVE", "PRESENT"))
        assert snap.current is not None
        self.assertEqual(snap.current.translation("r"), (0.1, 0.2, 0.3))
        self.assertEqual(snap.current.buttons["rightTrig"], (0.75,))
        self.assertIs(snap.current.buttons["A"], True)
        self.assertNotIn("rightGrip", snap.current.buttons)
        line = rx.format_snapshot(snap)
        self.assertIn("grip=-", line)  # missing grip shown as missing, not 0
        self.assertIn("trig=0.75", line)
        self.assertIn(qt.DIAGNOSTIC_BANNER, line)

    def test_missing_right_pose_clears_immediately(self) -> None:
        self.s.process(syn_sample(1, 1), 10 * MS)
        self.s.process(syn_sample(2, 2, r=None), 20 * MS)  # messages keep arriving
        snap = self.s.snapshot(21 * MS)
        self.assertEqual((snap.transport, snap.input), ("LIVE", "MISSING"))
        self.assertIsNone(snap.current)  # not the old pose, not the left pose

    def test_heartbeats_do_not_refresh_input(self) -> None:
        self.s.process(syn_sample(1, 1), 0)
        for i, t in enumerate(range(100, 1100, 100)):
            self.s.process(hb(2 + i), t * MS)
        snap = self.s.snapshot(1050 * MS)
        self.assertEqual(snap.transport, "LIVE")  # heartbeats keep the transport alive
        self.assertEqual(snap.input, "STALE")  # ...but not the input
        self.assertEqual(snap.valid_total, 1)
        self.assertAlmostEqual(snap.valid_age_ms or 0, 1050)

    def test_no_bytes_goes_stale(self) -> None:
        self.s.process(syn_sample(1, 1), 0)
        snap = self.s.snapshot(600 * MS)
        self.assertEqual((snap.transport, snap.input), ("STALE", "STALE"))

    def test_duplicate_and_out_of_order(self) -> None:
        self.s.process(syn_sample(1, 5), 0)
        self.assertEqual(self.s.process(syn_sample(1, 6), 1 * MS)["status"], "rejected")  # dup packet seq
        self.assertEqual(self.s.process(syn_sample(2, 5), 2 * MS)["status"], "rejected")  # dup source seq
        self.assertEqual(self.s.process(syn_sample(3, 4), 3 * MS)["status"], "rejected")  # old source seq
        self.assertEqual(self.s.process(syn_sample(4, 9), 4 * MS)["status"], "ok")
        self.assertEqual(self.s.snapshot(5 * MS).valid_total, 2)
        self.assertEqual(self.s.sessions[-1].source_seq_skipped, 3)  # 6,7,8 skipped by sender: not errors
        self.assertEqual(self.s.errors_total, 3)

    def test_invalid_matrix_invalidates(self) -> None:
        self.s.process(syn_sample(1, 1), 0)
        bad = syn_sample(2, 2)
        bad["poses"]["r"][0][0] = "nan"
        self.assertEqual(self.s.process(bad, 1 * MS)["status"], "rejected")
        snap = self.s.snapshot(2 * MS)
        self.assertEqual(snap.input, "MISSING")
        self.assertIsNone(snap.current)

    def test_invalidate_and_source_disconnect(self) -> None:
        self.s.process(syn_sample(1, 1), 0)
        self.s.process({"v": 1, "type": "invalidate", "session": "sess1", "seq": 2, "reason": "empty_message"}, 1 * MS)
        self.assertEqual(self.s.snapshot(2 * MS).input, "MISSING")
        self.s.process({"v": 1, "type": "invalidate", "session": "sess1", "seq": 3, "reason": "source_disconnected"}, 3 * MS)
        self.assertEqual(self.s.snapshot(4 * MS).input, "SOURCE_DISCONNECTED")

    def test_new_session_resets(self) -> None:
        self.s.process(syn_sample(1, 100), 0)
        self.s.close_connection(1 * MS, "EOF")
        snap = self.s.snapshot(2 * MS)
        self.assertEqual((snap.transport, snap.input), ("DISCONNECTED", "MISSING"))
        self.assertIsNone(snap.current)
        self.s.open_connection(3 * MS)
        self.s.process(hello("sess2", "src2"), 3 * MS)
        snap = self.s.snapshot(4 * MS)
        self.assertIsNone(snap.current)  # nothing reused from session 1
        self.assertEqual((snap.input, snap.valid_total, snap.last_source_seq), ("MISSING", 0, 0))
        # A new source session may restart its sequence numbers.
        self.assertEqual(self.s.process(syn_sample(1, 1, "sess2", "src2"), 5 * MS)["status"], "ok")
        # Packets tagged with the old session are rejected.
        self.assertEqual(self.s.process(syn_sample(2, 2, "sess1", "src1"), 6 * MS)["type"], "error")

    def test_packet_before_hello_rejected(self) -> None:
        s = rx.ReceiverState()
        s.open_connection(0)
        self.assertEqual(s.process(syn_sample(1, 1), 0)["type"], "error")
        self.assertIsNone(s.snapshot(0).current)


def _recv_lines(sock: socket.socket, n: int, timeout: float = 2.0) -> list[dict]:
    sock.settimeout(timeout)
    f, out = qt.LineFramer(), []
    while len(out) < n:
        data = sock.recv(4096)
        if not data:
            break
        out += [json.loads(x) for x in f.feed(data)]
    return out


class ServerTests(unittest.TestCase):
    """Real loopback sockets on an ephemeral port; SYNTHETIC packets."""

    def setUp(self) -> None:
        self.state = rx.ReceiverState(transport_stale_ms=150, input_stale_ms=150)  # test-only thresholds
        self.server = rx.DiagnosticServer(self.state, port=0)
        self.server.start()

    def tearDown(self) -> None:
        self.server.stop()

    def _client(self) -> socket.socket:
        c = socket.create_connection(("127.0.0.1", self.server.port), timeout=2)
        return c

    def test_binds_loopback_only(self) -> None:
        self.assertEqual(self.server._sock.getsockname()[0], "127.0.0.1")

    def test_handshake_partial_write_and_stale_watchdog(self) -> None:
        c = self._client()
        data = qt.encode(hello()) + qt.encode(syn_sample(1, 1))
        for i in range(0, len(data), 7):  # dribble bytes: partial TCP reads
            c.sendall(data[i:i + 7])
        replies = _recv_lines(c, 2)
        self.assertEqual([r["type"] for r in replies], ["hello_ack", "ack"])
        self.assertIs(replies[0]["robot_connected"], False)
        self.assertEqual(self.state.snapshot(time.monotonic_ns()).input, "PRESENT")
        time.sleep(0.3)  # no bytes; handler thread is blocked in recv
        snap = self.state.snapshot(time.monotonic_ns())
        self.assertEqual((snap.transport, snap.input), ("STALE", "STALE"))
        c.close()

    def test_one_client_at_a_time_and_eof(self) -> None:
        c1 = self._client()
        c1.sendall(qt.encode(hello()) + qt.encode(syn_sample(1, 1)))
        _recv_lines(c1, 2)
        c2 = self._client()
        busy = _recv_lines(c2, 1)
        self.assertEqual(busy[0]["type"], "error")
        self.assertIn("busy", busy[0]["reason"])
        c2.close()
        c1.close()  # EOF
        deadline = time.time() + 3
        while time.time() < deadline and self.state.snapshot(time.monotonic_ns()).transport != "DISCONNECTED":
            time.sleep(0.02)
        snap = self.state.snapshot(time.monotonic_ns())
        self.assertEqual((snap.transport, snap.input), ("DISCONNECTED", "MISSING"))
        self.assertIsNone(snap.current)

    def test_oversized_message_drops_connection(self) -> None:
        c = self._client()
        c.sendall(qt.encode(hello()))
        _recv_lines(c, 1)
        try:
            c.sendall(b"z" * (qt.MAX_MESSAGE_BYTES + 10))
        except OSError:
            pass
        c.settimeout(3)
        try:
            rest = c.recv(4096)
        except OSError:
            rest = b""
        self.assertEqual(rest, b"")  # server closed the connection
        c.close()


class SenderTests(unittest.TestCase):
    """Sender logic with SYNTHETIC ControllerMessages (no adb, no Quest)."""

    @classmethod
    def setUpClass(cls) -> None:
        import quest_input_monitor as qim
        import quest_stream_sender as tx

        cls.qim, cls.tx = qim, tx

    def _msg(self, r: bool = True, l: bool = True, buttons: str = "R,rightTrig 0.5"):  # type: ignore[no-untyped-def]
        ident = "1 0 0 0.1 0 1 0 0.2 0 0 1 0.3 0 0 0 1"
        parts = ([f"r:{ident}"] if r else []) + ([f"l:{ident}"] if l else [])
        return self.qim.parse_message("|".join(parts) + "&" + buttons)

    def _line(self):  # type: ignore[no-untyped-def]
        return self.qim.LogLine("0.0", 1, self.qim.TAG, "synthetic")

    def test_slot_keeps_only_newest_and_preserves_invalidation(self) -> None:
        slot = self.tx.LatestSlot()
        slot.put_message(1, 10, self._line(), self._msg())
        slot.put_message(2, 20, self._line(), self._msg(r=False))  # right missing
        slot.put_message(3, 30, self._line(), self._msg())
        events, sample, _ = slot.take(0)
        self.assertEqual([e.reason for e in events], ["required_pose_missing:r"])  # preserved despite skip
        assert sample is not None
        self.assertEqual(sample.source_seq, 3)  # newest only; 1 and 2 skipped
        events, sample, _ = slot.take(3)
        self.assertEqual((events, sample), ([], None))  # never resend a cached sample

    def test_errors_and_source_loss_clear_latest(self) -> None:
        slot = self.tx.LatestSlot()
        slot.put_message(1, 10, self._line(), self._msg())
        slot.put_error("synthetic parse error", 20)
        events, sample, _ = slot.take(0)
        self.assertIsNone(sample)  # last good pose not left active
        self.assertEqual(events[0].reason, "parse_error")
        slot.put_message(2, 30, self._line(), self._msg())
        slot.set_source(False, "app not running", 40)
        events, sample, src = slot.take(0)
        self.assertIsNone(sample)
        self.assertEqual(events[-1].reason, "source_disconnected")
        self.assertFalse(src["up"])

    def test_empty_message_is_invalidation(self) -> None:
        slot = self.tx.LatestSlot()
        slot.put_message(1, 10, self._line(), self._msg())
        slot.put_message(2, 20, self._line(), self.qim.parse_message("&"))
        events, sample, _ = slot.take(1)
        self.assertEqual([e.reason for e in events], ["empty_message"])
        assert sample is not None
        self.assertEqual(sample.input_state, "MISSING")

    def test_event_queue_is_bounded(self) -> None:
        slot = self.tx.LatestSlot(max_events=4)
        for i in range(10):
            slot.set_source(False, f"down {i}", i)
        events, _, _ = slot.take(0)
        self.assertEqual(len(events), 4)
        self.assertEqual(events[-1].detail, "down 9")  # newest kept
        self.assertEqual(slot.events_dropped, 6)

    def test_end_to_end_with_receiver(self) -> None:
        state = rx.ReceiverState()
        server = rx.DiagnosticServer(state, port=0)
        server.start()
        try:
            slot = self.tx.LatestSlot()
            slot.set_source(True, "synthetic source up", 0)
            sess = self.tx.SenderSession(self.tx.connect(server.port), slot, "srcA", max_rate_hz=50)
            ack = sess.handshake()
            self.assertIs(ack["robot_connected"], False)
            sess.start_reader()
            for i in range(1, 6):
                slot.put_message(i, time.monotonic_ns(), self._line(), self._msg())
            sess.tick()  # sends only #5
            slot.put_message(6, time.monotonic_ns(), self._line(), self._msg(r=False))
            sess.tick()
            deadline = time.time() + 2
            while time.time() < deadline and sess.counts["acks_ok"] < 3:
                time.sleep(0.01)
            self.assertEqual(sess.counts["source_frames_skipped"], 4)
            self.assertEqual(sess.counts["present_sent"], 1)
            self.assertEqual(sess.counts["invalidations_sent"], 1)
            self.assertEqual(len(sess.rtt_ns), 3)
            snap = state.snapshot(time.monotonic_ns())
            self.assertEqual((snap.input, snap.valid_total, snap.last_source_seq), ("MISSING", 1, 6))
            sess.close(say_bye=True)
        finally:
            server.stop()

    def test_slow_receiver_fails_bounded(self) -> None:
        a, b = socket.socketpair()  # b never reads: simulates a stalled link
        for s in (a, b):
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        slot = self.tx.LatestSlot()
        sess = self.tx.SenderSession(a, slot, "srcB", write_timeout_s=0.2, ack_timeout_s=0.5, max_outstanding=64)
        t0 = time.monotonic()
        with self.assertRaises(self.tx.SessionFailed):
            for i in range(1, 10_000):
                slot.put_message(i, time.monotonic_ns(), self._line(), self._msg())
                sess.tick()
        self.assertLess(time.monotonic() - t0, 5)
        self.assertLessEqual(len(sess._outstanding), 64)  # bounded application backlog
        sess.close(say_bye=False)
        self.assertEqual(len(sess._outstanding), 0)  # pending data discarded
        b.close()


if __name__ == "__main__":
    unittest.main()
