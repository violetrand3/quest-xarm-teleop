#!/usr/bin/env python3
"""Local sender for the Step 3 diagnostic link. ROBOT NOT CONNECTED.

Reads the live Quest controller stream with the Step 2 acquisition/parsing
code (one adb logcat reader), keeps only the newest source message in a
latest-sample slot, and sends it over TCP to 127.0.0.1:<port> -- the local
end of a dedicated SSH tunnel to the lab's diagnostic receiver.

  * Acquisition runs independently of network writes.
  * At most --max-rate sample packets per second (transport-test setting,
    NOT a robot servo rate). Each source message is sent at most once.
  * Empty/malformed input and acquisition failures invalidate the current
    input; invalidation events are sent even if rate limiting skips the
    samples around them.
  * Heartbeats carry source status only; they never count as input.
  * Every packet is acknowledged; round-trip time is measured on THIS
    host's monotonic clock only. RTT/2 is not reported as one-way latency.
  * Bounded buffers; explicit connect/handshake/write/ack timeouts. On any
    connection failure pending data is discarded and the sender exits.
    Reconnecting means running it again (new session). Nothing is replayed.

There is deliberately no option that enables robot control.
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, TextIO

from quest_input_monitor import (
    AdbLogcatSource,
    ControllerMessage,
    FreshnessTracker,
    LogLine,
    Monitor,
    MonitorError,
    find_adb,
    pick_serial,
    redact,
)
from quest_transport import (
    DIAGNOSTIC_BANNER,
    FRAME,
    INPUT_PRESENT,
    LOOPBACK,
    MAX_MESSAGE_BYTES,
    PROTOCOL_VERSION,
    TRACKING_VALIDITY,
    LineFramer,
    ProtocolError,
    compute_input_state,
    decode,
    encode,
    missing_reason,
    new_id,
    stats_ms,
)

DEFAULT_LOCAL_PORT = 18765  # local end of the dedicated SSH tunnel
DEFAULT_MAX_RATE_HZ = 30.0
HEARTBEAT_INTERVAL_S = 0.2
CONNECT_TIMEOUT_S = 3.0
HANDSHAKE_TIMEOUT_S = 10.0  # was 3 s; raised by Henok on 2026-11-02
WRITE_TIMEOUT_S = 1.0
ACK_TIMEOUT_S = 2.0
MAX_OUTSTANDING_ACKS = 256
MAX_PENDING_EVENTS = 64
SNDBUF_BYTES = 32 * 1024


class SessionFailed(RuntimeError):
    pass


# --------------------------------------------------------------------------
# Latest-sample slot
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SourceSample:
    source_seq: int
    recv_ns: int  # sender-host monotonic time the log line was read
    device_log_time: str
    msg: ControllerMessage
    input_state: str


@dataclass(frozen=True)
class InvalidationEvent:
    reason: str
    after_source_seq: int
    recv_ns: int
    detail: str = ""


class LatestSlot:
    """Newest source sample plus a bounded list of pending invalidation events.

    Written by the acquisition thread, read by the network thread. Holds at
    most ONE sample regardless of how slow the network is.
    """

    def __init__(self, max_events: int = MAX_PENDING_EVENTS):
        self._lock = threading.Lock()
        self._latest: SourceSample | None = None
        self._events: deque[InvalidationEvent] = deque()
        self._max_events = max_events
        self._last_reason: str | None = "no sample yet"
        self.events_dropped = 0
        self.source_up = False
        self.source_note = "starting"
        self.last_source_seq = 0
        self.last_source_recv_ns: int | None = None
        self.counts = {"messages": 0, "present": 0, "missing": 0, "parse_errors": 0, "source_down": 0, "invalidations": 0}

    def _push(self, ev: InvalidationEvent) -> None:
        if len(self._events) >= self._max_events:
            self._events.popleft()  # keep the newest; overflow is counted
            self.events_dropped += 1
        self._events.append(ev)
        self.counts["invalidations"] += 1

    def put_message(self, seq: int, recv_ns: int, line: LogLine, msg: ControllerMessage) -> None:
        state = compute_input_state(msg.poses)
        reason = missing_reason(msg.poses, msg.buttons)
        with self._lock:
            self.counts["messages"] += 1
            self.counts["present" if state == INPUT_PRESENT else "missing"] += 1
            self.last_source_seq = seq
            self.last_source_recv_ns = recv_ns
            self._latest = SourceSample(seq, recv_ns, line.device_log_time, msg, state)
            if reason is not None and reason != self._last_reason:
                self._push(InvalidationEvent(reason, seq, recv_ns))
            self._last_reason = reason

    def put_error(self, detail: str, recv_ns: int) -> None:
        with self._lock:
            self.counts["parse_errors"] += 1
            self._latest = None  # never leave the last good pose active
            if self._last_reason != "parse_error":
                self._push(InvalidationEvent("parse_error", self.last_source_seq, recv_ns, detail[:120]))
            self._last_reason = "parse_error"

    def set_source(self, up: bool, note: str, now_ns: int) -> None:
        with self._lock:
            self.source_up = up
            self.source_note = note
            if not up:
                self.counts["source_down"] += 1
                self._latest = None
                self._push(InvalidationEvent("source_disconnected", self.last_source_seq, now_ns, note[:120]))
                self._last_reason = "source_disconnected"

    def take(self, last_sent_seq: int) -> tuple[list[InvalidationEvent], SourceSample | None, dict[str, Any]]:
        """Pending events (oldest first) and the newest unsent sample, if any."""
        with self._lock:
            events = list(self._events)
            self._events.clear()
            latest = self._latest
            sample = latest if latest is not None and latest.source_seq > last_sent_seq else None
            source = {"up": self.source_up, "note": self.source_note[:120], "last_source_seq": self.last_source_seq}
            return events, sample, source

    def last_recv_ns(self) -> int | None:
        with self._lock:
            return self.last_source_recv_ns


# --------------------------------------------------------------------------
# Network session
# --------------------------------------------------------------------------


def sample_packet(s: SourceSample, source: dict[str, Any]) -> dict[str, Any]:
    msg = s.msg
    return {
        "type": "sample",
        "source_seq": s.source_seq,
        "source_recv_mono_ns": s.recv_ns,
        "device_log_time": s.device_log_time,
        "frame": FRAME,
        "tracking_validity": TRACKING_VALIDITY,
        "poses": {k: v for k, v in msg.poses.items()},  # None stays null
        "pose_present": msg.pose_present,
        "pose_validation": {
            k: None if c is None else {
                "finite": True, "shape": [4, 4], "homogeneous_ok": c.homogeneous_ok,
                "bottom_row_err": c.bottom_row_err, "orthonormal_err": c.orthonormal_err, "det": c.det,
            }
            for k, c in msg.pose_checks.items()
        },  # fmt: skip
        "buttons": {k: list(v) if isinstance(v, tuple) else v for k, v in msg.buttons.items()},
        "input_state": s.input_state,
        "source": source,
    }


class SenderSession:
    """One TCP connection = one session. Never reconnects by itself."""

    # Hooks so a separately versioned source path (protocol v2, quest_pose_stream_sender.py)
    # can reuse this tested socket/ack/timeout handling. v1 behaviour is unchanged.
    proto_version = PROTOCOL_VERSION
    _decode = staticmethod(decode)

    def _hello_packet(self) -> dict[str, Any]:
        return {"v": self.proto_version, "type": "hello", "session": self.session, "source_session": self.source_session,
                "role": "quest_stream_sender", "frame": FRAME, "tracking_validity": TRACKING_VALIDITY}  # fmt: skip

    def __init__(
        self,
        sock: socket.socket,
        slot: LatestSlot,
        source_session: str,
        max_rate_hz: float = DEFAULT_MAX_RATE_HZ,
        heartbeat_s: float = HEARTBEAT_INTERVAL_S,
        write_timeout_s: float = WRITE_TIMEOUT_S,
        ack_timeout_s: float = ACK_TIMEOUT_S,
        max_outstanding: int = MAX_OUTSTANDING_ACKS,
        log: TextIO | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ):
        self.sock = sock
        self.slot = slot
        self.session = new_id()
        self.source_session = source_session
        self.period_ns = int(1e9 / max_rate_hz)
        self.heartbeat_ns = int(heartbeat_s * 1e9)
        self.ack_timeout_ns = int(ack_timeout_s * 1e9)
        self.max_outstanding = max_outstanding
        self.log = log
        self.clock = clock
        self.sock.settimeout(write_timeout_s)  # bounds every sendall and recv
        self.seq = 0
        # Start from the slot's current position: nothing acquired before this
        # session is ever sent, and skips are counted from session start.
        self.last_sent_source_seq = slot.last_source_seq
        self.last_send_ns: int | None = None
        self._outstanding: dict[int, int] = {}
        self._out_lock = threading.Lock()
        self._stop = threading.Event()
        self.failure: str | None = None
        self.rtt_ns: list[int] = []
        self.acq_to_send_ns: list[int] = []
        self.counts = {"samples_sent": 0, "present_sent": 0, "missing_sent": 0, "invalidations_sent": 0,
                       "heartbeats_sent": 0, "acks_ok": 0, "acks_rejected": 0, "source_frames_skipped": 0}  # fmt: skip
        self.first_sent_source_seq: int | None = None
        self.reject_reasons: dict[str, int] = {}
        self._reader: threading.Thread | None = None

    # -- low level ---------------------------------------------------------
    def _send(self, pkt: dict[str, Any]) -> int:
        self.seq += 1
        pkt = {"v": self.proto_version, **pkt, "session": self.session, "seq": self.seq,
               "source_session": self.source_session}  # fmt: skip
        now = self.clock()
        pkt["send_mono_ns"] = now
        data = encode(pkt, MAX_MESSAGE_BYTES)
        with self._out_lock:
            if len(self._outstanding) >= self.max_outstanding:
                raise SessionFailed(f"ack backlog of {len(self._outstanding)} packets")
            self._outstanding[self.seq] = now
        try:
            self.sock.sendall(data)
        except socket.timeout:
            raise SessionFailed("write timeout") from None
        except OSError as e:
            raise SessionFailed(f"send failed: {e.__class__.__name__}") from None
        self.last_send_ns = now
        if self.log is not None:
            self.log.write(json.dumps({"t": now, "dir": "out", "type": pkt["type"], "seq": self.seq,
                                       "source_seq": pkt.get("source_seq"), "reason": pkt.get("reason")},
                                      separators=(",", ":")) + "\n")  # fmt: skip
        return now

    def handshake(self, timeout_s: float = HANDSHAKE_TIMEOUT_S) -> dict[str, Any]:
        hello = self._hello_packet()
        try:
            self.sock.sendall(encode(hello))
        except OSError as e:
            raise SessionFailed(f"hello send failed: {e.__class__.__name__}") from None
        framer = LineFramer(MAX_MESSAGE_BYTES)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                raise SessionFailed(f"handshake recv failed: {e.__class__.__name__}") from None
            if not data:
                raise SessionFailed("receiver closed during handshake (no receiver behind tunnel?)")
            for frame in framer.feed(data):
                reply = self._decode(frame)
                if reply.get("type") == "error":
                    raise SessionFailed(f"receiver refused: {reply.get('reason')}")
                if (reply.get("type") == "hello_ack" and reply.get("session") == self.session
                        and reply.get("robot_connected") is False and reply.get("diagnostic_only") is True):  # fmt: skip
                    self._framer = framer
                    return reply
                raise SessionFailed(f"unexpected handshake reply {reply.get('type')}")
        raise SessionFailed("handshake timeout")

    def _read_acks(self) -> None:
        framer = getattr(self, "_framer", LineFramer(MAX_MESSAGE_BYTES))
        while not self._stop.is_set():
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                self.failure = self.failure or f"recv failed: {e.__class__.__name__}"
                return
            if not data:
                self.failure = self.failure or "receiver closed connection"
                return
            try:
                frames = framer.feed(data)
                for frame in frames:
                    now = self.clock()
                    pkt = self._decode(frame)
                    if pkt.get("type") == "error":
                        self.failure = self.failure or f"receiver error: {pkt.get('reason')}"
                        continue
                    if pkt.get("type") != "ack" or pkt.get("session") != self.session:
                        continue
                    seq = pkt.get("seq")
                    with self._out_lock:
                        sent = self._outstanding.pop(seq, None) if isinstance(seq, int) else None
                    if sent is None:
                        continue
                    self.rtt_ns.append(now - sent)  # same host, same clock
                    if pkt.get("status") == "ok":
                        self.counts["acks_ok"] += 1
                    else:
                        self.counts["acks_rejected"] += 1
                        r = str(pkt.get("reason"))[:60]
                        self.reject_reasons[r] = self.reject_reasons.get(r, 0) + 1
            except ProtocolError as e:
                self.failure = self.failure or f"bad reply: {e}"
                return

    # -- main loop ------------------------------------------------------------
    def start_reader(self) -> None:
        self._reader = threading.Thread(target=self._read_acks, name="acks", daemon=True)
        self._reader.start()

    def tick(self) -> None:
        """One send opportunity (called at most max_rate times per second)."""
        if self.failure:
            raise SessionFailed(self.failure)
        events, sample, source = self.slot.take(self.last_sent_source_seq)
        for ev in events:
            self._send({"type": "invalidate", "reason": ev.reason, "after_source_seq": ev.after_source_seq,
                        "source_recv_mono_ns": ev.recv_ns, "detail": ev.detail, "source": source})  # fmt: skip
            self.counts["invalidations_sent"] += 1
        now = self.clock()
        if sample is not None:
            if self.first_sent_source_seq is None:
                self.first_sent_source_seq = sample.source_seq
            if sample.source_seq > self.last_sent_source_seq + 1:
                self.counts["source_frames_skipped"] += sample.source_seq - self.last_sent_source_seq - 1
            sent_ns = self._send(sample_packet(sample, source))
            self.last_sent_source_seq = sample.source_seq
            self.counts["samples_sent"] += 1
            if sample.input_state == INPUT_PRESENT:
                self.counts["present_sent"] += 1
                self.acq_to_send_ns.append(sent_ns - sample.recv_ns)  # same host, same clock
            else:
                self.counts["missing_sent"] += 1
        elif not events and (self.last_send_ns is None or now - self.last_send_ns >= self.heartbeat_ns):
            last = self.slot.last_recv_ns()
            src = dict(source, age_ms=None if last is None else round((now - last) / 1e6, 1))
            self._send({"type": "heartbeat", "source": src})
            self.counts["heartbeats_sent"] += 1
        with self._out_lock:
            oldest = min(self._outstanding.values(), default=None)
        if oldest is not None and self.clock() - oldest > self.ack_timeout_ns:
            raise SessionFailed("ack timeout")

    def run(self, should_stop: Callable[[], bool], on_tick: Callable[[], None] | None = None) -> str:
        next_ns = self.clock()
        try:
            while not should_stop():
                self.tick()
                if on_tick is not None:
                    on_tick()
                next_ns += self.period_ns
                delay = (next_ns - self.clock()) / 1e9
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_ns = self.clock()  # fell behind: do not burst to catch up
            return "stopped"
        except SessionFailed as e:
            return f"failed: {e}"

    def close(self, say_bye: bool) -> None:
        if say_bye and not self.failure:
            try:
                self.sock.settimeout(0.5)
                self._send({"type": "bye"})
            except (SessionFailed, OSError):
                pass
            deadline = time.monotonic() + 0.5  # brief, bounded wait for final acks
            while time.monotonic() < deadline and self._outstanding and not self.failure:
                time.sleep(0.01)
        self._stop.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        if self._reader is not None:
            self._reader.join(timeout=2.0)
        with self._out_lock:
            self.unacked_at_close = len(self._outstanding)
            self._outstanding.clear()  # discard pending data; nothing is replayed


def connect(port: int, timeout_s: float = CONNECT_TIMEOUT_S) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, SNDBUF_BYTES)  # bound kernel backlog
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(timeout_s)
    sock.connect((LOOPBACK, port))  # loopback only: the SSH tunnel's local end
    return sock


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serial", help="adb serial (required if more than one device)")
    ap.add_argument("--port", type=int, default=DEFAULT_LOCAL_PORT, help="local tunnel port on 127.0.0.1")
    ap.add_argument("--max-rate", type=float, default=DEFAULT_MAX_RATE_HZ,
                    help="max sample packets/s (transport test setting, not a robot servo rate)")  # fmt: skip
    ap.add_argument("--duration", type=float, help="seconds to run (default: until Ctrl+C)")
    ap.add_argument("--log", type=Path, help="JSONL packet log (must not exist)")
    ap.add_argument("--summary", type=Path, help="summary JSON (must not exist)")
    args = ap.parse_args(argv)
    if not 0 < args.max_rate <= 100:
        print("error: --max-rate must be in (0, 100]", file=sys.stderr)
        return 2
    for p in (args.log, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists; refusing to overwrite", file=sys.stderr)
            return 2
    try:
        adb = find_adb()
        serial = pick_serial(adb, args.serial)
    except (MonitorError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    source_session = new_id()
    slot = LatestSlot()
    try:
        sock = connect(args.port)
    except OSError as e:
        print(f"error: cannot connect to {LOOPBACK}:{args.port} (is the tunnel up?): {e}", file=sys.stderr)
        return 3
    log: TextIO | None = None
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        log = args.log.open("x", encoding="utf-8")
    session = SenderSession(sock, slot, source_session, max_rate_hz=args.max_rate, log=log)
    try:
        ack = session.handshake()
    except (SessionFailed, ProtocolError) as e:
        print(f"error: handshake failed: {e}", file=sys.stderr)
        session.close(say_bye=False)
        return 3
    print(f"{DIAGNOSTIC_BANNER}. Handshake OK: session {session.session}, receiver={ack.get('receiver')}, "
          f"robot_connected={ack.get('robot_connected')}. Device {redact(serial)}, max {args.max_rate:g} Hz.",
          flush=True)  # fmt: skip

    tracker = FreshnessTracker()
    tracker.on_transport = slot.set_source
    monitor = Monitor(tracker, out=None, on_message=slot.put_message, on_error=slot.put_error)
    source = AdbLogcatSource(adb, serial, monitor.handle_line, tracker)
    session.start_reader()
    source.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    start = time.monotonic_ns()
    last_print = [start]

    def on_tick() -> None:
        now = time.monotonic_ns()
        if now - last_print[0] >= 500_000_000:
            last_print[0] = now
            c = session.counts
            rtt = session.rtt_ns[-1] / 1e6 if session.rtt_ns else None
            print(f"{(now - start) / 1e9:7.1f}s src={'UP' if slot.source_up else 'DOWN'} "
                  f"src_present={slot.counts['present']} src_missing={slot.counts['missing']} "
                  f"sent={c['samples_sent']} present_sent={c['present_sent']} inval={c['invalidations_sent']} "
                  f"hb={c['heartbeats_sent']} skipped={c['source_frames_skipped']} "
                  f"last_rtt={'-' if rtt is None else f'{rtt:.1f}'}ms | {DIAGNOSTIC_BANNER}", flush=True)  # fmt: skip

    def should_stop() -> bool:
        return stop.is_set() or (args.duration is not None and time.monotonic_ns() - start >= args.duration * 1e9)

    result = session.run(should_stop, on_tick)
    end = time.monotonic_ns()
    session.close(say_bye=result == "stopped")
    source.stop()
    if log is not None:
        log.close()
    elapsed = (end - start) / 1e9
    summary = {
        "role": "sender",
        "banner": DIAGNOSTIC_BANNER,
        "robot_connection_attempted": False,
        "session": session.session,
        "source_session": source_session,
        "device": redact(serial),
        "result": result,
        "elapsed_s": round(elapsed, 3),
        "max_rate_hz": args.max_rate,
        "source": dict(slot.counts, present_rate_hz=round(slot.counts["present"] / elapsed, 2) if elapsed else None,
                       events_dropped=slot.events_dropped),  # fmt: skip
        "sent": dict(session.counts, present_sent_rate_hz=round(session.counts["present_sent"] / elapsed, 2)
                     if elapsed else None, first_sent_source_seq=session.first_sent_source_seq,
                     last_sent_source_seq=session.last_sent_source_seq),  # fmt: skip
        "ack_reject_reasons": session.reject_reasons,
        "unacked_at_close_discarded": getattr(session, "unacked_at_close", None),
        "rtt_sender_clock": stats_ms(session.rtt_ns),
        "acquisition_to_send_age_sender_clock": stats_ms(session.acq_to_send_ns),
        "notes": [
            "RTT is application round-trip on the sender's clock; RTT/2 is NOT one-way latency.",
            "source_recv time is when the host read the log line, not a Quest sensor-capture time.",
            "Regular arrival does not prove samples were not delayed inside adb, ssh or TCP.",
        ],
    }
    text = json.dumps(summary, indent=2)
    print(text)
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0 if result == "stopped" else 4


if __name__ == "__main__":
    sys.exit(main())
