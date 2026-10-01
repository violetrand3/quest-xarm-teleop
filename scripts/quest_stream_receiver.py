#!/usr/bin/env python3
"""Diagnostic receiver for the Step 3 Quest controller link. ROBOT NOT CONNECTED.

Listens on 127.0.0.1 ONLY (reach it through a dedicated SSH tunnel), accepts
one client at a time, validates every packet, acknowledges identified packets
so the sender can measure round-trip time on its own clock, and prints a
compact status about twice per second.

There is deliberately no robot, gripper or ROS code here, and no option to
add one. PRESENT means structurally valid input is available; it does not mean
physical tracking is verified or that robot control is allowed.

Thresholds (diagnostic only, NOT robot safety timeouts):
  --transport-stale-ms: receiver-clock time since ANY packet (sample,
      invalidate or heartbeat) was received on the current connection.
  --input-stale-ms: receiver-clock time since the receiver last accepted a
      NEW valid sample with input PRESENT. Heartbeats never refresh it.
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, TextIO

from quest_transport import (
    DEFAULT_PORT,
    DIAGNOSTIC_BANNER,
    FRAME,
    INPUT_MISSING,
    INPUT_PRESENT,
    INPUT_SOURCE_DISCONNECTED,
    INPUT_STALE,
    LOOPBACK,
    MAX_MESSAGE_BYTES,
    PROTOCOL_VERSION,
    TRACKING_VALIDITY,
    FrameTooLarge,
    LineFramer,
    ProtocolError,
    SampleView,
    decode,
    encode,
    require_int,
    require_str,
    stats_ms,
    validate_sample,
)

DEFAULT_STALE_MS = 500.0
SOCKET_TIMEOUT_S = 1.0  # bounds every blocking send (ack) and each recv wait
PRINT_INTERVAL_S = 0.5

TRANSPORT_LIVE = "LIVE"
TRANSPORT_STALE = "STALE"
TRANSPORT_DISCONNECTED = "DISCONNECTED"


@dataclass
class ReceiverSnapshot:
    transport: str
    input: str
    input_reason: str
    session: str | None
    last_source_seq: int
    new_valid: int
    valid_total: int
    valid_age_ms: float | None
    packet_age_ms: float | None
    current: SampleView | None
    errors: int


@dataclass
class SessionStats:
    session: str | None
    opened_ns: int
    closed_ns: int | None = None
    close_reason: str = ""
    packets: int = 0
    samples: int = 0
    valid_samples: int = 0
    missing_samples: int = 0
    invalidations: int = 0
    heartbeats: int = 0
    rejected: int = 0
    reject_reasons: dict[str, int] = field(default_factory=dict)
    valid_gaps_ns: list[int] = field(default_factory=list)
    packet_gaps_ns: list[int] = field(default_factory=list)
    first_source_seq: int | None = None
    last_source_seq: int = 0
    source_seq_skipped: int = 0  # intentional sender-side rate-limit skips


class ReceiverState:
    """Pure state machine; all times are passed in (receiver monotonic ns)."""

    def __init__(self, transport_stale_ms: float = DEFAULT_STALE_MS, input_stale_ms: float = DEFAULT_STALE_MS):
        self.transport_stale_ns = int(transport_stale_ms * 1e6)
        self.input_stale_ns = int(input_stale_ms * 1e6)
        self._lock = threading.Lock()
        self.sessions: list[SessionStats] = []
        self.events: list[tuple[int, str]] = []
        self.errors_total = 0
        self._printed_valid = 0
        self._reset(connected=False)

    # -- session lifecycle ------------------------------------------------
    def _reset(self, connected: bool) -> None:
        self.connected = connected
        self.session: str | None = None
        self.source_session: str | None = None
        self.last_seq = 0
        self.last_source_seq = 0
        self.current: SampleView | None = None
        self.input_reason = "no session" if not connected else "no sample yet"
        self.source_up: bool | None = None
        self.last_packet_ns: int | None = None
        self.last_valid_ns: int | None = None
        self.valid_total = 0

    def open_connection(self, now_ns: int) -> None:
        with self._lock:
            self._reset(connected=True)  # never carry input across sessions
            self._printed_valid = 0
            self.sessions.append(SessionStats(None, now_ns))
            self.events.append((now_ns, "connection opened"))

    def close_connection(self, now_ns: int, reason: str) -> None:
        with self._lock:
            if self.sessions and self.sessions[-1].closed_ns is None:
                self.sessions[-1].closed_ns = now_ns
                self.sessions[-1].close_reason = reason
            self.events.append((now_ns, f"connection closed: {reason}"))
            self._reset(connected=False)  # clear current input immediately

    # -- packet processing -------------------------------------------------
    def _reject(self, st: SessionStats, reason: str, clear: bool) -> None:
        self.errors_total += 1
        st.rejected += 1
        key = reason.split(":")[0][:60]
        st.reject_reasons[key] = st.reject_reasons.get(key, 0) + 1
        if clear:
            self.current = None
            self.input_reason = f"rejected: {reason[:60]}"

    def process(self, pkt: dict[str, Any], now_ns: int) -> dict[str, Any] | None:
        """Apply one decoded packet. Returns the reply packet (or None)."""
        with self._lock:
            st = self.sessions[-1]
            ptype = pkt["type"]
            if ptype == "hello":
                if self.session is not None:
                    self._reject(st, "duplicate hello", clear=True)
                    return self._err("duplicate hello")
                try:
                    session = require_str(pkt, "session", 64)
                    source_session = require_str(pkt, "source_session", 64)
                except ProtocolError as e:
                    self._reject(st, str(e), clear=True)
                    return self._err(str(e))
                self.session, self.source_session = session, source_session
                st.session = session
                self.last_packet_ns = now_ns
                self.events.append((now_ns, f"session {session} started"))
                return {
                    "v": PROTOCOL_VERSION,
                    "type": "hello_ack",
                    "session": session,
                    "receiver": "quest_stream_receiver",
                    "diagnostic_only": True,
                    "robot_connected": False,
                    "frame": FRAME,
                    "tracking_validity": TRACKING_VALIDITY,
                }
            if self.session is None:
                self._reject(st, "packet before hello", clear=True)
                return self._err("packet before hello")
            if pkt.get("session") != self.session:
                self._reject(st, "wrong session", clear=True)
                return self._err("wrong session")
            try:
                seq = require_int(pkt, "seq", 1)
            except ProtocolError as e:
                self._reject(st, str(e), clear=True)
                return self._err(str(e))
            if seq <= self.last_seq:
                # Duplicate / out-of-order: ignore content, keep current state.
                self._reject(st, "duplicate or out-of-order seq", clear=False)
                return self._ack(seq, "rejected", "duplicate or out-of-order seq")
            if seq != self.last_seq + 1:
                self._reject(st, "packet seq gap", clear=False)
            self.last_seq = seq
            if self.last_packet_ns is not None:
                st.packet_gaps_ns.append(now_ns - self.last_packet_ns)
            self.last_packet_ns = now_ns  # transport freshness only
            st.packets += 1

            if ptype == "sample":
                try:
                    view = validate_sample(pkt)
                    if view.source_session != self.source_session:
                        raise ProtocolError("source_session changed within session")
                    if view.source_seq <= self.last_source_seq:
                        raise ProtocolError("duplicate or out-of-order source_seq")
                except ProtocolError as e:
                    # Malformed/invalid input invalidates the current input.
                    self._reject(st, str(e), clear=True)
                    return self._ack(seq, "rejected", str(e))
                if st.first_source_seq is None:
                    st.first_source_seq = view.source_seq
                elif view.source_seq > self.last_source_seq + 1:
                    st.source_seq_skipped += view.source_seq - self.last_source_seq - 1
                self.last_source_seq = st.last_source_seq = view.source_seq
                self.source_up = view.source_up
                st.samples += 1
                if view.input_state == INPUT_PRESENT:
                    if self.last_valid_ns is not None:
                        st.valid_gaps_ns.append(now_ns - self.last_valid_ns)
                    self.current = view
                    self.last_valid_ns = now_ns
                    self.valid_total += 1
                    st.valid_samples += 1
                    self.input_reason = ""
                else:
                    self.current = None  # explicit missing: clear immediately
                    self.input_reason = "sample missing required pose"
                    st.missing_samples += 1
                return self._ack(seq, "ok")

            if ptype == "invalidate":
                try:
                    reason = require_str(pkt, "reason")
                except ProtocolError as e:
                    self._reject(st, str(e), clear=True)
                    return self._ack(seq, "rejected", str(e))
                self.current = None
                self.input_reason = reason
                st.invalidations += 1
                if reason.startswith("source_disconnected"):
                    self.source_up = False
                return self._ack(seq, "ok")

            if ptype == "heartbeat":
                source = pkt.get("source")
                if not isinstance(source, dict) or not isinstance(source.get("up"), bool):
                    self._reject(st, "heartbeat source.up must be boolean", clear=False)
                    return self._ack(seq, "rejected", "bad heartbeat")
                self.source_up = source["up"]  # never touches last_valid_ns
                st.heartbeats += 1
                return self._ack(seq, "ok")

            if ptype == "bye":
                self.events.append((now_ns, "sender said bye"))
                return self._ack(seq, "ok")

            self._reject(st, f"unexpected packet type {ptype}", clear=False)
            return self._ack(seq, "rejected", "unexpected type")

    def note_frame_error(self, reason: str) -> None:
        with self._lock:
            if self.sessions:
                self._reject(self.sessions[-1], reason, clear=True)

    def _ack(self, seq: int, status: str, reason: str = "") -> dict[str, Any]:
        ack: dict[str, Any] = {"v": PROTOCOL_VERSION, "type": "ack", "session": self.session, "seq": seq, "status": status}
        if reason:
            ack["reason"] = reason[:120]
        return ack

    def _err(self, reason: str) -> dict[str, Any]:
        return {"v": PROTOCOL_VERSION, "type": "error", "reason": reason[:120]}

    # -- watchdog view -------------------------------------------------------
    def snapshot(self, now_ns: int, consume: bool = True) -> ReceiverSnapshot:
        """Computed from the caller's clock; works while reads are blocked."""
        with self._lock:
            packet_age = None if self.last_packet_ns is None else now_ns - self.last_packet_ns
            valid_age = None if self.last_valid_ns is None else now_ns - self.last_valid_ns
            if not self.connected:
                transport = TRANSPORT_DISCONNECTED
            elif packet_age is None or packet_age > self.transport_stale_ns:
                transport = TRANSPORT_STALE
            else:
                transport = TRANSPORT_LIVE
            reason = self.input_reason
            if not self.connected:
                inp = INPUT_MISSING
            elif self.source_up is False:
                inp = INPUT_SOURCE_DISCONNECTED
            elif self.current is None:
                inp = INPUT_MISSING
            elif valid_age is None or valid_age > self.input_stale_ns:
                inp = INPUT_STALE
                reason = "no new valid input"
            else:
                inp = INPUT_PRESENT
            new = self.valid_total - self._printed_valid
            if consume:
                self._printed_valid = self.valid_total
            return ReceiverSnapshot(
                transport=transport,
                input=inp,
                input_reason=reason,
                session=self.session,
                last_source_seq=self.last_source_seq,
                new_valid=new,
                valid_total=self.valid_total,
                valid_age_ms=None if valid_age is None else valid_age / 1e6,
                packet_age_ms=None if packet_age is None else packet_age / 1e6,
                current=self.current,
                errors=self.errors_total,
            )


def _f(v: float | None, spec: str) -> str:
    return "-" if v is None else format(v, spec)


def format_snapshot(s: ReceiverSnapshot) -> str:
    c = s.current
    if c is None:
        pos, trig, grip, a, b = "-", "-", "-", "-", "-"
    else:
        t = c.translation("r")
        pos = "-" if t is None else f"({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})"
        bt = c.buttons
        tv, gv = bt.get("rightTrig"), bt.get("rightGrip")
        trig = f"{tv[0]:.2f}" if isinstance(tv, tuple) else "-"
        grip = f"{gv[0]:.2f}" if isinstance(gv, tuple) else "-"
        a = "-" if not isinstance(bt.get("A"), bool) else ("1" if bt["A"] else "0")
        b = "-" if not isinstance(bt.get("B"), bool) else ("1" if bt["B"] else "0")
    why = f" ({s.input_reason})" if s.input != INPUT_PRESENT and s.input_reason else ""
    return (
        f"transport={s.transport:<12} input={s.input}{why} src_seq={s.last_source_seq} new={s.new_valid} "
        f"R.pos={pos} trig={trig} grip={grip} A={a} B={b} valid_age={_f(s.valid_age_ms, '.0f')}ms "
        f"err={s.errors} | {DIAGNOSTIC_BANNER}"
    )


class DiagnosticServer:
    """Loopback-only TCP server; one client at a time; bounded per-connection state."""

    def __init__(self, state: Any, port: int = DEFAULT_PORT, log: TextIO | None = None,
                 clock: Callable[[], int] = time.monotonic_ns, decode_fn: Callable[[bytes], dict[str, Any]] = decode,
                 proto_version: int = PROTOCOL_VERSION,
                 log_fields: Callable[[dict[str, Any], dict[str, Any] | None], dict[str, Any]] | None = None):  # fmt: skip
        # decode_fn / proto_version / log_fields let the separately versioned v2 receiver
        # (quest_pose_stream_receiver.py) reuse this server; the v1 defaults are unchanged.
        self.decode_fn, self.proto_version, self.log_fields = decode_fn, proto_version, log_fields
        self.state = state
        self.log = log
        self.clock = clock
        self._log_lock = threading.Lock()
        self._stop = threading.Event()
        self._active = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((LOOPBACK, port))  # loopback only, by construction
        self._sock.listen(1)
        self._sock.settimeout(0.5)
        self.port = self._sock.getsockname()[1]
        self._accept_thread = threading.Thread(target=self._accept_loop, name="accept", daemon=True)
        self._handler: threading.Thread | None = None
        self._conn: socket.socket | None = None
        self.busy_rejections = 0

    def start(self) -> None:
        self._accept_thread.start()

    def _write_log(self, rec: dict[str, Any]) -> None:
        if self.log is not None:
            with self._log_lock:
                self.log.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if self._active.is_set():
                self.busy_rejections += 1
                try:
                    conn.settimeout(SOCKET_TIMEOUT_S)
                    conn.sendall(encode({"v": self.proto_version, "type": "error", "reason": "busy: one client at a time"}))
                except OSError:
                    pass
                conn.close()
                continue
            self._active.set()
            self._conn = conn
            self._handler = threading.Thread(target=self._handle, args=(conn,), name="client", daemon=True)
            self._handler.start()

    def _handle(self, conn: socket.socket) -> None:
        conn.settimeout(SOCKET_TIMEOUT_S)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # small acks: no Nagle delay
        framer = LineFramer(MAX_MESSAGE_BYTES)
        self.state.open_connection(self.clock())
        self._write_log({"t": self.clock(), "event": "connection opened"})
        reason = "EOF"
        try:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue  # watchdog in the main thread handles staleness
                if not data:
                    reason = "EOF"
                    break
                try:
                    frames = framer.feed(data)
                except FrameTooLarge as e:
                    self.state.note_frame_error(str(e))
                    reason = f"protocol error: {e}"
                    break
                for frame in frames:
                    now = self.clock()
                    try:
                        pkt = self.decode_fn(frame)
                    except ProtocolError as e:
                        self.state.note_frame_error(str(e))
                        self._write_log({"t": now, "event": "malformed", "reason": str(e)[:120]})
                        continue
                    reply = self.state.process(pkt, now)
                    if self.log_fields is not None:
                        self._write_log(dict(self.log_fields(pkt, reply), t=now))
                        if reply is not None:
                            conn.sendall(encode(reply))
                        continue
                    self._write_log({
                        "t": now, "type": pkt.get("type"), "seq": pkt.get("seq"),
                        "source_seq": pkt.get("source_seq"), "input_state": pkt.get("input_state"),
                        "reason": pkt.get("reason"),
                        "reply": None if reply is None else reply.get("status", reply.get("type")),
                    })  # fmt: skip
                    if reply is not None:
                        conn.sendall(encode(reply))  # bounded by SOCKET_TIMEOUT_S
            else:
                reason = "receiver stopping"
        except OSError as e:
            reason = f"socket error: {e.__class__.__name__}"
        finally:
            try:
                conn.close()
            except OSError:
                pass
            self.state.close_connection(self.clock(), reason)
            self._write_log({"t": self.clock(), "event": "connection closed", "reason": reason})
            self._conn = None
            self._active.clear()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        conn = self._conn
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._accept_thread.join(timeout=2.0)
        if self._handler is not None:
            self._handler.join(timeout=3.0)


def summarize(state: ReceiverState, start_ns: int, end_ns: int) -> dict[str, Any]:
    sessions = []
    for s in state.sessions:
        dur = ((s.closed_ns or end_ns) - s.opened_ns) / 1e9
        sessions.append({
            "session": s.session,
            "opened_s": round((s.opened_ns - start_ns) / 1e9, 3),
            "closed_s": None if s.closed_ns is None else round((s.closed_ns - start_ns) / 1e9, 3),
            "close_reason": s.close_reason,
            "duration_s": round(dur, 3),
            "packets": s.packets, "samples": s.samples, "valid_samples": s.valid_samples,
            "valid_rate_hz": round(s.valid_samples / dur, 2) if dur > 0 else None,
            "missing_samples": s.missing_samples, "invalidations": s.invalidations, "heartbeats": s.heartbeats,
            "rejected": s.rejected, "reject_reasons": s.reject_reasons,
            "source_seq_first": s.first_source_seq, "source_seq_last": s.last_source_seq,
            "source_seq_skipped_by_sender_rate_limit": s.source_seq_skipped,
            "valid_interarrival_receiver_clock": stats_ms(s.valid_gaps_ns),
            "packet_interarrival_receiver_clock": stats_ms(s.packet_gaps_ns),
        })  # fmt: skip
    return {
        "role": "receiver",
        "banner": DIAGNOSTIC_BANNER,
        "robot_connection_attempted": False,
        "bind": f"{LOOPBACK}",
        "elapsed_s": round((end_ns - start_ns) / 1e9, 3),
        "errors_total": state.errors_total,
        "sessions": sessions,
        "events": [{"t_s": round((t - start_ns) / 1e9, 3), "event": e} for t, e in state.events],
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="loopback port (host is always 127.0.0.1)")
    ap.add_argument("--transport-stale-ms", type=float, default=DEFAULT_STALE_MS)
    ap.add_argument("--input-stale-ms", type=float, default=DEFAULT_STALE_MS)
    ap.add_argument("--duration", type=float, help="seconds to run (default: until Ctrl+C)")
    ap.add_argument("--log", type=Path, help="JSONL packet/event log (must not exist)")
    ap.add_argument("--summary", type=Path, help="summary JSON (must not exist)")
    args = ap.parse_args(argv)
    for p in (args.log, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists; refusing to overwrite", file=sys.stderr)
            return 2

    log: TextIO | None = None
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        log = args.log.open("x", encoding="utf-8")
    state = ReceiverState(args.transport_stale_ms, args.input_stale_ms)
    try:
        server = DiagnosticServer(state, args.port, log)
    except OSError as e:
        print(f"error: cannot bind {LOOPBACK}:{args.port}: {e}", file=sys.stderr)
        return 2

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    start = time.monotonic_ns()
    print(f"{DIAGNOSTIC_BANNER}. Listening on {LOOPBACK}:{server.port} (loopback only). "
          f"transport stale {args.transport_stale_ms:.0f} ms, input stale {args.input_stale_ms:.0f} ms "
          f"(diagnostic thresholds, not safety timeouts).", flush=True)  # fmt: skip
    server.start()
    try:
        while not stop.wait(PRINT_INTERVAL_S):
            now = time.monotonic_ns()
            print(f"{(now - start) / 1e9:7.1f}s {format_snapshot(state.snapshot(now))}", flush=True)
            if args.duration is not None and now - start >= args.duration * 1e9:
                break
    finally:
        server.stop()
        end = time.monotonic_ns()
        if log is not None:
            log.close()
    summary = summarize(state, start, end)
    text = json.dumps(summary, indent=2)
    print(text)
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
