#!/usr/bin/env python3
"""Diagnostic receiver for the NEW OpenXR reader (protocol v2). ROBOT NOT CONNECTED.

Loopback-only (127.0.0.1), one client at a time, same server/ack machinery as the Step 3
receiver, but for the separately versioned "quest-pose-link" protocol carrying QuestXarmPoseV1
LOCAL-space poses. There is no robot, gripper or ROS code and no option to add one.

Three things are shown separately and none of them is robot readiness:
  transport   LIVE / STALE / DISCONNECTED      (any packet on this connection)
  input       PRESENT / MISSING / STALE / SOURCE_DISCONNECTED   (raw right-controller input)
  continuity  the sender's persistent continuity generation; a pose is "continuous" only if the
              generation AND every identity/counter are unchanged since the previous pose and
              both poses had available input. There is no motion latch here.

Thresholds (diagnostic only, NOT robot safety timeouts):
  --transport-stale-ms  receiver-clock time since any packet arrived.
  --input-stale-ms      receiver-clock time since a pose packet with AVAILABLE right input was
                        accepted. Status/heartbeat and event packets never refresh it.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TextIO

import quest_pose_transport as pt
import quest_pose_v1 as qp
from quest_stream_receiver import DiagnosticServer
from quest_transport import DIAGNOSTIC_BANNER, LOOPBACK, ProtocolError, require_int, require_str, stats_ms

DEFAULT_PORT = 28765
DEFAULT_STALE_MS = 500.0
PRINT_INTERVAL_S = 0.5


@dataclass
class PoseSessionStats:
    session: str | None
    opened_ns: int
    closed_ns: int | None = None
    close_reason: str = ""
    packets: int = 0
    poses: int = 0
    poses_available: int = 0
    poses_unavailable: int = 0
    unavailable_reasons: dict = field(default_factory=dict)
    status_packets: int = 0
    source_events: int = 0
    rejected: int = 0
    reject_reasons: dict = field(default_factory=dict)
    continuous_poses: int = 0
    continuity_gen_first: int | None = None
    continuity_gen_last: int | None = None
    generation_changes_seen: int = 0
    break_reasons: dict = field(default_factory=dict)
    breaks_not_listed: int = 0
    identity_changed_without_generation: int = 0
    decimated_by_sender: int = 0
    source_gap_records: int = 0
    loss_count: int = 0
    source_identities: list = field(default_factory=list)
    valid_gaps_ns: list = field(default_factory=list)
    packet_gaps_ns: list = field(default_factory=list)
    events: list = field(default_factory=list)


@dataclass
class PoseSnapshot:
    transport: str
    input: str
    input_reason: str
    session: str | None
    continuity_gen: int | None
    continuous_run: int
    last_source_seq: int
    source_identity: tuple | None
    new_valid: int
    valid_total: int
    valid_age_ms: float | None
    current: pt.PoseView | None
    errors: int


class PoseReceiverState:
    """Pure state machine (receiver monotonic ns passed in)."""

    def __init__(self, transport_stale_ms: float = DEFAULT_STALE_MS, input_stale_ms: float = DEFAULT_STALE_MS):
        self.transport_stale_ns = int(transport_stale_ms * 1e6)
        self.input_stale_ns = int(input_stale_ms * 1e6)
        self._lock = threading.Lock()
        self.sessions: list[PoseSessionStats] = []
        self.events: list[tuple[int, str]] = []
        self.errors_total = 0
        self._printed_valid = 0
        self._reset(False)

    def _reset(self, connected: bool) -> None:
        self.connected = connected
        self.session: str | None = None
        self.sender_run: str | None = None
        self.last_seq = 0
        self.last_packet_ns: int | None = None
        self.last_valid_ns: int | None = None
        self.current: pt.PoseView | None = None
        self.input_reason = "no session" if not connected else "no sample yet"
        self.source_up: bool | None = None
        self.source_stale = False
        self.cont_gen: int | None = None  # latest generation seen in any packet
        self.last_pose_gen: int | None = None
        self.last_pose_identity: tuple | None = None
        self.last_pose_available = False
        self.source_key: tuple | None = None  # (acq_gen, sid) of the current source
        self.retired: set = set()
        self.last_source_seq = 0
        self.continuous_run = 0
        self.valid_total = 0

    def open_connection(self, now_ns: int) -> None:
        with self._lock:
            self._reset(True)  # nothing is carried across connections
            self._printed_valid = 0
            self.sessions.append(PoseSessionStats(None, now_ns))
            self.events.append((now_ns, "connection opened"))

    def close_connection(self, now_ns: int, reason: str) -> None:
        with self._lock:
            if self.sessions and self.sessions[-1].closed_ns is None:
                self.sessions[-1].closed_ns, self.sessions[-1].close_reason = now_ns, reason
            self.events.append((now_ns, f"connection closed: {reason}"))
            self._reset(False)

    # -- helpers -----------------------------------------------------------------
    def _reject(self, st: PoseSessionStats, reason: str, clear: bool) -> None:
        self.errors_total += 1
        st.rejected += 1
        key = reason.split(":")[0][:60]
        st.reject_reasons[key] = st.reject_reasons.get(key, 0) + 1
        if clear:
            self.current = None
            self.last_pose_available = False
            self.input_reason = f"rejected: {reason[:60]}"

    def _ack(self, seq: int, status: str, reason: str = "", **extra: Any) -> dict[str, Any]:
        ack: dict[str, Any] = {"v": pt.PROTOCOL_VERSION, "type": "ack", "session": self.session, "seq": seq, "status": status}
        if reason:
            ack["reason"] = reason[:120]
        ack.update(extra)
        return ack

    def _err(self, reason: str) -> dict[str, Any]:
        return {"v": pt.PROTOCOL_VERSION, "type": "error", "reason": reason[:120]}

    def _note_continuity(self, st: PoseSessionStats, gen: int, loss: int, gaps: int, breaks: list, not_listed: int) -> None:
        if st.continuity_gen_first is None:
            st.continuity_gen_first = gen
        if self.cont_gen is not None and gen != self.cont_gen:
            st.generation_changes_seen += 1
        self.cont_gen = st.continuity_gen_last = gen
        st.loss_count, st.source_gap_records = loss, gaps
        st.breaks_not_listed += not_listed
        for b in breaks:
            k = str(b.get("reason", "?")).split(":")[0]
            st.break_reasons[k] = st.break_reasons.get(k, 0) + 1

    # -- packets ------------------------------------------------------------------
    def process(self, pkt: dict[str, Any], now_ns: int) -> dict[str, Any] | None:
        with self._lock:
            st = self.sessions[-1]
            ptype = pkt["type"]
            if ptype == "hello":
                if self.session is not None:
                    self._reject(st, "duplicate hello", True)
                    return self._err("duplicate hello")
                try:
                    session = require_str(pkt, "session", 64)
                    sender_run = require_str(pkt, "sender_run", 64)
                    if (pkt.get("protocol"), pkt.get("source_format"), pkt.get("source_schema"), pkt.get("frame")) != (
                            pt.PROTOCOL, pt.SOURCE_FORMAT, pt.SOURCE_SCHEMA, pt.FRAME):  # fmt: skip
                        raise ProtocolError("incompatible source: need %s / %s schema %d / frame %s"
                                            % (pt.PROTOCOL, pt.SOURCE_FORMAT, pt.SOURCE_SCHEMA, pt.FRAME))
                except ProtocolError as e:
                    self._reject(st, str(e), True)
                    return self._err(str(e))
                self.session, self.sender_run, st.session = session, sender_run, session
                self.last_packet_ns = now_ns
                self.events.append((now_ns, f"session {session} started (sender_run {sender_run})"))
                return {"v": pt.PROTOCOL_VERSION, "type": "hello_ack", "session": session,
                        "receiver": "quest_pose_stream_receiver", "protocol": pt.PROTOCOL,
                        "source_format": pt.SOURCE_FORMAT, "source_schema": pt.SOURCE_SCHEMA, "frame": pt.FRAME,
                        "diagnostic_only": True, "robot_connected": False}  # fmt: skip
            if self.session is None:
                self._reject(st, "packet before hello", True)
                return self._err("packet before hello")
            if pkt.get("session") != self.session:
                self._reject(st, "wrong session", True)
                return self._err("wrong session")
            try:
                seq = require_int(pkt, "seq", 1)
            except ProtocolError as e:
                self._reject(st, str(e), True)
                return self._err(str(e))
            if seq <= self.last_seq:
                self._reject(st, "duplicate or out-of-order seq", False)
                return self._ack(seq, "rejected", "duplicate or out-of-order seq")
            if seq != self.last_seq + 1:
                self._reject(st, "packet seq gap", False)
            self.last_seq = seq
            if self.last_packet_ns is not None:
                st.packet_gaps_ns.append(now_ns - self.last_packet_ns)
            self.last_packet_ns = now_ns
            st.packets += 1

            if ptype == "pose":
                try:
                    v = pt.validate_pose(pkt)
                    if v.sender_run != self.sender_run:
                        raise ProtocolError("sender_run changed within connection")
                    key = (v.acq_gen, v.sid)
                    if key in self.retired:
                        raise ProtocolError("retired source identity (old acquisition/app run)")
                    if self.cont_gen is not None and v.continuity_gen < self.cont_gen:
                        raise ProtocolError("continuity generation went backwards")
                    if key == self.source_key and v.source_seq <= self.last_source_seq:
                        raise ProtocolError("duplicate or out-of-order source seq")
                except ProtocolError as e:
                    stale_identity = "retired source identity" in str(e)
                    self._reject(st, str(e), clear=not stale_identity)  # an old-session pose never becomes current
                    return self._ack(seq, "rejected", str(e))
                if key != self.source_key:
                    if self.source_key is not None:
                        self.retired.add(self.source_key)
                    self.source_key, self.last_source_seq = key, 0
                    st.source_identities.append({"acq_gen": v.acq_gen, "sid": v.sid, "build": v.build,
                                                 "first_seq": v.source_seq, "session_gen": v.session_gen})  # fmt: skip
                    self.events.append((now_ns, f"source identity acq_gen={v.acq_gen} sid={v.sid}"))
                # Compare generation AND identities/counters BEFORE presenting the pose as continuous.
                gen_changed = self.last_pose_gen is not None and v.continuity_gen != self.last_pose_gen
                ident_changed = self.last_pose_identity is not None and v.identity() != self.last_pose_identity
                if ident_changed and not gen_changed:
                    st.identity_changed_without_generation += 1  # counter changed although no break was signalled
                continuous = (self.last_pose_identity is not None and not gen_changed and not ident_changed
                              and self.last_pose_available and v.available)  # fmt: skip
                self._note_continuity(st, v.continuity_gen, v.loss_count, v.source_gap_records, v.breaks, v.breaks_not_listed)
                self.last_pose_gen, self.last_pose_identity = v.continuity_gen, v.identity()
                self.last_pose_available = v.available
                self.last_source_seq = v.source_seq
                self.source_up, self.source_stale = v.source_up, v.source_stale
                st.poses += 1
                st.decimated_by_sender += v.decimated_since_last
                if continuous:
                    st.continuous_poses += 1
                    self.continuous_run += 1
                else:
                    self.continuous_run = 0
                if v.available:
                    if self.last_valid_ns is not None:
                        st.valid_gaps_ns.append(now_ns - self.last_valid_ns)
                    self.current, self.last_valid_ns = v, now_ns
                    self.valid_total += 1
                    st.poses_available += 1
                    self.input_reason = ""
                else:
                    self.current = None  # explicit unavailability clears the input when processed
                    st.poses_unavailable += 1
                    st.unavailable_reasons[v.reasons[0]] = st.unavailable_reasons.get(v.reasons[0], 0) + 1
                    self.input_reason = ",".join(v.reasons)
                return self._ack(seq, "ok", available=v.available, continuous=continuous, gen=v.continuity_gen)

            if ptype == "status":
                try:
                    gen, loss, gaps, breaks, nl = pt.validate_continuity(pkt.get("continuity"))
                    up, stale = pt._require_bool(pkt, "source_up"), pt._require_bool(pkt, "stale")
                    if self.cont_gen is not None and gen < self.cont_gen:
                        raise ProtocolError("continuity generation went backwards")
                except ProtocolError as e:
                    self._reject(st, str(e), False)
                    return self._ack(seq, "rejected", str(e))
                self._note_continuity(st, gen, loss, gaps, breaks, nl)
                self.source_up, self.source_stale = up, stale
                st.status_packets += 1
                if not up or stale:
                    self.current = None  # source loss reported: clear now, do not wait for a timeout
                    self.last_pose_available = False
                    self.input_reason = str(pkt.get("source_note", ""))[:80] if not up else "source stale (sender watchdog)"
                return self._ack(seq, "ok", gen=gen)  # never touches last_valid_ns

            if ptype == "source_event":
                try:
                    raw = pkt.get("event")
                    if not isinstance(raw, dict):
                        raise ProtocolError("event must be the source JSON object")
                    ev = qp.decode_record(json.dumps(raw, allow_nan=False))
                    if not isinstance(ev, qp.EventRecord):
                        raise ProtocolError("source_event must carry an event record")
                    require_int(pkt, "acq_gen", 1)
                except (ProtocolError, qp.RecordError, ValueError) as e:
                    self._reject(st, f"bad source_event: {e}", False)
                    return self._ack(seq, "rejected", str(e))
                st.source_events += 1
                if len(st.events) < 400:
                    st.events.append({"t_ns": now_ns, "sid": ev.sid, "seq": ev.seq, "event": ev.event,
                                      "session_state": ev.session_state, "time_ns": ev.time_ns,
                                      "local_epoch": ev.local_epoch, "pose_valid": ev.pose_valid,
                                      "pose_in_previous_space": raw.get("pose_in_previous_space")})  # fmt: skip
                return self._ack(seq, "ok")

            if ptype == "bye":
                self.events.append((now_ns, "sender said bye"))
                return self._ack(seq, "ok")
            self._reject(st, f"unexpected packet type {ptype}", False)
            return self._ack(seq, "rejected", "unexpected type")

    def note_frame_error(self, reason: str) -> None:
        with self._lock:
            if self.sessions:
                self._reject(self.sessions[-1], reason, True)

    # -- watchdog view ---------------------------------------------------------------
    def snapshot(self, now_ns: int, consume: bool = True) -> PoseSnapshot:
        with self._lock:
            packet_age = None if self.last_packet_ns is None else now_ns - self.last_packet_ns
            valid_age = None if self.last_valid_ns is None else now_ns - self.last_valid_ns
            if not self.connected:
                transport = "DISCONNECTED"
            elif packet_age is None or packet_age > self.transport_stale_ns:
                transport = "STALE"
            else:
                transport = "LIVE"
            reason = self.input_reason
            if not self.connected:
                inp = "MISSING"
            elif self.source_up is False:
                inp = "SOURCE_DISCONNECTED"
            elif self.source_stale:
                inp, reason = "STALE", "source stale (sender watchdog)"
            elif self.current is None:
                inp = "MISSING"
            elif valid_age is None or valid_age > self.input_stale_ns:
                inp, reason = "STALE", "no new available input (receiver watchdog)"
            else:
                inp = "PRESENT"
            new = self.valid_total - self._printed_valid
            if consume:
                self._printed_valid = self.valid_total
            return PoseSnapshot(transport, inp, reason, self.session, self.cont_gen, self.continuous_run,
                                self.last_source_seq, self.source_key, new, self.valid_total,
                                None if valid_age is None else valid_age / 1e6,
                                self.current if inp == "PRESENT" else None, self.errors_total)  # fmt: skip


def format_snapshot(s: PoseSnapshot) -> str:
    c = s.current
    if c is None:  # values are hidden whenever input is not PRESENT
        vals = "R.pos=- grip=- trig=- A=- B=- flags=-"
    else:
        g, i = c.record.poses["grip_r"], c.record.inputs["in_r"]
        m = c.matrices["grip_r"]
        b = lambda v: "-" if v is None else ("1" if v else "0")  # noqa: E731
        f = lambda v: "-" if v is None else f"{v:.2f}"  # noqa: E731
        vals = (f"R.pos=({m[0][3]:+.3f},{m[1][3]:+.3f},{m[2][3]:+.3f}) grip={f(i['squeeze'])} trig={f(i['trigger'])} "  # type: ignore[index]
                f"A={b(i['a'])} B={b(i['b'])} flags={g.flags}")  # fmt: skip
    why = f" ({s.input_reason})" if s.input != "PRESENT" and s.input_reason else ""
    src = "-" if s.source_identity is None else f"acq{s.source_identity[0]}/{str(s.source_identity[1])[:8]}"
    age = "-" if s.valid_age_ms is None else f"{s.valid_age_ms:.0f}"
    return (f"transport={s.transport:<12} input={s.input}{why} cont_gen={s.continuity_gen} run={s.continuous_run} "
            f"src={src} seq={s.last_source_seq} new={s.new_valid} {vals} valid_age={age}ms err={s.errors} "
            f"| {DIAGNOSTIC_BANNER}")  # fmt: skip


def log_fields(pkt: dict[str, Any], reply: dict[str, Any] | None) -> dict[str, Any]:
    src = pkt.get("source") if isinstance(pkt.get("source"), dict) else {}
    cont = pkt.get("continuity") if isinstance(pkt.get("continuity"), dict) else {}
    rr = pkt.get("raw_right") if isinstance(pkt.get("raw_right"), dict) else {}
    ev = pkt.get("event") if isinstance(pkt.get("event"), dict) else {}
    return {"type": pkt.get("type"), "seq": pkt.get("seq"), "acq_gen": pkt.get("acq_gen"), "sid": src.get("sid") or ev.get("sid"),
            "source_seq": src.get("seq") or ev.get("seq"), "available": rr.get("available"), "reasons": rr.get("reasons"),
            "gen": cont.get("gen"), "breaks": [b.get("reason") for b in cont.get("breaks", [])] if cont else None,
            "decimated": cont.get("decimated_since_last"), "source_up": pkt.get("source_up", (pkt.get("status") or {}).get("source_up")),
            "stale": pkt.get("stale", (pkt.get("status") or {}).get("stale")), "event": ev.get("event"),
            "reply": None if reply is None else reply.get("status", reply.get("type")),
            "continuous": None if reply is None else reply.get("continuous"), "reject": None if reply is None else reply.get("reason")}  # fmt: skip


def summarize(state: PoseReceiverState, start_ns: int, end_ns: int) -> dict[str, Any]:
    out = []
    for s in state.sessions:
        dur = ((s.closed_ns or end_ns) - s.opened_ns) / 1e9
        d = {k: getattr(s, k) for k in (
            "session", "close_reason", "packets", "poses", "poses_available", "poses_unavailable", "unavailable_reasons",
            "status_packets", "source_events", "rejected", "reject_reasons", "continuous_poses", "continuity_gen_first",
            "continuity_gen_last", "generation_changes_seen", "break_reasons", "breaks_not_listed",
            "identity_changed_without_generation", "decimated_by_sender", "source_gap_records", "loss_count",
            "source_identities")}  # fmt: skip
        d.update(opened_s=round((s.opened_ns - start_ns) / 1e9, 3),
                 closed_s=None if s.closed_ns is None else round((s.closed_ns - start_ns) / 1e9, 3),
                 duration_s=round(dur, 3), available_rate_hz=round(s.poses_available / dur, 2) if dur > 0 else None,
                 available_interarrival_receiver_clock=stats_ms(s.valid_gaps_ns),
                 packet_interarrival_receiver_clock=stats_ms(s.packet_gaps_ns),
                 source_events_received=[dict(e, t_s=round((e["t_ns"] - start_ns) / 1e9, 3)) for e in s.events])  # fmt: skip
        out.append(d)
    return {"role": "receiver", "protocol": pt.PROTOCOL, "protocol_version": pt.PROTOCOL_VERSION, "banner": DIAGNOSTIC_BANNER,
            "robot_connection_attempted": False, "bind": LOOPBACK, "elapsed_s": round((end_ns - start_ns) / 1e9, 3),
            "errors_total": state.errors_total, "sessions": out,
            "events": [{"t_s": round((t - start_ns) / 1e9, 3), "event": e} for t, e in state.events]}  # fmt: skip


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="loopback port (host is always 127.0.0.1)")
    ap.add_argument("--transport-stale-ms", type=float, default=DEFAULT_STALE_MS)
    ap.add_argument("--input-stale-ms", type=float, default=DEFAULT_STALE_MS)
    ap.add_argument("--duration", type=float)
    ap.add_argument("--log", type=Path)
    ap.add_argument("--summary", type=Path)
    args = ap.parse_args(argv)
    for p in (args.log, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists; refusing to overwrite", file=sys.stderr)
            return 2
    log: TextIO | None = None
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        log = args.log.open("x", encoding="utf-8")
    state = PoseReceiverState(args.transport_stale_ms, args.input_stale_ms)
    try:
        server = DiagnosticServer(state, args.port, log, decode_fn=pt.decode, proto_version=pt.PROTOCOL_VERSION,
                                  log_fields=log_fields)  # fmt: skip
    except OSError as e:
        print(f"error: cannot bind {LOOPBACK}:{args.port}: {e}", file=sys.stderr)
        return 2
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    start = time.monotonic_ns()
    print(f"{DIAGNOSTIC_BANNER}. {pt.PROTOCOL} v{pt.PROTOCOL_VERSION} receiver on {LOOPBACK}:{server.port} (loopback only); "
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
    text = json.dumps(summarize(state, start, end), indent=1)
    print(text)
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
