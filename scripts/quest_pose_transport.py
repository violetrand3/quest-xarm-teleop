"""Protocol v2 ("quest-pose-link") for the NEW OpenXR reader. DIAGNOSTIC ONLY.

Carries QuestXarmPoseV1 telemetry (LOCAL-space poses from org.questxarm.reader) to the lab's
diagnostic receiver. It is a separate, explicitly versioned source path: protocol v1
(quest_transport.py, the old head-relative wE9ryARX source) is unchanged and the two are never
converted into each other. Nothing here talks to a robot or gripper.

Three separate properties are carried and must not be conflated:
  1. Transport freshness   - packets arriving (any type, incl. status heartbeats).
  2. Raw input availability - the newest source pose is focused, action-active, position and
     orientation VALID and TRACKED, grip action active (see right_input_reasons).
  3. Continuity             - whether a pose may be treated as continuous with the previous one.
     The sender processes EVERY source record (before 30 Hz decimation) through
     ContinuityTracker; any break increments a persistent `continuity_gen` that rides in every
     later packet, so a one-frame loss between transmitted poses cannot be erased.
None of these is robot readiness. There is no motion-enable latch here.

Stdlib only; must run on the lab computer's Python 3.10.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import quest_pose_v1 as qp
import quest_transport as qt
from quest_transport import ProtocolError, require_int, require_str

PROTOCOL = "quest-pose-link"
PROTOCOL_VERSION = 2
SOURCE_FORMAT = "QuestXarmPoseV1"
SOURCE_SCHEMA = qp.SCHEMA_VERSION
FRAME = "openxr_local"  # XR_REFERENCE_SPACE_TYPE_LOCAL, unmodified (no mapping/compensation)
PACKET_TYPES = frozenset({"hello", "hello_ack", "pose", "source_event", "status", "bye", "ack", "error"})
MATRIX_TOL = 1e-9
MAX_BREAKS_PER_PACKET = 16
POSE_KEYS = qp.POSE_KEYS


def decode(frame: bytes) -> dict[str, Any]:
    """Strict decode of one v2 frame (rejects non-finite numbers and other versions)."""
    try:
        obj = json.loads(frame.decode("utf-8"), parse_constant=qt._reject_constant, parse_float=qt._finite_float)
    except ProtocolError:
        raise
    except (UnicodeDecodeError, ValueError) as e:
        raise ProtocolError(f"invalid JSON: {str(e)[:80]}") from None
    if not isinstance(obj, dict):
        raise ProtocolError("packet is not a JSON object")
    if obj.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {obj.get('v')!r} (this endpoint speaks v{PROTOCOL_VERSION})")
    if obj.get("type") not in PACKET_TYPES:
        raise ProtocolError(f"unknown packet type {str(obj.get('type'))[:20]!r}")
    return obj


# --------------------------------------------------------------------------
# Raw input availability (same rule as the Step 4B recorder's FRESH_TRACKED, minus freshness)
# --------------------------------------------------------------------------


def right_input_reasons(rec: qp.PoseRecord) -> list[str]:
    """ALL concurrent reasons the right input is unavailable in this record ([] = available).

    The left controller and the headset pose are not required. A finite pose alone is never
    enough; either tracked bit alone being clear makes the input unavailable.
    """
    g = rec.poses["grip_r"]
    out: list[str] = []
    if not rec.focused or rec.session_state != "FOCUSED":
        out.append("unfocused")
    if not g.queried or g.result < 0:
        out.append("locate_failed")
    if g.active is not True:
        out.append("pose_action_inactive")
    if not g.pos_valid or g.p is None:
        out.append("pos_invalid")
    if not g.ori_valid or g.q is None:
        out.append("ori_invalid")
    if g.pos_valid and not g.pos_tracked:
        out.append("pos_untracked")
    if g.ori_valid and not g.ori_tracked:
        out.append("ori_untracked")
    if rec.inputs["in_r"]["squeeze"] is None:
        out.append("grip_action_inactive")
    return out


def matrices_of(rec: qp.PoseRecord) -> dict[str, list[list[float]] | None]:
    """T_LOCAL_from_X (column-vector convention) per pose, or None when p/q are not both valid."""
    return {k: rec.poses[k].matrix() for k in POSE_KEYS}


# --------------------------------------------------------------------------
# Sender-side continuity tracking (pure; times are passed in)
# --------------------------------------------------------------------------


@dataclass
class ContinuityTracker:
    """Sees EVERY decoded source record, in order, before any decimation.

    continuity_gen increments on every continuity break and is never reset for the life of the
    sender process. A break is recorded for: acquisition (re)start, app-run id change, OpenXR
    session begin/end, any non-FOCUSED session state event, a source sequence gap (missing
    record(s) of unknown type), parse error, overflow, a LOCAL or unknown-type reference-space
    change event, events_lost, a changed reference counter/change-time in a pose record even if
    the event itself was never seen, raw right-input loss, source down, and watchdog staleness.
    """

    expect_build: str | None = None
    stale_ns: int = 500_000_000
    max_pending: int = 64

    acq_gen: int = 0
    build: str | None = None
    sid: str | None = None
    session_gen: int = 0  # session_begin events seen for this sid in this acquisition generation
    last_seq: int | None = None
    local_epoch: int | None = None
    local_change_time_ns: int | None = None
    continuity_gen: int = 0
    loss_count: int = 0
    source_gap_records: int = 0
    available: bool = False
    source_up: bool = False
    source_note: str = "starting"
    stale: bool = False
    last_valid_recv_ns: int | None = None
    last_record_recv_ns: int | None = None
    gen_started_ns: int | None = None
    pending: deque = field(default_factory=deque)
    breaks_dropped: int = 0
    break_counts: dict = field(default_factory=dict)
    counts: dict = field(default_factory=lambda: {
        "records": 0, "pose": 0, "pose_available": 0, "pose_unavailable": 0, "event": 0, "overflow": 0,
        "probe": 0, "parse_errors": 0, "foreign_build": 0, "duplicate_or_old_seq": 0})  # fmt: skip
    unavailable_reasons: dict = field(default_factory=dict)
    stale_detections: list = field(default_factory=list)

    # -- helpers ------------------------------------------------------------------
    def _break(self, reason: str, now_ns: int, source_seq: int | None = None, detail: str = "") -> None:
        self.continuity_gen += 1
        key = reason.split(":")[0]
        self.break_counts[key] = self.break_counts.get(key, 0) + 1
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()  # the generation counter itself still records the break
            self.breaks_dropped += 1
        self.pending.append({"gen": self.continuity_gen, "reason": reason[:80], "source_seq": source_seq,
                             "host_mono_ns": now_ns, "detail": detail[:80]})  # fmt: skip

    def _lose_input(self, reason: str, now_ns: int, source_seq: int | None) -> None:
        if self.available:
            self.loss_count += 1
            self.available = False
            self._break(f"input_lost:{reason}", now_ns, source_seq)

    # -- acquisition lifecycle ------------------------------------------------------
    def new_generation(self, now_ns: int, note: str = "") -> int:
        self.acq_gen += 1
        self.gen_started_ns = now_ns
        self.available = False
        self.stale = False
        self.build = self.sid = None
        self.session_gen = 0
        self.last_seq = self.local_epoch = self.local_change_time_ns = None
        self.last_valid_recv_ns = None
        self._break("acquisition_restart", now_ns, detail=note)
        return self.acq_gen

    def set_source(self, up: bool, note: str, now_ns: int) -> None:
        was_up = self.source_up
        self.source_up, self.source_note = up, note
        if not up and was_up:
            if self.available:
                self.loss_count += 1
            self.available = False
            self._break(f"source_down:{note}", now_ns)

    def watchdog(self, now_ns: int) -> bool:
        """Independent stale check (called from the sender loop, not the reader thread)."""
        ref = self.last_valid_recv_ns if self.last_valid_recv_ns is not None else self.gen_started_ns
        if ref is not None and not self.stale and now_ns - ref > self.stale_ns:
            self.stale = True
            had = self.available
            if had:
                self.loss_count += 1
                self.available = False
            self.stale_detections.append({"last_valid_host_mono_ns": self.last_valid_recv_ns, "detected_host_mono_ns": now_ns,
                                          "delay_ms": None if self.last_valid_recv_ns is None else round((now_ns - self.last_valid_recv_ns) / 1e6, 1),
                                          "had_available_input": had, "acq_gen": self.acq_gen})  # fmt: skip
            self._break("source_stale", now_ns)
        return self.stale

    # -- per-record processing --------------------------------------------------------
    def on_parse_error(self, detail: str, now_ns: int) -> None:
        self.counts["records"] += 1
        self.counts["parse_errors"] += 1
        self.last_record_recv_ns = now_ns
        if self.available:
            self.loss_count += 1
            self.available = False
        self._break("parse_error", now_ns, detail=detail)

    def on_record(self, rec: Any, now_ns: int) -> str:
        """Returns 'pose', 'event', 'overflow', 'probe', 'foreign' or 'duplicate'."""
        self.counts["records"] += 1
        if self.expect_build is not None and rec.build != self.expect_build:
            self.counts["foreign_build"] += 1
            return "foreign"  # another build's output is never mixed in
        self.last_record_recv_ns = now_ns
        if rec.sid != self.sid:
            if self.sid is not None:
                self._lose_input("app_run_changed", now_ns, rec.seq)
                self._break("app_run_changed", now_ns, rec.seq)
            self.sid, self.build = rec.sid, rec.build
            self.session_gen, self.last_seq = 0, None
            self.local_epoch = self.local_change_time_ns = None
        if self.last_seq is not None:
            if rec.seq <= self.last_seq:
                self.counts["duplicate_or_old_seq"] += 1
                return "duplicate"
            if rec.seq > self.last_seq + 1:
                missing = rec.seq - self.last_seq - 1
                self.source_gap_records += missing
                # The missing record's type is unknown (pose and event records share the counter);
                # it is NOT assumed harmless.
                self._lose_input("source_gap", now_ns, rec.seq)
                self._break("source_gap", now_ns, rec.seq, f"{missing} missing record(s), type unknown")
        self.last_seq = rec.seq

        if isinstance(rec, qp.EventRecord):
            self.counts["event"] += 1
            ev = rec.event
            if ev == "session_begin":
                self.session_gen += 1
                self._lose_input("session_begin", now_ns, rec.seq)
                self._break("session_begin", now_ns, rec.seq)
            elif ev == "session_end":
                self._lose_input("session_end", now_ns, rec.seq)
                self._break("session_end", now_ns, rec.seq)
            elif ev == "session_state":
                if rec.session_state != "FOCUSED":
                    self._lose_input("unfocused_event", now_ns, rec.seq)
                    self._break(f"session_state:{rec.session_state}", now_ns, rec.seq)
            elif ev == "local_space_change_pending":
                self.local_epoch, self.local_change_time_ns = rec.local_epoch, rec.time_ns
                self._break("local_space_change", now_ns, rec.seq, f"changeTime {rec.time_ns}")
            elif ev == "other_space_change_pending":
                self._break("unknown_space_change", now_ns, rec.seq, f"type not emitted; changeTime {rec.time_ns}")
            elif ev == "events_lost":
                self._lose_input("events_lost", now_ns, rec.seq)
                self._break("events_lost", now_ns, rec.seq)
            return "event"
        if isinstance(rec, qp.OverflowRecord):
            self.counts["overflow"] += 1
            self._lose_input("source_overflow", now_ns, rec.seq)
            self._break("source_overflow", now_ns, rec.seq)
            return "overflow"
        if isinstance(rec, qp.ProbeRecord):
            self.counts["probe"] += 1
            return "probe"

        # pose record: compare identities/counters BEFORE treating it as continuous
        self.counts["pose"] += 1
        if self.local_epoch is not None and rec.local_epoch != self.local_epoch:
            self._break("ref_counter_changed_event_absent", now_ns, rec.seq,
                        f"local_epoch {self.local_epoch}->{rec.local_epoch}")  # fmt: skip
        elif self.local_change_time_ns is not None and rec.local_change_time_ns != self.local_change_time_ns:
            self._break("ref_change_time_changed_event_absent", now_ns, rec.seq)
        self.local_epoch, self.local_change_time_ns = rec.local_epoch, rec.local_change_time_ns
        reasons = right_input_reasons(rec)
        if reasons:
            self.counts["pose_unavailable"] += 1
            self.unavailable_reasons[reasons[0]] = self.unavailable_reasons.get(reasons[0], 0) + 1
            self._lose_input(reasons[0], now_ns, rec.seq)
        else:
            self.counts["pose_available"] += 1
            self.available = True
            self.stale = False
            self.last_valid_recv_ns = now_ns
        return "pose"

    # -- packet fields ------------------------------------------------------------------
    def drain_breaks(self) -> tuple[list[dict[str, Any]], int]:
        out = list(self.pending)[-MAX_BREAKS_PER_PACKET:]
        extra = len(self.pending) - len(out)
        self.pending.clear()
        dropped, self.breaks_dropped = self.breaks_dropped + extra, 0
        return out, dropped

    def continuity_fields(self) -> dict[str, Any]:
        breaks, dropped = self.drain_breaks()
        return {"gen": self.continuity_gen, "loss_count": self.loss_count, "source_gap_records": self.source_gap_records,
                "breaks": breaks, "breaks_not_listed": dropped}  # fmt: skip


# --------------------------------------------------------------------------
# Packet validation (receiver side)
# --------------------------------------------------------------------------


@dataclass
class PoseView:
    """A validated v2 pose packet. Values are exactly as sent; missing stays None."""

    sender_run: str
    acq_gen: int
    build: str
    sid: str
    session_gen: int
    source_seq: int
    t_query_ns: int
    record: qp.PoseRecord
    matrices: dict[str, list[list[float]] | None]
    reasons: list[str]
    continuity_gen: int
    loss_count: int
    source_gap_records: int
    breaks: list[dict[str, Any]]
    breaks_not_listed: int
    decimated_since_last: int
    source_up: bool
    source_stale: bool

    @property
    def available(self) -> bool:
        return not self.reasons

    def identity(self) -> tuple[Any, ...]:
        """Everything that must be unchanged for a pose to be continuous with the previous one."""
        return (self.sender_run, self.acq_gen, self.build, self.sid, self.session_gen,
                self.record.local_epoch, self.record.local_change_time_ns)  # fmt: skip


def _require_bool(pkt: dict[str, Any], key: str) -> bool:
    v = pkt.get(key)
    if not isinstance(v, bool):
        raise ProtocolError(f"{key} must be a boolean")
    return v


def validate_continuity(c: Any) -> tuple[int, int, int, list[dict[str, Any]], int]:
    if not isinstance(c, dict):
        raise ProtocolError("continuity must be an object")
    gen, loss, gaps = require_int(c, "gen"), require_int(c, "loss_count"), require_int(c, "source_gap_records")
    breaks = c.get("breaks")
    if not isinstance(breaks, list) or len(breaks) > MAX_BREAKS_PER_PACKET or not all(isinstance(b, dict) for b in breaks):
        raise ProtocolError("continuity.breaks must be a short list of objects")
    for b in breaks:
        require_int(b, "gen")
        require_str(b, "reason", 80)
    return gen, loss, gaps, breaks, require_int(c, "breaks_not_listed")


def validate_pose(pkt: dict[str, Any]) -> PoseView:
    if pkt.get("frame") != FRAME:
        raise ProtocolError(f"frame must be {FRAME!r}")
    sender_run = require_str(pkt, "sender_run", 64)
    acq_gen = require_int(pkt, "acq_gen", 1)
    src = pkt.get("source")
    if not isinstance(src, dict):
        raise ProtocolError("source must be an object")
    build, sid = require_str(src, "build", 64), require_str(src, "sid", 64)
    session_gen, seq = require_int(src, "session_gen"), require_int(src, "seq", 1)
    t_query = require_int(src, "t_query_ns")
    require_int(src, "recv_mono_ns")
    require_int(pkt, "send_mono_ns")
    raw = pkt.get("record")
    if not isinstance(raw, dict):
        raise ProtocolError("record must be the source JSON object")
    try:
        rec = qp.decode_record(json.dumps(raw, allow_nan=False))
    except (qp.RecordError, ValueError) as e:
        raise ProtocolError(f"source record invalid: {str(e)[:80]}") from None
    if not isinstance(rec, qp.PoseRecord):
        raise ProtocolError("pose packet must carry a pose record")
    if (rec.build, rec.sid, rec.seq, rec.t_query_ns) != (build, sid, seq, t_query):
        raise ProtocolError("source identity fields disagree with the carried record")
    mats = pkt.get("matrices")
    if not isinstance(mats, dict) or set(mats) != set(POSE_KEYS):
        raise ProtocolError("matrices must have exactly keys head, grip_l, grip_r (null when unavailable)")
    expect = matrices_of(rec)
    out: dict[str, list[list[float]] | None] = {}
    for k in POSE_KEYS:
        if mats[k] is None or expect[k] is None:
            if mats[k] is not None or expect[k] is not None:
                raise ProtocolError(f"matrix {k!r} presence disagrees with the record's valid p/q")
            out[k] = None
            continue
        m = qt.validate_matrix(mats[k], k)
        if any(abs(m[i][j] - expect[k][i][j]) > MATRIX_TOL for i in range(4) for j in range(4)):  # type: ignore[index]
            raise ProtocolError(f"matrix {k!r} is not the unmodified LOCAL pose of the record")
        out[k] = m
    rr = pkt.get("raw_right")
    reasons = right_input_reasons(rec)
    if not isinstance(rr, dict) or rr.get("available") is not (not reasons) or rr.get("reasons") != reasons:
        raise ProtocolError("raw_right disagrees with the record")
    gen, loss, gaps, breaks, not_listed = validate_continuity(pkt.get("continuity"))
    st = pkt.get("status")
    if not isinstance(st, dict):
        raise ProtocolError("status must be an object")
    return PoseView(sender_run, acq_gen, build, sid, session_gen, seq, t_query, rec, out, reasons, gen, loss, gaps,
                    breaks, not_listed, require_int(pkt.get("continuity"), "decimated_since_last"),
                    _require_bool(st, "source_up"), _require_bool(st, "stale"))  # fmt: skip


def pose_packet(sender_run: str, acq_gen: int, session_gen: int, rec: qp.PoseRecord, raw: dict[str, Any],
                recv_ns: int, device_log_time: str, continuity: dict[str, Any], decimated: int,
                source_up: bool, stale: bool) -> dict[str, Any]:  # fmt: skip
    reasons = right_input_reasons(rec)
    return {
        "type": "pose", "frame": FRAME, "sender_run": sender_run, "acq_gen": acq_gen,
        "source": {"build": rec.build, "sid": rec.sid, "session_gen": session_gen, "seq": rec.seq,
                   "t_query_ns": rec.t_query_ns, "t_query_kind": "predicted_display_time",
                   "recv_mono_ns": recv_ns, "device_log_time": device_log_time},
        "continuity": dict(continuity, decimated_since_last=decimated),
        "raw_right": {"available": not reasons, "reasons": reasons},
        "record": raw,  # the original source JSON object, unmodified (null stays null)
        "matrices": matrices_of(rec),
        "status": {"source_up": source_up, "stale": stale},
    }  # fmt: skip
