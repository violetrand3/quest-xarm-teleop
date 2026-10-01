"""Input validation and engagement state for the teleop adapter.

InputChannel holds the NEWEST accepted input only (never a queue of hand movements) plus what
the link has said since: the latest continuity generation, source status, reference-change
notices and rejected packets. It is filled by the protocol-v2 receiver thread and read by the
scheduler; neither waits on the other.

Engagement is a clutch:

    WAIT_RELEASE --valid released grip--> ARMED --later valid press--> ACTIVE
         ^                                  |                             |
         +------- any input problem --------+        any problem ---------+
    FAULT_LATCHED --explicit acknowledgement (never while ACTIVE)--> WAIT_RELEASE

It starts in WAIT_RELEASE. A grip that is already held, a missing/null grip value, or input
that comes back with the grip held never engages: a valid release and then a valid press must
both be observed, in one unbroken identity and continuity generation. Reference-space changes
and serious backend or limit faults additionally latch until explicitly acknowledged.

"Input valid" here is a statement about the Quest stream. It is not robot readiness.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

import quest_pose_transport as pt
from quest_pose_stream_receiver import PoseReceiverState

WAIT_RELEASE, ARMED, ACTIVE, FAULT_LATCHED = "WAIT_RELEASE", "ARMED", "ACTIVE", "FAULT_LATCHED"

# Continuity break reasons / source events that mean the LOCAL frame may have changed (or that
# such a change could have been missed). These latch until acknowledged.
REFERENCE_BREAKS = frozenset({"local_space_change", "unknown_space_change", "ref_counter_changed_event_absent",
                              "ref_change_time_changed_event_absent", "events_lost", "event_queue_overflow"})  # fmt: skip
REFERENCE_EVENTS = frozenset({"local_space_change_pending", "other_space_change_pending", "events_lost"})


@dataclass(frozen=True)
class InputSample:
    arrival_ns: int  # this host's monotonic clock when the packet was accepted
    source_seq: int
    t_query_ns: int  # device clock; only differences within one app run are used
    identity: tuple  # (sender_run, acq_gen, build, sid, session_gen, local_epoch, local_change_time_ns)
    continuity_gen: int
    available: bool
    reasons: tuple
    p_m: tuple | None  # right grip pose position in OpenXR LOCAL, metres, unmodified
    q_xyzw: tuple | None  # recorded for diagnosis; never used for the target
    grip: float | None  # side grip (middle finger); None = missing, which is NOT a release
    trigger: float | None  # index trigger; None stays None


@dataclass(frozen=True)
class InputView:
    connected: bool
    source_up: bool | None
    source_stale: bool
    continuity_gen: int | None  # newest generation seen in ANY packet
    sample: InputSample | None
    ref_notices: tuple  # reference-change notices since the last consuming view
    rejects: int  # rejected / malformed packets since the last consuming view


class InputChannel:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._connected = False
        self._source_up: bool | None = None
        self._stale = False
        self._gen: int | None = None
        self._sample: InputSample | None = None
        self._notices: list = []
        self._rejects = 0
        self.packets = 0

    def on_open(self, now_ns: int) -> None:
        with self._lock:
            self._connected, self._source_up, self._stale, self._gen, self._sample = True, None, False, None, None

    def on_close(self, now_ns: int, reason: str) -> None:
        with self._lock:
            self._connected, self._sample = False, None

    def on_reject(self) -> None:
        with self._lock:
            self._rejects += 1
            self._sample = None  # a rejected packet never leaves the previous pose current

    def _note_breaks(self, breaks: list, not_listed: int) -> None:
        for b in breaks:
            key = str(b.get("reason", "?")).split(":")[0]
            if key in REFERENCE_BREAKS:
                self._notices.append(key)
        if not_listed:
            self._notices.append("breaks_not_listed")  # unknown breaks could include a reference change

    def on_packet(self, pkt: dict[str, Any], accepted: bool, now_ns: int) -> None:
        if not accepted:
            self.on_reject()
            return
        ptype = pkt.get("type")
        with self._lock:
            self.packets += 1
            if ptype == "pose":
                v = pt.validate_pose(pkt)  # already validated by the receiver; this is the same pure check
                self._note_breaks(v.breaks, v.breaks_not_listed)
                g, i = v.record.poses["grip_r"], v.record.inputs["in_r"]
                self._sample = InputSample(now_ns, v.source_seq, v.t_query_ns, v.identity(), v.continuity_gen, v.available,
                                           tuple(v.reasons), None if g.p is None else tuple(g.p),
                                           None if g.q is None else tuple(g.q), i.get("squeeze"), i.get("trigger"))  # fmt: skip
                self._gen, self._source_up, self._stale = v.continuity_gen, v.source_up, v.source_stale
            elif ptype == "status":
                gen, _loss, _gaps, breaks, not_listed = pt.validate_continuity(pkt.get("continuity"))
                self._note_breaks(breaks, not_listed)
                self._gen, self._source_up, self._stale = gen, bool(pkt.get("source_up")), bool(pkt.get("stale"))
                if not self._source_up or self._stale:
                    self._sample = None
            elif ptype == "source_event":
                name = (pkt.get("event") or {}).get("event")
                if name in REFERENCE_EVENTS:
                    self._notices.append(f"event:{name}")

    def view(self, consume: bool = True) -> InputView:
        with self._lock:
            v = InputView(self._connected, self._source_up, self._stale, self._gen, self._sample,
                          tuple(self._notices), self._rejects)  # fmt: skip
            if consume:
                self._notices, self._rejects = [], 0
            return v


class ChannelReceiverState(PoseReceiverState):
    """The existing v2 receiver state, unchanged, with every outcome mirrored into an InputChannel."""

    def __init__(self, channel: InputChannel, **kw: Any):
        super().__init__(**kw)
        self.channel = channel

    def open_connection(self, now_ns: int) -> None:
        super().open_connection(now_ns)
        self.channel.on_open(now_ns)

    def close_connection(self, now_ns: int, reason: str) -> None:
        super().close_connection(now_ns, reason)
        self.channel.on_close(now_ns, reason)

    def note_frame_error(self, reason: str) -> None:
        super().note_frame_error(reason)
        self.channel.on_reject()

    def process(self, pkt: dict[str, Any], now_ns: int) -> dict[str, Any] | None:
        before = self.errors_total
        reply = super().process(pkt, now_ns)
        ok = (reply is not None and reply.get("type") in ("ack", "hello_ack") and reply.get("status", "ok") == "ok"
              and self.errors_total == before)  # fmt: skip
        self.channel.on_packet(pkt, ok, now_ns)
        return reply


@dataclass
class EngagementRef:
    identity: tuple
    continuity_gen: int
    p_ctrl0_m: tuple
    p_tool0_m: tuple
    orientation: tuple  # backend-native, captured at engagement, passed back verbatim
    last_p_m: tuple
    last_t_query_ns: int
    last_seq: int
    engaged_ns: int


class Engagement:
    def __init__(self, released_max: float, pressed_min: float, hold_min: float, input_fresh_ns: int):
        if not (0.0 <= released_max < hold_min <= pressed_min <= 1.0):
            raise ValueError("need 0 <= grip_released_max < grip_hold_min <= grip_pressed_min <= 1")
        self.released_max, self.pressed_min, self.hold_min = released_max, pressed_min, hold_min
        self.input_fresh_ns = input_fresh_ns
        self.state = WAIT_RELEASE  # never ACTIVE at start
        self.latch_reasons: list = []
        self.ref: EngagementRef | None = None
        self._armed_key: tuple | None = None  # (identity, continuity_gen, release source_seq)
        self.blocked = "no input yet"
        self.engagements = 0

    # -- input validity: identities and continuity are checked BEFORE any pose is used --------
    def input_problem(self, view: InputView, now_ns: int, key: tuple | None) -> tuple:
        """-> (reason | None, latch). `key` = (identity, continuity_gen) the input must still match."""
        if not view.connected:
            return "transport_disconnected", False
        if view.rejects:
            return "packet_rejected", False
        if view.source_up is not True:
            return "source_down", False
        if view.source_stale:
            return "source_stale", False
        s = view.sample
        if s is None:
            return "no_input", False
        if view.continuity_gen != s.continuity_gen:
            return "continuity_break_after_last_pose", False
        if key is not None:
            if s.identity != key[0]:
                if s.identity[:5] == key[0][:5]:  # only the reference counter / change time differ
                    return "reference_counter_changed", True
                return "source_identity_changed", False
            if s.continuity_gen != key[1]:
                detail = "" if s.available else " (input_unavailable:" + ",".join(s.reasons) + ")"
                return "continuity_generation_changed" + detail, False
        if not s.available:
            return "input_unavailable:" + ",".join(s.reasons), False
        if now_ns - s.arrival_ns > self.input_fresh_ns:
            return "input_stale", False
        if s.p_m is None:
            return "pose_missing", False
        if s.grip is None:
            return "grip_missing", False  # a missing value is never a release
        return None, False

    def latch(self, reason: str) -> None:
        if reason not in self.latch_reasons:
            self.latch_reasons.append(reason)
        self.ref = self._armed_key = None
        self.state = FAULT_LATCHED

    def acknowledge(self) -> tuple:
        """Explicit acknowledgement. Refused while ACTIVE. Clears the latch, never engages."""
        if self.state == ACTIVE:
            return False, "refused: acknowledge only while disengaged"
        if self.state != FAULT_LATCHED:
            return False, "nothing to acknowledge"
        cleared, self.latch_reasons = list(self.latch_reasons), []
        self.state, self.blocked = WAIT_RELEASE, "acknowledged; waiting for a valid release"
        return True, "acknowledged: " + "; ".join(cleared)

    def step_disengaged(self, view: InputView, now_ns: int) -> str | None:
        """WAIT_RELEASE / ARMED logic. Returns "press" when a valid press after a valid release is seen."""
        if self.state == FAULT_LATCHED:
            self.blocked = "acknowledgement required: " + "; ".join(self.latch_reasons)
            return None
        key = None if self._armed_key is None else self._armed_key[:2]
        why, latch = self.input_problem(view, now_ns, key)
        if why is not None:
            if latch:
                self.latch(why)
            elif self.state == ARMED:
                self.state, self._armed_key = WAIT_RELEASE, None  # the release must be observed again
            self.blocked = why
            return None
        s = view.sample
        assert s is not None and s.grip is not None
        if self.state == WAIT_RELEASE:
            if s.grip <= self.released_max:
                self.state, self._armed_key = ARMED, (s.identity, s.continuity_gen, s.source_seq)
                self.blocked = "released; waiting for a valid press"
            else:
                self.blocked = "grip held or partly pressed: release required first"
            return None
        if s.grip >= self.pressed_min and s.source_seq > self._armed_key[2]:  # type: ignore[index]
            return "press"
        self.blocked = "released; waiting for a valid press"
        return None

    def engage(self, view: InputView, p_tool_m: tuple, orientation: tuple, now_ns: int) -> None:
        s = view.sample
        assert s is not None and s.p_m is not None and self.state == ARMED
        self.ref = EngagementRef(s.identity, s.continuity_gen, s.p_m, tuple(p_tool_m), tuple(orientation), s.p_m,
                                 s.t_query_ns, s.source_seq, now_ns)  # fmt: skip
        self.state, self._armed_key, self.blocked = ACTIVE, None, ""
        self.engagements += 1

    def refuse_engagement(self, reason: str) -> None:
        self.state, self._armed_key, self.blocked = WAIT_RELEASE, None, reason

    def active_problem(self, view: InputView, now_ns: int) -> tuple:
        assert self.ref is not None
        why, latch = self.input_problem(view, now_ns, (self.ref.identity, self.ref.continuity_gen))
        if why is not None:
            return why, latch
        if view.sample.grip < self.hold_min:  # type: ignore[union-attr,operator]
            return "grip_released", False
        return None, False

    def disengage(self, reason: str, latch: bool) -> None:
        self.ref = self._armed_key = None  # controller and tool references are cleared
        if latch:
            self.latch(reason)
        elif self.state != FAULT_LATCHED:
            self.state = WAIT_RELEASE
        self.blocked = reason
