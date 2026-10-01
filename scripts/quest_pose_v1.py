#!/usr/bin/env python3
"""Decoder and live monitor for QuestXarmPoseV1 telemetry (Step 4A). DIAGNOSTIC ONLY.

Source app: org.questxarm.reader ("Quest xArm Input Diagnostic"), built from
external/oculus_reader_openxr branch questxarm/step04a-local-space. Separate
from the legacy wE9ryARX format, which quest_input_monitor.py still handles.

Conventions (see docs/step04a_openxr_reader.md):
  * Every pose is an OpenXR XrPosef located in XR_REFERENCE_SPACE_TYPE_LOCAL:
    p = [x, y, z] metres, q = [x, y, z, w] unit quaternion.
  * pose_matrix() returns T_local_from_X as a 4x4 nested list (rows), using
    COLUMN-VECTOR semantics: [x_L, y_L, z_L, 1]^T = T @ [x_X, y_X, z_X, 1]^T.
    The translation is the last column; the rotation is the upper-left 3x3.
  * head_relative(grip, head) = inverse(T_local_head) @ T_local_grip
    (only for diagnosis; published data is never head-relative).
  * t_query_ns is the XrTime used for xrLocateSpace (predicted display time),
    not a sensor-capture time. Buttons are not claimed to share that timestamp.

Pose status (never inferred from matrix presence or message arrival):
  missing          not queried, xrLocateSpace failed, or pose action inactive
  invalid          queried/active but position or orientation not VALID
  valid_untracked  both VALID, but not both TRACKED
  tracked          both VALID and both TRACKED
No identity fallback and no reuse of an older pose: absent values stay None.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from quest_input_monitor import AdbLogcatSource, FreshnessTracker, find_adb, parse_logcat_line, pick_serial, redact

TAG = "QuestXarmPoseV1"
PACKAGE = "org.questxarm.reader"
ACTIVITY = "com.rail.oculus.teleop.MainActivity"  # Java class name kept from upstream
SCHEMA_VERSION = 1
SPACE = "LOCAL"
# Application cap for one complete serialized record (UTF-8 bytes, no NUL); same as the app's
# questxarm::kMaxRecordBytes since the Step 4B logging repair. Not an Android maximum.
MAX_RECORD_BYTES = 3000
LEGACY_TRUNCATION_BYTES = 1023  # __android_log_print() cut-off seen with build s4a-9e6f1f24dbed
QUAT_NORM_TOL = 1e-3

ORIENTATION_VALID = 0x1
POSITION_VALID = 0x2
ORIENTATION_TRACKED = 0x4
POSITION_TRACKED = 0x8

MISSING, INVALID, VALID_UNTRACKED, TRACKED = "missing", "invalid", "valid_untracked", "tracked"
POSE_KEYS = ("head", "grip_l", "grip_r")
SESSION_STATES = {"IDLE", "READY", "SYNCHRONIZED", "VISIBLE", "FOCUSED", "STOPPING", "LOSS_PENDING", "EXITING", "UNKNOWN"}


class RecordError(ValueError):
    """Malformed, truncated, non-finite or inconsistent record."""


# --------------------------------------------------------------------------
# Strict JSON
# --------------------------------------------------------------------------


def _reject_constant(name: str) -> Any:
    raise RecordError(f"non-finite constant {name}")


def _finite_float(text: str) -> float:
    v = float(text)
    if not math.isfinite(v):
        raise RecordError(f"non-finite number {text[:20]}")
    return v


def loads_strict(text: str) -> dict[str, Any]:
    if len(text.encode("utf-8")) > MAX_RECORD_BYTES:
        raise RecordError("record over the application size cap")
    try:
        obj = json.loads(text, parse_constant=_reject_constant, parse_float=_finite_float)
    except RecordError:
        raise
    except ValueError as e:
        raise RecordError(f"malformed or truncated JSON: {str(e)[:60]}") from None
    if not isinstance(obj, dict):
        raise RecordError("record is not an object")
    return obj


# --------------------------------------------------------------------------
# Records
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Pose:
    key: str
    queried: bool
    result: int
    active: bool | None  # None for the head (no pose action)
    flags: int
    p: tuple[float, float, float] | None
    q: tuple[float, float, float, float] | None

    @property
    def pos_valid(self) -> bool:
        return bool(self.flags & POSITION_VALID)

    @property
    def ori_valid(self) -> bool:
        return bool(self.flags & ORIENTATION_VALID)

    @property
    def pos_tracked(self) -> bool:
        return bool(self.flags & POSITION_TRACKED)

    @property
    def ori_tracked(self) -> bool:
        return bool(self.flags & ORIENTATION_TRACKED)

    @property
    def status(self) -> str:
        if not self.queried or self.result < 0 or self.active is False:
            return MISSING
        if not (self.pos_valid and self.ori_valid) or self.p is None or self.q is None:
            return INVALID
        if not (self.pos_tracked and self.ori_tracked):
            return VALID_UNTRACKED
        return TRACKED

    def matrix(self) -> list[list[float]] | None:
        """T_local_from_X (column-vector convention) or None when not fully valid."""
        if self.p is None or self.q is None:
            return None
        return pose_matrix(self.p, self.q)


@dataclass(frozen=True)
class PoseRecord:
    build: str
    sid: str
    seq: int
    t_query_ns: int
    session_state: str
    focused: bool
    local_epoch: int
    local_change_time_ns: int
    poses: dict[str, Pose]
    inputs: dict[str, dict[str, Any]]  # "in_l"/"in_r": field -> value or None (inactive)


@dataclass(frozen=True)
class EventRecord:
    build: str
    sid: str
    seq: int
    event: str
    local_epoch: int
    time_ns: int
    session_state: str | None
    pose_valid: bool | None


@dataclass(frozen=True)
class OverflowRecord:
    """A record the app dropped at the source because it exceeded the cap. Never input."""

    build: str
    sid: str
    seq: int
    attempted_bytes: int | None
    cap_bytes: int | None


@dataclass(frozen=True)
class ProbeRecord:
    """Opt-in logging self-test record. NON-POSE: never input, never readiness."""

    build: str
    sid: str
    seq: int
    probe_id: int
    target_bytes: int
    utf8: bool
    raw: str


def expected_probe(build: str, sid: str, seq: int, probe_id: int, target_bytes: int, utf8: bool) -> str:
    """Byte-exact mirror of questxarm::make_probe() for end-to-end payload verification."""
    head = (f'{{"v":1,"type":"probe","non_pose":true,"build":"{build}","sid":"{sid}","seq":{seq},'
            f'"probe_id":{probe_id},"target_bytes":{target_bytes},"utf8":{"true" if utf8 else "false"},"pad":"')  # fmt: skip
    tail = f'","end":"QXEND-{probe_id}"}}'
    need = target_bytes - len(head.encode()) - len(tail.encode())
    pad = ""
    if need > 0:
        if utf8:
            pad = ("x" if need % 2 else "") + "\u00e9" * (need // 2)
        else:
            pat = "abcdefghijklmnopqrstuvwxyz0123456789"
            pad = (pat * (need // len(pat) + 1))[:need]
    return head + pad + tail


def _req(obj: dict[str, Any], key: str, typ: type | tuple[type, ...]) -> Any:
    v = obj.get(key)
    if isinstance(v, bool) and typ in (int, float, (int, float)):
        raise RecordError(f"{key}: bool not allowed")
    if not isinstance(v, typ):
        raise RecordError(f"{key}: expected {typ}")
    return v


def _vec(v: Any, n: int, key: str) -> tuple[float, ...] | None:
    if v is None:
        return None
    if not isinstance(v, list) or len(v) != n or any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in v):
        raise RecordError(f"{key}: expected {n} numbers or null")
    return tuple(float(x) for x in v)


def _pose(obj: Any, key: str) -> Pose:
    if not isinstance(obj, dict):
        raise RecordError(f"{key}: missing pose object")
    queried = _req(obj, "queried", bool)
    result = _req(obj, "result", int)
    flags = _req(obj, "flags", int)
    if flags < 0:
        raise RecordError(f"{key}: negative flags")
    for name, bit in (("pos_valid", POSITION_VALID), ("ori_valid", ORIENTATION_VALID),
                      ("pos_tracked", POSITION_TRACKED), ("ori_tracked", ORIENTATION_TRACKED)):  # fmt: skip
        if _req(obj, name, bool) != bool(flags & bit):
            raise RecordError(f"{key}: {name} disagrees with flags")
    active = obj.get("active")
    if key == "head":
        if active is not None:
            raise RecordError("head has no pose action")
    elif not isinstance(active, bool):
        raise RecordError(f"{key}: active must be boolean")
    p = _vec(obj.get("p"), 3, f"{key}.p")
    q = _vec(obj.get("q"), 4, f"{key}.q")
    if p is not None and not flags & POSITION_VALID:
        raise RecordError(f"{key}: position given without POSITION_VALID")
    if q is not None:
        if not flags & ORIENTATION_VALID:
            raise RecordError(f"{key}: orientation given without ORIENTATION_VALID")
        if abs(math.sqrt(sum(x * x for x in q)) - 1.0) > QUAT_NORM_TOL:
            raise RecordError(f"{key}: quaternion not unit length")
    if obj.get("nonfinite"):
        raise RecordError(f"{key}: app reported non-finite pose values")
    return Pose(key, queried, result, active, flags, p, q)  # type: ignore[arg-type]


_INPUT_FIELDS = {
    "in_l": ("trigger", "squeeze", "thumbstick", "thumbstick_click", "thumbstick_touch", "trigger_touch",
             "thumbrest_touch", "x", "y", "menu"),
    "in_r": ("trigger", "squeeze", "thumbstick", "thumbstick_click", "thumbstick_touch", "trigger_touch",
             "thumbrest_touch", "a", "b"),
}  # fmt: skip
_ANALOG = {"trigger", "squeeze"}


def _inputs(obj: Any, key: str) -> dict[str, Any]:
    if not isinstance(obj, dict):
        raise RecordError(f"{key}: missing input object")
    out: dict[str, Any] = {}
    for name in _INPUT_FIELDS[key]:
        if name not in obj:
            raise RecordError(f"{key}.{name}: field absent (inactive must be explicit null)")
        v = obj[name]
        if v is None:
            out[name] = None  # action inactive: missing, NOT zero
        elif name in _ANALOG:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise RecordError(f"{key}.{name}: expected number")
            out[name] = float(v)
        elif name == "thumbstick":
            out[name] = _vec(v, 2, f"{key}.{name}")
        else:
            if not isinstance(v, bool):
                raise RecordError(f"{key}.{name}: expected boolean")
            out[name] = v
    return out


def decode_record(text: str) -> PoseRecord | EventRecord | OverflowRecord | ProbeRecord:
    obj = loads_strict(text)
    if obj.get("v") != SCHEMA_VERSION:
        raise RecordError(f"unsupported schema version {obj.get('v')!r}")
    typ = obj.get("type")
    build = _req(obj, "build", str)
    sid = _req(obj, "sid", str)
    seq = _req(obj, "seq", int)
    if typ == "overflow":
        ab, cb = obj.get("attempted_bytes"), obj.get("cap_bytes")
        return OverflowRecord(build, sid, seq, ab if isinstance(ab, int) else None, cb if isinstance(cb, int) else None)
    if typ == "probe":
        rec = ProbeRecord(build, sid, seq, _req(obj, "probe_id", int), _req(obj, "target_bytes", int),
                          _req(obj, "utf8", bool), text)  # fmt: skip
        if obj.get("non_pose") is not True or obj.get("end") != f"QXEND-{rec.probe_id}":
            raise RecordError("probe record incomplete (missing non_pose/end marker)")
        return rec
    if typ == "event":
        if obj.get("space") != SPACE:
            raise RecordError("event space must be LOCAL")
        pv = obj.get("pose_valid")
        return EventRecord(build, sid, seq, _req(obj, "event", str), _req(obj, "local_epoch", int),
                           _req(obj, "time_ns", int), obj.get("session_state"),
                           pv if isinstance(pv, bool) else None)  # fmt: skip
    if typ != "pose":
        raise RecordError(f"unknown record type {str(typ)[:20]!r}")
    if obj.get("space") != SPACE:
        raise RecordError("pose space must be LOCAL")
    if obj.get("t_query_kind") != "predicted_display_time":
        raise RecordError("t_query_kind must be predicted_display_time")
    state = _req(obj, "session_state", str)
    if state not in SESSION_STATES:
        raise RecordError(f"unknown session_state {state[:20]!r}")
    return PoseRecord(
        build=build, sid=sid, seq=seq,
        t_query_ns=_req(obj, "t_query_ns", int),
        session_state=state,
        focused=_req(obj, "focused", bool),
        local_epoch=_req(obj, "local_epoch", int),
        local_change_time_ns=_req(obj, "local_change_time_ns", int),
        poses={k: _pose(obj.get(k), k) for k in POSE_KEYS},
        inputs={k: _inputs(obj.get(k), k) for k in ("in_l", "in_r")},
    )  # fmt: skip


# --------------------------------------------------------------------------
# Pose math (column-vector convention)
# --------------------------------------------------------------------------


def quat_to_rot(q: tuple[float, float, float, float]) -> list[list[float]]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def pose_matrix(p: tuple[float, float, float], q: tuple[float, float, float, float]) -> list[list[float]]:
    r = quat_to_rot(q)
    return [r[0] + [p[0]], r[1] + [p[1]], r[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def invert_rigid(t: list[list[float]]) -> list[list[float]]:
    rt = [[t[j][i] for j in range(3)] for i in range(3)]
    p = [t[0][3], t[1][3], t[2][3]]
    tp = [-sum(rt[i][k] * p[k] for k in range(3)) for i in range(3)]
    return [rt[0] + [tp[0]], rt[1] + [tp[1]], rt[2] + [tp[2]], [0.0, 0.0, 0.0, 1.0]]


def apply(t: list[list[float]], point: tuple[float, float, float]) -> tuple[float, float, float]:
    v = (point[0], point[1], point[2], 1.0)
    return tuple(sum(t[i][k] * v[k] for k in range(4)) for i in range(3))  # type: ignore[return-value]


def head_relative(grip: Pose, head: Pose) -> list[list[float]] | None:
    """inverse(T_local_head) @ T_local_grip, only if both poses are fully valid."""
    g, h = grip.matrix(), head.matrix()
    if g is None or h is None:
        return None
    return matmul(invert_rigid(h), g)


# --------------------------------------------------------------------------
# Stream bookkeeping
# --------------------------------------------------------------------------


@dataclass
class StreamState:
    """Per-app-run (sid) sequencing and LOCAL-origin epochs. No pose reuse."""

    sid: str | None = None
    last_seq: int = 0
    change_times: list[int] = field(default_factory=list)  # LOCAL changeTime per epoch boundary
    events_lost: bool = False
    rejected: int = 0
    sessions: int = 0

    def accept(self, rec: PoseRecord | EventRecord | OverflowRecord | ProbeRecord) -> bool:
        if rec.sid != self.sid:
            # New app run: nothing carries over.
            self.sid, self.last_seq, self.change_times, self.events_lost = rec.sid, 0, [], False
            self.sessions += 1
        if rec.seq <= self.last_seq:
            self.rejected += 1
            return False
        self.last_seq = rec.seq
        if isinstance(rec, EventRecord):
            if rec.event == "local_space_change_pending":
                self.change_times.append(rec.time_ns)
            elif rec.event == "events_lost":
                self.events_lost = True
        return True

    def origin_epoch(self, rec: PoseRecord) -> int:
        """Which LOCAL origin a pose is expressed in: count of changes effective at t_query."""
        return sum(1 for t in self.change_times if t <= rec.t_query_ns)


# --------------------------------------------------------------------------
# Live monitor (reuses Step 2 filtered-adb acquisition)
# --------------------------------------------------------------------------


def _fmt_pose(p: Pose) -> str:
    if p.p is None:
        return f"{p.key}:{p.status}"
    return f"{p.key}:{p.status}({p.p[0]:+.3f},{p.p[1]:+.3f},{p.p[2]:+.3f})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Live QuestXarmPoseV1 monitor (diagnostic only)")
    ap.add_argument("--serial")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--output", type=Path, help="JSONL of decoded records (must not exist)")
    args = ap.parse_args(argv)
    if args.output is not None and args.output.exists():
        print(f"error: {args.output} exists", file=sys.stderr)
        return 2
    adb = find_adb()
    serial = pick_serial(adb, args.serial)
    state = StreamState()
    lock = threading.Lock()
    latest: dict[str, Any] = {"rec": None, "errors": 0, "n": 0}
    out = args.output.open("x", encoding="utf-8") if args.output else None

    def on_line(line: str, recv_ns: int) -> None:
        ll = parse_logcat_line(line, TAG)
        if ll is None:
            return
        with lock:
            try:
                rec = decode_record(ll.message)
            except RecordError as e:
                latest["errors"] += 1
                if out:
                    out.write(json.dumps({"host_monotonic_ns": recv_ns, "error": str(e)}) + "\n")
                return
            if state.accept(rec):
                latest["n"] += 1
                if isinstance(rec, PoseRecord):
                    latest["rec"] = rec
                elif isinstance(rec, OverflowRecord):
                    latest["rec"] = None  # dropped record: never keep an older pose current
                if out:
                    out.write(json.dumps({"host_monotonic_ns": recv_ns, "device_log_time": ll.device_log_time,
                                          "record": json.loads(ll.message)}) + "\n")  # fmt: skip

    tracker = FreshnessTracker()
    source = AdbLogcatSource(adb, serial, on_line, tracker, package=PACKAGE, tag=TAG)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    print(f"QuestXarmPoseV1 monitor on {redact(serial)} (DIAGNOSTIC ONLY)", flush=True)
    source.start()
    start = time.monotonic()
    try:
        while not stop.wait(0.5):
            with lock:
                r, n, err = latest["rec"], latest["n"], latest["errors"]
            if r is None:
                print(f"records={n} errors={err} source={tracker.transport_note}", flush=True)
            else:
                print(f"records={n} errors={err} state={r.session_state} focused={r.focused} "
                      f"epoch={state.origin_epoch(r)} {' '.join(_fmt_pose(r.poses[k]) for k in POSE_KEYS)} "
                      f"trigR={r.inputs['in_r']['trigger']} sqzR={r.inputs['in_r']['squeeze']} "
                      f"A={r.inputs['in_r']['a']} B={r.inputs['in_r']['b']}", flush=True)  # fmt: skip
            if args.duration and time.monotonic() - start >= args.duration:
                break
    finally:
        source.stop()
        if out:
            out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
