#!/usr/bin/env python3
"""Step 4B recorder for QuestXarmPoseV1 telemetry. DIAGNOSTIC ONLY.

Records EVERY new QuestXarmPoseV1 line from org.questxarm.reader (pid-filtered
adb logcat via the Step 2 AdbLogcatSource; buffered output from earlier
processes excluded), without downsampling, and refreshes a status line about
twice per second.

Each JSONL line keeps: host monotonic receive time, device log time, pid,
payload byte size, the ORIGINAL payload, and the decode result (or error).
Decoding uses quest_pose_v1 (never the legacy wE9ryARX parser).

Right-controller status (transport checked first, then input):
  DISCONNECTED    app not running / device unavailable (adb source down)
  STALE           no new line of any kind for > --stale-ms (stream freshness;
                  diagnostic watchdog only)
  OVERFLOW        latest pose was dropped at the source (overflow record):
                  current input invalidated, never refreshed by the overflow
  INPUT_STALE     lines arrive but no new decoded pose record for > --stale-ms
  UNFOCUSED       latest pose record not FOCUSED (incl. UNKNOWN), or a session_state event
                  other than FOCUSED was processed (invalidates input immediately; only a
                  later FOCUSED pose record can restore availability)
  MISSING         right grip pose missing / pose action inactive
  INVALID         right grip pose not VALID
  VALID_UNTRACKED right grip valid but not tracked
  GRIP_UNAVAILABLE right grip pose tracked but the right squeeze action is inactive (null)
  FRESH_TRACKED   fresh, focused, active, valid and tracked, grip action active
Pose/grip values are displayed only for FRESH_TRACKED; other statuses show flags only.
A finite pose alone never yields FRESH_TRACKED; the left controller is not
required. Inactive analog values are shown as '-' (null), never 0.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, TextIO

import quest_pose_v1 as qp
from quest_input_monitor import AdbLogcatSource, FreshnessTracker, find_adb, parse_logcat_line, pick_serial, redact

DEFAULT_STALE_MS = 500.0


def right_status(rec: qp.PoseRecord | None, source_up: bool, age_ms: float | None, stale_ms: float,
                 pose_age_ms: float | None = None, invalidated: str | None = None) -> str:
    """age_ms: since ANY new line (stream). pose_age_ms: since the last accepted pose record."""
    if not source_up:
        return "DISCONNECTED"
    if age_ms is None or age_ms > stale_ms:
        return "STALE"
    if invalidated == "overflow":
        return "OVERFLOW"
    if invalidated == "unfocused_event":
        return "UNFOCUSED"
    if rec is None:
        return "MISSING"
    if pose_age_ms is not None and pose_age_ms > stale_ms:
        return "INPUT_STALE"
    if not rec.focused or rec.session_state != "FOCUSED":
        return "UNFOCUSED"
    st = rec.poses["grip_r"].status
    out = {qp.MISSING: "MISSING", qp.INVALID: "INVALID", qp.VALID_UNTRACKED: "VALID_UNTRACKED",
           qp.TRACKED: "FRESH_TRACKED"}[st]  # fmt: skip
    if out == "FRESH_TRACKED" and rec.inputs["in_r"]["squeeze"] is None:
        return "GRIP_UNAVAILABLE"
    return out


def _v(x: Any, spec: str = ".2f") -> str:
    if x is None:
        return "-"
    if isinstance(x, bool):
        return "1" if x else "0"
    return format(x, spec)


class Recorder:
    def __init__(self, out: TextIO, stale_ms: float, expect_build: str | None = None):
        self.out = out
        self.expect_build = expect_build
        self.foreign = 0
        self.stale_ns = int(stale_ms * 1e6)
        self.lock = threading.Lock()
        self.state = qp.StreamState()
        self.latest: qp.PoseRecord | None = None
        self.last_recv_ns: int | None = None  # stream freshness: any new line
        self.last_pose_ns: int | None = None  # input freshness: accepted pose records only
        self.invalidated: str | None = None
        self.n_overflow = 0
        self.n_probe = 0
        self.probes: list[dict[str, Any]] = []
        self.n_lines = 0
        self.n_pose = 0
        self.n_event = 0
        self.errors = 0
        self.max_bytes = 0
        self.seq_gaps = 0
        self.dup_or_old = 0
        self.gaps_ns: list[int] = []
        self.events: list[dict[str, Any]] = []
        self.sids: list[str] = []
        self.builds: set[str] = set()
        self._printed = 0

    def on_line(self, line: str, recv_ns: int) -> None:
        ll = parse_logcat_line(line, qp.TAG)
        if ll is None:
            return
        size = len(ll.message.encode("utf-8"))
        rec_out: dict[str, Any] = {"host_ns": recv_ns, "device_log_time": ll.device_log_time, "pid": ll.pid,
                                   "bytes": size, "raw": ll.message}  # fmt: skip
        with self.lock:
            self.n_lines += 1
            self.max_bytes = max(self.max_bytes, size)
            if self.last_recv_ns is not None:
                self.gaps_ns.append(recv_ns - self.last_recv_ns)
            self.last_recv_ns = recv_ns
            try:
                rec = qp.decode_record(ll.message)
            except qp.RecordError as e:
                self.errors += 1
                rec_out["error"] = str(e)
                rec_out["suspect_truncation"] = size >= qp.MAX_RECORD_BYTES - 64 or not ll.message.rstrip().endswith("}")
                self.out.write(json.dumps(rec_out) + "\n")
                return
            if self.expect_build and rec.build != self.expect_build:
                self.foreign += 1  # another build's output (e.g. old buffered lines): never mixed in
                rec_out["foreign_build"] = rec.build
                self.out.write(json.dumps(rec_out) + "\n")
                return
            prev_sid, prev_seq = self.state.sid, self.state.last_seq
            accepted = self.state.accept(rec)
            if rec.sid != prev_sid:
                self.sids.append(rec.sid)
            elif accepted and rec.seq != prev_seq + 1:
                self.seq_gaps += rec.seq - prev_seq - 1
            if not accepted:
                self.dup_or_old += 1
            self.builds.add(rec.build)
            rec_out["ok"] = accepted
            rec_out["type"] = {qp.PoseRecord: "pose", qp.EventRecord: "event", qp.OverflowRecord: "overflow",
                               qp.ProbeRecord: "probe"}[type(rec)]  # fmt: skip
            if isinstance(rec, qp.PoseRecord):
                self.n_pose += 1
                rec_out["origin_epoch"] = self.state.origin_epoch(rec)
                if accepted:
                    self.latest = rec
                    self.last_pose_ns = recv_ns
                    self.invalidated = None
            elif isinstance(rec, qp.OverflowRecord):
                self.n_overflow += 1
                self.latest = None  # never keep an earlier pose marked ready
                self.invalidated = "overflow"  # last_pose_ns deliberately NOT refreshed
                rec_out["attempted_bytes"], rec_out["cap_bytes"] = rec.attempted_bytes, rec.cap_bytes
            elif isinstance(rec, qp.ProbeRecord):
                self.n_probe += 1  # NON-POSE: never touches latest/readiness
                exp = qp.expected_probe(rec.build, rec.sid, rec.seq, rec.probe_id, rec.target_bytes, rec.utf8)
                rec_out["probe_exact_match"] = ll.message == exp
                self.probes.append({"probe_id": rec.probe_id, "target_bytes": rec.target_bytes, "utf8": rec.utf8,
                                    "bytes": size, "exact_match": ll.message == exp})  # fmt: skip
            else:
                self.n_event += 1
                if rec.event == "session_state" and rec.session_state != "FOCUSED" and accepted:
                    # Explicit focus loss: invalidate when processed, not at the next refresh.
                    self.latest = None
                    self.invalidated = "unfocused_event"
                self.events.append({"host_ns": recv_ns, "seq": rec.seq, "event": rec.event,
                                    "session_state": rec.session_state, "time_ns": rec.time_ns,
                                    "local_epoch": rec.local_epoch})  # fmt: skip
            # Diagnostic status right after processing this line (stream assumed fresh/up here),
            # and the host processing delay from line receipt to that status update.
            rec_out["status_after"] = right_status(self.latest, True, 0.0, self.stale_ns / 1e6, 0.0, self.invalidated)
            rec_out["proc_ns"] = time.monotonic_ns() - recv_ns
            self.out.write(json.dumps(rec_out) + "\n")

    def status_line(self, now_ns: int, source_up: bool, note: str, stale_ms: float) -> str:
        with self.lock:
            rec = self.latest
            age = None if self.last_recv_ns is None else (now_ns - self.last_recv_ns) / 1e6
            page = None if self.last_pose_ns is None else (now_ns - self.last_pose_ns) / 1e6
            new = self.n_lines - self._printed
            self._printed = self.n_lines
            st = right_status(rec, source_up, age, stale_ms, page, self.invalidated)
            if rec is None or st != "FRESH_TRACKED":
                if rec is not None and st not in ("DISCONNECTED", "STALE", "INPUT_STALE", "OVERFLOW"):
                    g = rec.poses["grip_r"]
                    return (f"R={st:<15} new={new:<3} age={_v(age, '.0f')}ms {rec.session_state} grip_r={g.status} "
                            f"act={_v(g.active)} f={g.flags} [values hidden: not available] err={self.errors} gaps={self.seq_gaps}")  # fmt: skip
                return f"R={st:<15} new={new:<3} age={_v(age, '.0f')}ms err={self.errors} gaps={self.seq_gaps} [{note}]"
            r, g = rec.inputs["in_r"], rec.poses["grip_r"]
            pos = "-" if g.p is None else f"({g.p[0]:+.3f},{g.p[1]:+.3f},{g.p[2]:+.3f})"
            return (f"R={st:<15} new={new:<3} age={_v(age, '.0f')}ms {rec.session_state} "
                    f"grip_r={g.status} act={_v(g.active)} f={g.flags} pos={pos} "
                    f"trig={_v(r['trigger'])} sqz={_v(r['squeeze'])} A={_v(r['a'])} B={_v(r['b'])} "
                    f"grip_l={rec.poses['grip_l'].status} head={rec.poses['head'].status} "
                    f"epoch={self.state.origin_epoch(rec)} err={self.errors} gaps={self.seq_gaps}")  # fmt: skip


class TagLogcatSource:
    """Opt-in acquisition that attaches BEFORE the app starts (Step 4B logging repair).

    One `adb logcat -T <since> -s QuestXarmPoseV1:V` process (no --pid), so the app's first
    records are read live instead of depending on logd ring-buffer retention. Old buffered
    output is excluded by `since`; the recorder additionally drops records whose build id
    differs from --expect-build. A separate 1 s `pidof` poll reports app running/stopped
    (DISCONNECTED) exactly like AdbLogcatSource does. Only this object's own adb client
    processes are terminated; the shared adb server is never touched.
    """

    def __init__(self, adb: str, serial: str, on_line: Any, tracker: FreshnessTracker, since: str):
        self.adb, self.serial, self.on_line, self.tracker, self.since = adb, serial, on_line, tracker, since
        self.stop_event = threading.Event()
        self._proc: subprocess.Popen[bytes] | None = None
        self._threads = [threading.Thread(target=self._read_loop, daemon=True),
                         threading.Thread(target=self._poll_app, daemon=True)]  # fmt: skip

    def start(self) -> None:
        for th in self._threads:
            th.start()

    def _read_loop(self) -> None:
        cmd = [self.adb, "-s", self.serial, "logcat", "-T", self.since, "-v", "epoch", "-v", "usec",
               "-v", "threadtime", "-s", f"{qp.TAG}:V"]  # fmt: skip
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      stdin=subprocess.DEVNULL, start_new_session=True)  # fmt: skip
        assert self._proc.stdout is not None
        for raw in iter(self._proc.stdout.readline, b""):
            recv_ns = time.monotonic_ns()
            line = raw.decode("utf-8", errors="replace")
            ll = parse_logcat_line(line, qp.TAG)
            if ll is not None and float(ll.device_log_time) < float(self.since):
                continue
            self.on_line(line, recv_ns)
        if not self.stop_event.is_set():
            self.tracker.set_transport(False, "logcat exited (device lost?)", time.monotonic_ns())

    def _poll_app(self) -> None:
        while not self.stop_event.wait(1.0):
            try:
                r = subprocess.run([self.adb, "-s", self.serial, "shell", "pidof", qp.PACKAGE],
                                   capture_output=True, text=True, timeout=5)  # fmt: skip
                pid = r.stdout.strip()
                err = (r.stderr or "").strip().replace(self.serial, redact(self.serial))
                if r.returncode == 0 and pid:
                    self.tracker.set_transport(True, f"app pid {pid}", time.monotonic_ns())
                elif err:
                    self.tracker.set_transport(False, f"device unavailable: {err.splitlines()[-1][:60]}", time.monotonic_ns())
                else:
                    self.tracker.set_transport(False, "app not running", time.monotonic_ns())
            except subprocess.TimeoutExpired:
                self.tracker.set_transport(False, "adb pidof timed out", time.monotonic_ns())

    def stop(self) -> None:
        self.stop_event.set()
        p = self._proc
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=2)
        for th in self._threads:
            th.join(timeout=3)


def device_epoch(adb: str, serial: str) -> str | None:
    r = subprocess.run([adb, "-s", serial, "shell", "date", "+%s.%N"], capture_output=True, text=True, timeout=5)
    t = r.stdout.strip()
    return t[: t.index(".") + 4] if r.returncode == 0 and "." in t else None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Step 4B QuestXarmPoseV1 recorder (diagnostic only)")
    ap.add_argument("--serial")
    ap.add_argument("--output", type=Path, required=True, help="JSONL (must not exist)")
    ap.add_argument("--summary", type=Path, help="summary JSON (must not exist)")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--stale-ms", type=float, default=DEFAULT_STALE_MS, help="diagnostic watchdog, not a safety timeout")
    ap.add_argument("--pre-attach", action="store_true",
                    help="tag-only logcat attached before app start (captures startup/probe records live)")
    ap.add_argument("--expect-build", help="drop (and count) records whose build id differs")
    args = ap.parse_args(argv)
    for p in (args.output, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} exists", file=sys.stderr)
            return 2
    adb = find_adb()
    serial = pick_serial(adb, args.serial)
    since = device_epoch(adb, serial)  # excludes anything logged before this recorder started
    args.output.parent.mkdir(parents=True, exist_ok=True)
    out = args.output.open("x", encoding="utf-8")
    rec = Recorder(out, args.stale_ms, args.expect_build)
    tracker = FreshnessTracker(stale_ms=args.stale_ms)
    source: Any
    if args.pre_attach:
        if since is None:
            print("error: could not read device clock", file=sys.stderr)
            return 2
        source = TagLogcatSource(adb, serial, rec.on_line, tracker, since)
    else:
        source = AdbLogcatSource(adb, serial, rec.on_line, tracker, package=qp.PACKAGE, tag=qp.TAG,
                                 initial_since=since, resume_from_last_seen=True)  # fmt: skip
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    start = time.monotonic_ns()
    print(f"recording {qp.TAG} from {qp.PACKAGE} on {redact(serial)} since device epoch {since}; "
          f"start host_ns={start} (DIAGNOSTIC ONLY)", flush=True)  # fmt: skip
    source.start()
    try:
        while not stop.wait(0.5):
            now = time.monotonic_ns()
            print(f"{(now - start) / 1e9:7.1f}s {rec.status_line(now, tracker.transport_up, tracker.transport_note, args.stale_ms)}", flush=True)
            if args.duration and now - start >= args.duration * 1e9:
                break
    finally:
        end = time.monotonic_ns()
        source.stop()
        with rec.lock:
            out.close()
    el = (end - start) / 1e9
    summary = {
        "start_host_ns": start, "end_host_ns": end, "elapsed_s": round(el, 3),
        "lines": rec.n_lines, "pose_records": rec.n_pose, "event_records": rec.n_event,
        "rate_hz": round(rec.n_lines / el, 2) if el else None,
        "parse_errors": rec.errors, "seq_gaps": rec.seq_gaps, "dup_or_old": rec.dup_or_old,
        "overflow_records": rec.n_overflow, "probe_records": rec.n_probe, "probes": rec.probes,
        "foreign_build_records": rec.foreign, "acquisition": "pre-attach tag-only" if args.pre_attach else "pid-filtered",
        "max_record_bytes": rec.max_bytes, "sids": rec.sids, "builds": sorted(rec.builds),
        "host_interarrival": qp_stats(rec.gaps_ns), "events": rec.events,
        "transport_events": [{"t_s": round((t - start) / 1e9, 3), "event": e} for t, e in tracker.transport_events],
    }  # fmt: skip
    text = json.dumps(summary, indent=1)
    print(text)
    if args.summary:
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0


def qp_stats(gaps: list[int]) -> dict[str, Any]:
    from quest_transport import stats_ms

    return stats_ms(gaps)


if __name__ == "__main__":
    sys.exit(main())
