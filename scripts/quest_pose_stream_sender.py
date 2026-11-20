#!/usr/bin/env python3
"""Local sender for the NEW OpenXR reader (protocol v2). ROBOT NOT CONNECTED.

Reads QuestXarmPoseV1 telemetry from org.questxarm.reader with ONE owned `adb logcat` child
(tag-only, attached before the app starts), processes EVERY received source record through the
continuity tracker, and sends at most --max-rate pose packets per second to 127.0.0.1:<port>
(the local end of the dedicated SSH tunnel to the lab's diagnostic receiver).

  * Acquisition is independent of network writes; a latest-sample slot holds one pose.
  * Validation, events, source-sequence continuity and input loss are processed for every
    record BEFORE the 30 Hz decimation. A loss between transmitted poses raises the persistent
    continuity generation carried in every later packet.
  * An independent watchdog in the send loop detects missing input even if the reader thread
    is blocked (e.g. the logcat child is suspended).
  * Acquisition generations: SIGUSR1 kills/reaps the owned logcat child and discards its
    generation (it is never resumed); SIGUSR2 starts a fresh generation from "now". Records
    from an older generation are never accepted.
  * Bounded buffers, explicit timeouts, no automatic reconnect, nothing replayed.

Raw input availability and continuity are diagnostics, not robot readiness. There is no
motion-enable latch and no option that enables robot control.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, TextIO

import quest_pose_transport as pt
import quest_pose_v1 as qp
from quest_input_monitor import MonitorError, find_adb, parse_logcat_line, pick_serial, redact
from quest_stream_sender import (
    DEFAULT_LOCAL_PORT,
    ACK_TIMEOUT_S,
    DEFAULT_MAX_RATE_HZ,
    MAX_OUTSTANDING_ACKS,
    SenderSession,
    SessionFailed,
    connect,
)
from quest_transport import DIAGNOSTIC_BANNER, LOOPBACK, ProtocolError, new_id, stats_ms
from quest_web_source import WEB_BUILD, WebAcquisition

DEFAULT_BUILD = "s4a-ff0da2692500"
DEFAULT_STALE_MS = 500.0  # diagnostic watchdog, NOT a robot safety timeout
MAX_PENDING_SOURCE_EVENTS = 64
CHILD_REAP_TIMEOUT_S = 3.0


# --------------------------------------------------------------------------
# Latest-sample slot + per-record processing
# --------------------------------------------------------------------------


class PoseSlot:
    """Every source line passes through here (acquisition thread); the sender thread takes
    at most one newest pose per send opportunity. Holds ONE pose regardless of network speed."""

    last_source_seq = 0  # for SenderSession.__init__ (unused by the v2 path)

    def __init__(self, expect_build: str | None, stale_ms: float = DEFAULT_STALE_MS, source_log: TextIO | None = None):
        self.lock = threading.Lock()
        self.tracker = pt.ContinuityTracker(expect_build=expect_build, stale_ns=int(stale_ms * 1e6))
        self.source_log = source_log
        self._latest: dict[str, Any] | None = None
        self._latest_sent = True
        self._events: deque = deque()
        self.events_dropped = 0
        self.decimated_since_last = 0
        self.decimated_total = 0
        self.stale_generation_lines = 0
        self.lines = 0
        self.max_bytes = 0

    def on_line(self, line: str, recv_ns: int, gen: int) -> None:
        ll = parse_logcat_line(line, qp.TAG)
        if ll is None:
            return
        self.on_message(ll.message, recv_ns, gen, ll.device_log_time)

    def on_message(self, message: str, recv_ns: int, gen: int, device_log_time: Any) -> None:
        """One source record (the app's log line without its logcat prefix, or a message of the headset web page)."""
        with self.lock:
            tr = self.tracker
            if gen != tr.acq_gen:
                self.stale_generation_lines += 1  # an aborted generation's output is never accepted
                return
            self.lines += 1
            self.max_bytes = max(self.max_bytes, len(message.encode("utf-8")))
            kind = "parse_error"
            try:
                rec = qp.decode_record(message)
            except qp.RecordError as e:
                tr.on_parse_error(str(e), recv_ns)
                self._latest, self._latest_sent = None, True  # never leave the last good pose active
                rec = None
            if rec is not None:
                kind = tr.on_record(rec, recv_ns)
                if kind == "pose":
                    if self._latest is not None and not self._latest_sent:
                        self.decimated_since_last += 1  # intentional: replaced before a send opportunity
                        self.decimated_total += 1
                    self._latest = {"rec": rec, "raw": json.loads(message), "recv_ns": recv_ns,
                                    "device_log_time": device_log_time, "acq_gen": tr.acq_gen,
                                    "session_gen": tr.session_gen}  # fmt: skip
                    self._latest_sent = False
                elif kind == "event":
                    if len(self._events) >= MAX_PENDING_SOURCE_EVENTS:
                        self._events.popleft()
                        self.events_dropped += 1
                        tr._break("event_queue_overflow", recv_ns)
                    self._events.append({"raw": json.loads(message), "recv_ns": recv_ns, "acq_gen": tr.acq_gen})
                elif kind == "overflow":
                    self._latest, self._latest_sent = None, True
            if self.source_log is not None:
                self.source_log.write(json.dumps({"host_ns": recv_ns, "gen": gen, "kind": kind, "cont_gen": tr.continuity_gen,
                                                  "available": tr.available, "raw": message}) + "\n")  # fmt: skip

    def new_generation(self, now_ns: int, note: str) -> int:
        with self.lock:
            self._latest, self._latest_sent = None, True  # no cached pose survives a restart
            self._events.clear()
            return self.tracker.new_generation(now_ns, note)

    def set_source(self, up: bool, note: str, now_ns: int) -> None:
        with self.lock:
            self.tracker.set_source(up, note, now_ns)
            if not up:
                self._latest, self._latest_sent = None, True

    def watchdog(self, now_ns: int) -> bool:
        with self.lock:
            return self.tracker.watchdog(now_ns)

    def take(self) -> tuple[list[dict[str, Any]], dict[str, Any] | None, int]:
        with self.lock:
            events = list(self._events)
            self._events.clear()
            sample = None
            dec = 0
            if self._latest is not None and not self._latest_sent:
                sample, self._latest_sent = self._latest, True
                dec, self.decimated_since_last = self.decimated_since_last, 0
            return events, sample, dec

    def continuity_fields(self) -> dict[str, Any]:
        with self.lock:
            return self.tracker.continuity_fields()

    def status(self, now_ns: int) -> dict[str, Any]:
        with self.lock:
            tr = self.tracker
            last = tr.last_record_recv_ns
            return {"acq_gen": tr.acq_gen, "source_up": tr.source_up, "source_note": tr.source_note[:120] or "-",
                    "stale": tr.stale, "sid": tr.sid, "session_gen": tr.session_gen, "last_source_seq": tr.last_seq,
                    "available": tr.available, "pending_breaks": len(tr.pending),
                    "source_age_ms": None if last is None else round((now_ns - last) / 1e6, 1)}  # fmt: skip


# --------------------------------------------------------------------------
# Owned adb logcat acquisition with generations
# --------------------------------------------------------------------------


def device_epoch(adb: str, serial: str) -> str | None:
    r = subprocess.run([adb, "-s", serial, "shell", "date", "+%s.%N"], capture_output=True, text=True, timeout=5)
    t = r.stdout.strip()
    return t[: t.index(".") + 4] if r.returncode == 0 and "." in t else None


class OwnedLogcatAcquisition:
    """One `adb logcat -T <since> -s QuestXarmPoseV1:V` child per generation, owned by this
    object (its Popen/PID is the only process ever signalled). Never touches the adb server."""

    def __init__(self, adb: str, serial: str, slot: PoseSlot, on_event: Callable[[dict[str, Any]], None]):
        self.adb, self.serial, self.slot, self.on_event = adb, serial, slot, on_event
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._aborting = False
        self._poll = threading.Thread(target=self._poll_app, name="app-poll", daemon=True)
        self.gen = 0
        self.since: str | None = None

    @property
    def child_pid(self) -> int | None:
        p = self._proc
        return None if p is None or p.poll() is not None else p.pid

    def start_poll(self) -> None:
        self._poll.start()

    def start_generation(self, note: str) -> int:
        if self._proc is not None and self._proc.poll() is None:
            raise RuntimeError("a generation is already running; abort it first")
        since = device_epoch(self.adb, self.serial)
        if since is None:
            raise RuntimeError("could not read the device clock")
        now = time.monotonic_ns()
        self.gen = self.slot.new_generation(now, note)
        self.since, self._aborting = since, False
        cmd = [self.adb, "-s", self.serial, "logcat", "-T", since, "-v", "epoch", "-v", "usec", "-v", "threadtime",
               "-s", f"{qp.TAG}:V"]  # fmt: skip
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                                      start_new_session=True)  # fmt: skip
        self._reader = threading.Thread(target=self._read, args=(self._proc, self.gen, float(since)), daemon=True)
        self._reader.start()
        self.on_event({"t": now, "event": "acquisition_generation_started", "gen": self.gen, "child_pid": self._proc.pid,
                       "since_device_epoch": since, "note": note})  # fmt: skip
        return self.gen

    def _read(self, proc: subprocess.Popen, gen: int, since: float) -> None:
        assert proc.stdout is not None
        try:
            for raw in iter(proc.stdout.readline, b""):
                recv_ns = time.monotonic_ns()
                line = raw.decode("utf-8", errors="replace")
                ll = parse_logcat_line(line, qp.TAG)
                if ll is not None and float(ll.device_log_time) < since:
                    continue  # logged before this generation started
                self.slot.on_line(line, recv_ns, gen)
        except (OSError, ValueError):
            pass  # pipe closed during abort
        if not self._aborting and not self._stop.is_set() and gen == self.gen:
            self.slot.set_source(False, "logcat exited (device lost?)", time.monotonic_ns())
            self.on_event({"t": time.monotonic_ns(), "event": "logcat_exited_unexpectedly", "gen": gen})

    def abort_generation(self, reason: str) -> dict[str, Any]:
        """Kill and reap the owned child (works on a SIGSTOPped child), close its pipe, discard
        the generation. Bounded; never resumes the child, so its buffered output is never read."""
        proc, reader = self._proc, self._reader
        info: dict[str, Any] = {"gen": self.gen, "reason": reason, "child_pid": None if proc is None else proc.pid}
        self._aborting = True
        now = time.monotonic_ns()
        self.slot.set_source(False, f"acquisition aborted: {reason}", now)
        self.slot.new_generation(now, f"aborted: {reason}")  # later lines from the old child carry a stale gen
        if proc is not None:
            if proc.poll() is None:
                proc.kill()  # SIGKILL: effective even while stopped
            try:
                proc.wait(timeout=CHILD_REAP_TIMEOUT_S)
                info["reaped"], info["returncode"] = True, proc.returncode
            except subprocess.TimeoutExpired:
                info["reaped"] = False
            try:
                if proc.stdout is not None:
                    proc.stdout.close()
            except OSError:
                pass
        if reader is not None:
            reader.join(timeout=2.0)
            info["reader_joined"] = not reader.is_alive()
        self._proc = self._reader = None
        self.on_event(dict(info, t=time.monotonic_ns(), event="acquisition_generation_aborted"))
        return info

    def _poll_app(self) -> None:
        while not self._stop.wait(1.0):
            if self._proc is None:
                continue  # no acquisition: source stays down ("acquisition aborted")
            try:
                r = subprocess.run([self.adb, "-s", self.serial, "shell", "pidof", qp.PACKAGE],
                                   capture_output=True, text=True, timeout=5)  # fmt: skip
                pid = r.stdout.strip()
                err = (r.stderr or "").strip().replace(self.serial, redact(self.serial))
                now = time.monotonic_ns()
                if self._proc is None:
                    continue
                if r.returncode == 0 and pid:
                    self.slot.set_source(True, f"app pid {pid}", now)
                elif err:
                    self.slot.set_source(False, f"device unavailable: {err.splitlines()[-1][:60]}", now)
                else:
                    self.slot.set_source(False, "app not running", now)
            except subprocess.TimeoutExpired:
                self.slot.set_source(False, "adb pidof timed out", time.monotonic_ns())

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._proc is not None:
                self.abort_generation("sender stopping")
        finally:
            p = self._proc
            if p is not None and p.poll() is None:  # last resort: never leave a (possibly stopped) child behind
                p.kill()
                p.wait(timeout=CHILD_REAP_TIMEOUT_S)
        if self._poll.is_alive():
            self._poll.join(timeout=7.0)


# --------------------------------------------------------------------------
# Network session (reuses the tested v1 SenderSession socket/ack handling)
# --------------------------------------------------------------------------


class PoseSenderSession(SenderSession):
    proto_version = pt.PROTOCOL_VERSION
    _decode = staticmethod(pt.decode)

    def __init__(self, sock: Any, slot: PoseSlot, sender_run: str, **kw: Any):
        super().__init__(sock, slot, sender_run, **kw)  # type: ignore[arg-type]
        self.pslot = slot
        self.sender_run = sender_run
        self.counts = {"pose_sent": 0, "pose_available_sent": 0, "pose_unavailable_sent": 0, "source_events_sent": 0,
                       "status_sent": 0, "acks_ok": 0, "acks_rejected": 0}  # fmt: skip

    def _hello_packet(self) -> dict[str, Any]:
        return {"v": self.proto_version, "type": "hello", "session": self.session, "sender_run": self.sender_run,
                "protocol": pt.PROTOCOL, "source_format": pt.SOURCE_FORMAT, "source_schema": pt.SOURCE_SCHEMA,
                "frame": pt.FRAME, "role": "quest_pose_stream_sender"}  # fmt: skip

    def tick(self) -> None:
        if self.failure:
            raise SessionFailed(self.failure)
        now = self.clock()
        self.pslot.watchdog(now)  # independent of the reader thread
        events, sample, decimated = self.pslot.take()
        for ev in events:
            self._send({"type": "source_event", "acq_gen": ev["acq_gen"], "event": ev["raw"],
                        "source_recv_mono_ns": ev["recv_ns"]})  # fmt: skip
            self.counts["source_events_sent"] += 1
        st = self.pslot.status(now)
        if sample is not None:
            rec = sample["rec"]
            pkt = pt.pose_packet(self.sender_run, sample["acq_gen"], sample["session_gen"], rec, sample["raw"],
                                 sample["recv_ns"], sample["device_log_time"], self.pslot.continuity_fields(),
                                 decimated, st["source_up"], st["stale"])  # fmt: skip
            sent_ns = self._send(pkt)
            self.counts["pose_sent"] += 1
            if pkt["raw_right"]["available"]:
                self.counts["pose_available_sent"] += 1
                self.acq_to_send_ns.append(sent_ns - sample["recv_ns"])  # same host, same clock
            else:
                self.counts["pose_unavailable_sent"] += 1
        elif st["pending_breaks"] or self.last_send_ns is None or now - self.last_send_ns >= self.heartbeat_ns:
            # status/heartbeat: carries source status and continuity; never counts as input
            self._send({"type": "status", "acq_gen": st["acq_gen"], "source_up": st["source_up"],
                        "source_note": st["source_note"], "stale": st["stale"], "sid": st["sid"],
                        "last_source_seq": st["last_source_seq"], "source_age_ms": st["source_age_ms"],
                        "continuity": self.pslot.continuity_fields()})  # fmt: skip
            self.counts["status_sent"] += 1
        with self._out_lock:
            oldest = min(self._outstanding.values(), default=None)
        if oldest is not None and self.clock() - oldest > self.ack_timeout_ns:
            raise SessionFailed("ack timeout")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serial")
    ap.add_argument("--port", type=int, default=DEFAULT_LOCAL_PORT, help="local tunnel port on 127.0.0.1")
    ap.add_argument("--max-rate", type=float, default=DEFAULT_MAX_RATE_HZ,
                    help="max pose packets/s (transport setting, not a robot servo rate)")  # fmt: skip
    ap.add_argument("--stale-ms", type=float, default=DEFAULT_STALE_MS, help="diagnostic watchdog, not a safety timeout")
    ap.add_argument("--expect-build", help=f"default: {DEFAULT_BUILD} (the app), {WEB_BUILD} (the web page)")
    ap.add_argument("--source", choices=("app", "web"), default="app",
                    help="app: the headset app's log over adb; web: the headset's VR web page (quest_web_source.py)")  # fmt: skip
    ap.add_argument("--web-port", type=int, default=8791, help="web: the page's port on 127.0.0.1 (adb reverse to the headset)")
    ap.add_argument("--web-cams", default="http://127.0.0.1:18091", help="web: the camera relay's tunnel on this PC")
    ap.add_argument("--web-views", default="", help="web: comma list of the relay's views to show in the headset")
    ap.add_argument("--web-big", default="", help="web: the views shown large, on top")
    ap.add_argument("--web-status", type=Path, help="web: a text file whose content is shown in the headset (the session's state)")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--log", type=Path, help="JSONL packet/event log (must not exist)")
    ap.add_argument("--source-log", type=Path, help="JSONL of EVERY received source record (must not exist)")
    ap.add_argument("--summary", type=Path, help="summary JSON (must not exist)")
    ap.add_argument("--acq-file", type=Path, help="JSON status of the owned acquisition child (rewritten on change)")
    ap.add_argument("--ack-timeout", type=float, default=ACK_TIMEOUT_S,
                    help="give up when the receiver has not answered for this long, s (the xArm keeps the default)")  # fmt: skip
    args = ap.parse_args(argv)
    for p in (args.log, args.source_log, args.summary):
        if p is not None and p.exists():
            print(f"error: {p} already exists; refusing to overwrite", file=sys.stderr)
            return 2
    web = args.source == "web"
    if args.expect_build is None:
        args.expect_build = WEB_BUILD if web else DEFAULT_BUILD
    adb, serial = "", "web page"
    if not web:
        try:
            adb = find_adb()
            serial = pick_serial(adb, args.serial)
        except (MonitorError, OSError) as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
    try:
        sock = connect(args.port)
    except OSError as e:
        print(f"error: cannot connect to {LOOPBACK}:{args.port} (is the tunnel up?): {e}", file=sys.stderr)
        return 3
    log = args.log.open("x", encoding="utf-8") if args.log else None
    source_log = args.source_log.open("x", encoding="utf-8", buffering=1) if args.source_log else None
    sender_run = new_id()
    slot = PoseSlot(args.expect_build, args.stale_ms, source_log)
    session = PoseSenderSession(sock, slot, sender_run, max_rate_hz=args.max_rate, ack_timeout_s=args.ack_timeout,
                                # room for every packet of a stall that long (30 Hz + heartbeats < 64/s); the default stays 256
                                max_outstanding=max(MAX_OUTSTANDING_ACKS, int(args.ack_timeout * 64)), log=log)
    try:
        ack = session.handshake()
        if ack.get("protocol") != pt.PROTOCOL:
            raise SessionFailed("receiver does not speak " + pt.PROTOCOL)
    except (SessionFailed, ProtocolError) as e:
        print(f"error: handshake failed: {e}", file=sys.stderr)
        session.close(say_bye=False)
        return 3
    print(f"{DIAGNOSTIC_BANNER}. Handshake OK ({pt.PROTOCOL} v{pt.PROTOCOL_VERSION}): session {session.session}, "
          f"sender_run {sender_run}, receiver={ack.get('receiver')}, robot_connected={ack.get('robot_connected')}. "
          f"Device {redact(serial)}, build {args.expect_build}, max {args.max_rate:g} Hz. pid {os.getpid()}", flush=True)  # fmt: skip

    log_lock = threading.Lock()
    acq_events: list[dict[str, Any]] = []

    def on_acq_event(ev: dict[str, Any]) -> None:
        acq_events.append(ev)
        if log is not None:
            with log_lock:
                log.write(json.dumps(ev) + "\n")
        if args.acq_file is not None:
            args.acq_file.write_text(json.dumps({"sender_pid": os.getpid(), "sender_run": sender_run, **ev}) + "\n")
        print(f"ACQ {ev.get('event')} gen={ev.get('gen')} child_pid={ev.get('child_pid')}", flush=True)

    if web:
        if not args.web_cams.startswith("http://127.0.0.1:"):
            print("error: --web-cams must be on 127.0.0.1", file=sys.stderr)
            return 2
        try:
            acq: Any = WebAcquisition(slot, on_acq_event, args.web_port, args.web_cams, [v for v in args.web_views.split(",") if v],
                                      [v for v in args.web_big.split(",") if v], args.web_status)  # fmt: skip
        except OSError as e:
            print(f"error: cannot serve the headset page on 127.0.0.1:{args.web_port}: {e}", file=sys.stderr)
            return 3
    else:
        acq = OwnedLogcatAcquisition(adb, serial, slot, on_acq_event)
    control = {"abort": False, "start": False}
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGUSR1, lambda *_: control.__setitem__("abort", True))
    signal.signal(signal.SIGUSR2, lambda *_: control.__setitem__("start", True))
    start = time.monotonic_ns()
    last_print = [start]
    result = "failed: setup"
    try:
        acq.start_generation("initial (attached before app launch)")
        acq.start_poll()
        session.start_reader()

        def on_tick() -> None:
            if control["abort"]:
                control["abort"] = False
                acq.abort_generation("requested (SIGUSR1)")
            if control["start"]:
                control["start"] = False
                try:
                    acq.start_generation("explicit restart (SIGUSR2)")
                except RuntimeError as e:
                    print(f"ACQ start refused: {e}", flush=True)
            now = time.monotonic_ns()
            if now - last_print[0] >= 500_000_000:
                last_print[0] = now
                st = slot.status(now)
                tr = slot.tracker
                c = session.counts
                rtt = session.rtt_ns[-1] / 1e6 if session.rtt_ns else None
                inp = "DOWN" if not st["source_up"] else ("STALE" if st["stale"] else ("AVAILABLE" if st["available"] else "UNAVAILABLE"))
                print(f"{(now - start) / 1e9:7.1f}s src={inp} acq={st['acq_gen']} sid={str(st['sid'])[:8]} "
                      f"cont_gen={tr.continuity_gen} loss={tr.loss_count} gaps={tr.source_gap_records} "
                      f"recs={tr.counts['records']} poses={tr.counts['pose']} avail={tr.counts['pose_available']} "
                      f"sent={c['pose_sent']} dec={slot.decimated_total} ev={c['source_events_sent']} st={c['status_sent']} "
                      f"rtt={'-' if rtt is None else f'{rtt:.0f}'}ms [{st['source_note']}] | {DIAGNOSTIC_BANNER}", flush=True)  # fmt: skip

        def should_stop() -> bool:
            return stop.is_set() or (args.duration is not None and time.monotonic_ns() - start >= args.duration * 1e9)

        result = session.run(should_stop, on_tick)
    finally:
        end = time.monotonic_ns()
        try:
            session.close(say_bye=result == "stopped")
        finally:
            acq.stop()  # kills/reaps the owned child even if it is stopped
            for f in (log, source_log):
                if f is not None:
                    f.close()
    elapsed = (end - start) / 1e9
    tr = slot.tracker
    summary = {
        "role": "sender", "protocol": pt.PROTOCOL, "protocol_version": pt.PROTOCOL_VERSION, "banner": DIAGNOSTIC_BANNER,
        "robot_connection_attempted": False, "session": session.session, "sender_run": sender_run,
        "device": redact(serial), "expect_build": args.expect_build, "result": result, "elapsed_s": round(elapsed, 3),
        "max_rate_hz": args.max_rate, "stale_ms": args.stale_ms,
        "source": dict(tr.counts, lines=slot.lines, max_record_bytes=slot.max_bytes,
                       stale_generation_lines_discarded=slot.stale_generation_lines,
                       source_gap_records=tr.source_gap_records, unavailable_reasons=tr.unavailable_reasons,
                       source_events_dropped=slot.events_dropped),
        "sent": dict(session.counts, intentionally_decimated_poses=slot.decimated_total,
                     pose_rate_hz=round(session.counts["pose_sent"] / elapsed, 2) if elapsed else None),
        "continuity": {"gen": tr.continuity_gen, "loss_count": tr.loss_count, "break_counts": tr.break_counts},
        "stale_detections": tr.stale_detections,
        "acquisition_events": acq_events,
        "ack_reject_reasons": session.reject_reasons,
        "unacked_at_close_discarded": getattr(session, "unacked_at_close", None),
        "rtt_sender_clock": stats_ms(session.rtt_ns),
        "acquisition_to_send_age_sender_clock": stats_ms(session.acq_to_send_ns),
        "notes": ["RTT is application round-trip on the sender's clock; RTT/2 is NOT one-way latency.",
                  "recv time is when this host read the log line, not a Quest sensor-capture time.",
                  "Arrival rate says nothing about absolute sample age."],
    }  # fmt: skip
    text = json.dumps(summary, indent=1)
    print(text)
    if args.summary is not None:
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0 if result == "stopped" else 4


if __name__ == "__main__":
    sys.exit(main())
