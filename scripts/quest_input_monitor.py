#!/usr/bin/env python3
"""Local monitor for Quest controller poses/buttons streamed via adb logcat.

Reads only the teleop app's controller log lines (tag ``wE9ryARX``, filtered
by the app's pid), validates each message, and prints a compact freshness
status about twice per second. Standard library only; uses the existing
``adb`` executable. It does not open network listeners, forward ports, or
touch the shared adb server (the server is never killed).

Protocol (verified against upstream oculus_reader.py / buttons_parser.py at
QuestArmTeleop ros2 145b80360cf2 and live output from the headset):

    <transforms>&<buttons>
    transforms := entry ('|' entry)*   entry := ('r'|'l') ':' 16 floats
    buttons    := comma list of markers 'R'/'L', pressed boolean keys, and
                  analog items '<key> <float> [<float> ...]'

Each matrix is 16 space-separated floats in row-major order (upstream fills
transform[r][c] with c advancing first). Matrices are kept exactly in the
source representation: no coordinate conversion, Euler angles or robot units.

Timestamps:
  * host_monotonic_ns -- time.monotonic_ns() taken when the line was read on
    this computer. All freshness/age/rate metrics use only this clock.
  * device_log_time -- logcat's epoch time (device clock) at which the app
    wrote the log line. It is NOT a sensor capture timestamp and is never
    subtracted from a host clock reading.

The protocol carries no tracking-valid flag, so tracking validity is always
reported as unknown (null). Fresh messages only prove the app is logging.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, TextIO

PACKAGE = "com.rail.oculus.teleop"
ACTIVITY = "com.rail.oculus.teleop.MainActivity"
TAG = "wE9ryARX"

CONTROLLER_KEYS = ("r", "l")
RIGHT_BOOL_KEYS = ("A", "B", "RThU", "RJ", "RG", "RTr")
LEFT_BOOL_KEYS = ("X", "Y", "LThU", "LJ", "LG", "LTr")

# Homogeneous-transform consistency tolerances. The app prints floats with
# ~6 significant digits, so rounding alone produces errors around 1e-6..1e-5.
BOTTOM_ROW_TOL = 1e-4  # max |bottom_row - [0, 0, 0, 1]|
ORTHONORMAL_TOL = 1e-3  # max |R^T R - I| element
DET_TOL = 1e-3  # |det(R) - 1|

DEFAULT_STALE_MS = 500.0  # diagnostic only; NOT a robot safety timeout
DEFAULT_PRINT_INTERVAL_S = 0.5
RATE_WINDOW_S = 2.0
ADB_CMD_TIMEOUT_S = 5.0
APP_POLL_INTERVAL_S = 1.0

TRACKING_VALID_NOTE = "protocol has no tracking-valid flag; validity unknown"

# logcat -v epoch -v usec -v threadtime:
#   "  1790800025.520487 19405 19437 I wE9ryARX: <payload>"
_LOGCAT_RE = re.compile(
    r"^\s*(?P<ts>\d+\.\d+)\s+(?P<pid>\d+)\s+(?P<tid>\d+)\s+"
    r"(?P<level>[VDIWEFAS])\s+(?P<tag>[^:]*?)\s*: ?(?P<msg>.*)$"
)


class MessageError(ValueError):
    """A tagged log message that does not match the expected protocol."""


class MonitorError(RuntimeError):
    """Setup problem (adb missing, device ambiguous, output exists, ...)."""


# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LogLine:
    device_log_time: str  # verbatim epoch seconds string from logcat
    pid: int
    tag: str
    message: str


def parse_logcat_line(line: str, tag: str = TAG) -> LogLine | None:
    """Return the parsed line if it carries ``tag``; None for anything else."""
    m = _LOGCAT_RE.match(line.rstrip("\r\n"))
    if m is None or m.group("tag").strip() != tag:
        return None
    return LogLine(m.group("ts"), int(m.group("pid")), tag, m.group("msg"))


@dataclass
class PoseCheck:
    homogeneous_ok: bool
    bottom_row_err: float
    orthonormal_err: float
    det: float


@dataclass
class ControllerMessage:
    poses: dict[str, list[list[float]] | None]
    pose_checks: dict[str, PoseCheck | None]
    buttons: dict[str, bool | tuple[float, ...]]
    unknown_button_tokens: list[str] = field(default_factory=list)

    @property
    def pose_present(self) -> dict[str, bool]:
        return {k: self.poses[k] is not None for k in CONTROLLER_KEYS}

    def translation(self, key: str) -> tuple[float, float, float] | None:
        m = self.poses.get(key)
        if m is None:
            return None
        return (m[0][3], m[1][3], m[2][3])


def _parse_float(text: str, where: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise MessageError(f"{where}: not a number: {text[:32]!r}") from None
    if not math.isfinite(value):
        raise MessageError(f"{where}: non-finite value {text!r}")
    return value


def check_pose(m: list[list[float]]) -> PoseCheck:
    """Homogeneous-transform consistency, within the documented tolerances."""
    bottom = max(abs(a - b) for a, b in zip(m[3], (0.0, 0.0, 0.0, 1.0)))
    r = [row[:3] for row in m[:3]]
    ortho = 0.0
    for i in range(3):
        for j in range(3):
            dot = sum(r[k][i] * r[k][j] for k in range(3))
            ortho = max(ortho, abs(dot - (1.0 if i == j else 0.0)))
    det = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )
    ok = bottom <= BOTTOM_ROW_TOL and ortho <= ORTHONORMAL_TOL and abs(det - 1.0) <= DET_TOL
    return PoseCheck(ok, bottom, ortho, det)


def parse_matrix(text: str, key: str) -> list[list[float]]:
    values = text.split()
    if len(values) != 16:
        raise MessageError(f"pose {key!r}: expected 16 values, got {len(values)}")
    flat = [_parse_float(v, f"pose {key!r}[{i}]") for i, v in enumerate(values)]
    return [flat[r * 4 : r * 4 + 4] for r in range(4)]


def parse_buttons(text: str) -> tuple[dict[str, bool | tuple[float, ...]], list[str]]:
    """Stdlib port of upstream buttons_parser.parse_buttons, but strict.

    Boolean keys for a hand exist only when its marker ('R'/'L') is present;
    analog values are float tuples. Non-numeric/non-finite analog values raise
    MessageError; unrecognised bare tokens are returned rather than dropped.
    """
    tokens = [t for t in text.strip().split(",") if t != ""]
    buttons: dict[str, bool | tuple[float, ...]] = {}
    if "R" in tokens:
        tokens.remove("R")
        buttons.update({k: False for k in RIGHT_BOOL_KEYS})
    if "L" in tokens:
        tokens.remove("L")
        buttons.update({k: False for k in LEFT_BOOL_KEYS})
    unknown: list[str] = []
    for tok in tokens:
        parts = tok.split(" ")
        if len(parts) == 1:
            if tok in buttons and buttons[tok] is False:
                buttons[tok] = True
            else:
                unknown.append(tok)
            continue
        key = parts[0]
        vals = tuple(_parse_float(v, f"button {key!r}") for v in parts[1:] if v != "")
        if not vals:
            raise MessageError(f"button {key!r}: no values")
        buttons[key] = vals
    return buttons, unknown


def parse_message(payload: str) -> ControllerMessage:
    parts = payload.split("&")
    if len(parts) != 2:
        raise MessageError(f"expected exactly one '&', got {len(parts) - 1}")
    transforms_text, buttons_text = parts
    poses: dict[str, list[list[float]] | None] = {k: None for k in CONTROLLER_KEYS}
    checks: dict[str, PoseCheck | None] = {k: None for k in CONTROLLER_KEYS}
    for entry in transforms_text.split("|"):
        entry = entry.strip()
        if not entry:
            continue
        key, sep, values = entry.partition(":")
        key = key.strip()
        if not sep or key not in CONTROLLER_KEYS:
            raise MessageError(f"bad pose entry key {key[:8]!r}")
        if poses[key] is not None:
            raise MessageError(f"duplicate pose {key!r}")
        matrix = parse_matrix(values, key)
        poses[key] = matrix
        checks[key] = check_pose(matrix)
    buttons, unknown = parse_buttons(buttons_text)
    return ControllerMessage(poses, checks, buttons, unknown)


# --------------------------------------------------------------------------
# Freshness tracking
# --------------------------------------------------------------------------


class Status:
    LIVE = "LIVE"
    STALE = "STALE"
    DISCONNECTED = "DISCONNECTED"


@dataclass
class Snapshot:
    status: str
    new_messages: int
    total_messages: int
    parse_errors: int
    rate_hz: float
    age_ms: float | None
    last: ControllerMessage | None
    transport_note: str


class FreshnessTracker:
    """Counts and receive times; mutated only by record_message/record_error.

    ``snapshot`` is read-only with respect to freshness: polling or printing
    never refreshes the receive time. Staleness is computed from the caller's
    current monotonic time, so it is detected even while the reader thread
    is blocked waiting for input.
    """

    def __init__(self, stale_ms: float = DEFAULT_STALE_MS, rate_window_s: float = RATE_WINDOW_S):
        self.stale_ns = int(stale_ms * 1e6)
        self.rate_window_ns = int(rate_window_s * 1e9)
        self._lock = threading.Lock()
        self._recent: deque[int] = deque()
        self.total = 0
        self.errors = 0
        self.homogeneous_warnings = 0
        self.last_recv_ns: int | None = None
        self.last_msg: ControllerMessage | None = None
        self.gaps_ns: list[int] = []
        self._printed_total = 0
        self.transport_up = False
        self.transport_note = "starting"
        self.transport_events: list[tuple[int, str]] = []
        # Optional hook (up, note, now_ns), called outside the lock on each change.
        self.on_transport: Callable[[bool, str, int], None] | None = None

    def record_message(self, msg: ControllerMessage, recv_ns: int) -> int:
        with self._lock:
            if self.last_recv_ns is not None:
                self.gaps_ns.append(recv_ns - self.last_recv_ns)
            self.last_recv_ns = recv_ns
            self.last_msg = msg
            self.total += 1
            if any(c is not None and not c.homogeneous_ok for c in msg.pose_checks.values()):
                self.homogeneous_warnings += 1
            self._recent.append(recv_ns)
            return self.total

    def record_error(self) -> None:
        with self._lock:
            self.errors += 1

    def set_transport(self, up: bool, note: str, now_ns: int) -> None:
        with self._lock:
            changed = up != self.transport_up or note != self.transport_note
            if changed:
                self.transport_events.append((now_ns, f"{'up' if up else 'down'}: {note}"))
            self.transport_up = up
            self.transport_note = note
        if changed and self.on_transport is not None:
            self.on_transport(up, note, now_ns)

    def snapshot(self, now_ns: int, consume: bool = True) -> Snapshot:
        with self._lock:
            while self._recent and now_ns - self._recent[0] > self.rate_window_ns:
                self._recent.popleft()
            rate = len(self._recent) / (self.rate_window_ns / 1e9)
            age_ns = None if self.last_recv_ns is None else now_ns - self.last_recv_ns
            if not self.transport_up:
                status = Status.DISCONNECTED
            elif age_ns is None or age_ns > self.stale_ns:
                status = Status.STALE
            else:
                status = Status.LIVE
            new = self.total - self._printed_total
            if consume:
                self._printed_total = self.total
            return Snapshot(
                status=status,
                new_messages=new,
                total_messages=self.total,
                parse_errors=self.errors,
                rate_hz=rate,
                age_ms=None if age_ns is None else age_ns / 1e6,
                last=self.last_msg,
                transport_note=self.transport_note,
            )


def gap_stats_ms(gaps_ns: list[int], stale_ms: float) -> dict[str, Any]:
    if not gaps_ns:
        return {"count": 0}
    ms = sorted(g / 1e6 for g in gaps_ns)
    p95 = ms[min(len(ms) - 1, max(0, math.ceil(0.95 * len(ms)) - 1))]  # nearest rank
    over = [round(g, 1) for g in ms if g > stale_ms]
    return {
        "count": len(ms),
        "median_ms": round(statistics.median(ms), 3),
        "p95_ms": round(p95, 3),
        "max_ms": round(ms[-1], 3),
        "gaps_over_stale_threshold": len(over),
        "largest_gaps_over_threshold_ms": sorted(over, reverse=True)[:10],
    }


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def _fmt(v: Any, spec: str = ".3f") -> str:
    return "-" if v is None else format(v, spec)


def _analog0(buttons: dict[str, Any], key: str) -> float | None:
    v = buttons.get(key)
    return v[0] if isinstance(v, tuple) and v else None


def _bool(buttons: dict[str, Any], key: str) -> str:
    v = buttons.get(key)
    return "-" if not isinstance(v, bool) else ("1" if v else "0")


def format_status(s: Snapshot) -> str:
    m = s.last
    if m is None:
        pres, pos, btn = "r=- l=-", "-", "trig=- grip=- A=- B=-"
    else:
        p = m.pose_present
        pres = f"r={'Y' if p['r'] else 'n'} l={'Y' if p['l'] else 'n'}"
        t = m.translation("r")
        pos = "-" if t is None else f"({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})"
        b = m.buttons
        btn = (
            f"trig={_fmt(_analog0(b, 'rightTrig'), '.2f')} grip={_fmt(_analog0(b, 'rightGrip'), '.2f')} "
            f"A={_bool(b, 'A')} B={_bool(b, 'B')}"
        )
    age = _fmt(s.age_ms, ".0f")
    note = "" if s.status != Status.DISCONNECTED else f" [{s.transport_note}]"
    return (
        f"{s.status:<12} new={s.new_messages:<3} rate={s.rate_hz:5.1f}Hz age={age:>6}ms "
        f"pose {pres} R.pos={pos} {btn} err={s.parse_errors} track=unknown{note}"
    )


def message_record(seq: int, recv_ns: int, line: LogLine, msg: ControllerMessage) -> dict[str, Any]:
    return {
        "seq": seq,
        "host_monotonic_ns": recv_ns,
        "device_log_time": line.device_log_time,
        "device_log_time_note": "device-clock logcat write time; not sensor capture time",
        "device_pid": line.pid,
        "poses": msg.poses,
        "pose_present": msg.pose_present,
        "pose_validation": {
            k: None
            if c is None
            else {
                "finite": True,
                "shape": [4, 4],
                "homogeneous_ok": c.homogeneous_ok,
                "bottom_row_err": c.bottom_row_err,
                "orthonormal_err": c.orthonormal_err,
                "det": c.det,
            }
            for k, c in msg.pose_checks.items()
        },
        "buttons": {k: list(v) if isinstance(v, tuple) else v for k, v in msg.buttons.items()},
        "unknown_button_tokens": msg.unknown_button_tokens,
        "tracking_valid": None,
        "tracking_valid_note": TRACKING_VALID_NOTE,
    }


def redact(serial: str) -> str:
    return "[REDACTED]" if serial else serial


# --------------------------------------------------------------------------
# adb source
# --------------------------------------------------------------------------


class AdbLogcatSource:
    """Supervises one ``adb logcat`` client subprocess for the app's pid.

    Only this monitor's own adb client processes are ever terminated; the
    shared adb server is left alone.
    """

    def __init__(
        self,
        adb: str,
        serial: str,
        on_line: Callable[[str, int], None],
        tracker: FreshnessTracker,
        package: str = PACKAGE,
        tag: str = TAG,
        initial_since: str | None = None,
        resume_from_last_seen: bool = False,
    ):
        self.adb = adb
        self.serial = serial
        self.on_line = on_line
        self.tracker = tracker
        self.package = package
        self.tag = tag
        self.stop_event = threading.Event()
        # Optional (Step 4B): start the first logcat at a given device epoch, and after an
        # app restart resume from the last line seen instead of "now", so the new process's
        # first records are not lost. The --pid filter still excludes other processes.
        self._next_since = initial_since
        self._resume = resume_from_last_seen
        self._last_seen: str | None = None
        self._proc: subprocess.Popen[bytes] | None = None
        self._proc_lock = threading.Lock()
        self._thread = threading.Thread(target=self._supervise, name="adb-supervisor", daemon=True)

    def _adb(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [self.adb, "-s", self.serial, *args],
            capture_output=True,
            text=True,
            timeout=ADB_CMD_TIMEOUT_S,
        )

    def _app_pid(self) -> tuple[int | None, str]:
        """(pid, note). pid None means app not running or device unavailable."""
        try:
            r = self._adb("shell", "pidof", self.package)
        except subprocess.TimeoutExpired:
            return None, "adb pidof timed out"
        out = r.stdout.strip()
        if r.returncode == 0 and out:
            try:
                return int(out.split()[0]), "app running"
            except ValueError:
                return None, "unexpected pidof output"
        err = (r.stderr or r.stdout).strip().replace(self.serial, redact(self.serial))
        if err:
            return None, f"device unavailable: {err.splitlines()[-1][:80]}"
        return None, "app not running"

    def _device_epoch(self) -> str | None:
        try:
            r = self._adb("shell", "date", "+%s.%N")
        except subprocess.TimeoutExpired:
            return None
        text = r.stdout.strip()
        if r.returncode != 0 or not re.fullmatch(r"\d+\.\d+", text):
            return None
        return text[: text.index(".") + 4]  # seconds.milliseconds

    def start(self) -> None:
        self._thread.start()

    def _now(self) -> int:
        return time.monotonic_ns()

    def _supervise(self) -> None:
        while not self.stop_event.is_set():
            pid, note = self._app_pid()
            if pid is None:
                self.tracker.set_transport(False, note, self._now())
                self.stop_event.wait(APP_POLL_INTERVAL_S)
                continue
            since = self._next_since or self._device_epoch()
            if since is None:
                self.tracker.set_transport(False, "could not read device clock", self._now())
                self.stop_event.wait(APP_POLL_INTERVAL_S)
                continue
            # -T <device time now>: skip messages buffered before this session.
            # The reader additionally drops lines older than `since`.
            cmd = [
                self.adb, "-s", self.serial, "logcat",
                f"--pid={pid}", "-T", since,
                "-v", "epoch", "-v", "usec", "-v", "threadtime",
                "-s", f"{self.tag}:V",
            ]  # fmt: skip
            with self._proc_lock:
                if self.stop_event.is_set():
                    return
                self._proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )  # fmt: skip
            proc = self._proc
            self.tracker.set_transport(True, f"logcat on app pid {pid}", self._now())
            reader = threading.Thread(target=self._read, args=(proc, float(since)), daemon=True)
            reader.start()
            reason = ""
            while not self.stop_event.wait(APP_POLL_INTERVAL_S):
                if proc.poll() is not None:
                    reason = "logcat exited"
                    break
                new_pid, pnote = self._app_pid()
                if new_pid != pid:
                    reason = pnote if new_pid is None else f"app restarted (pid {new_pid})"
                    break
            self._terminate(proc)
            reader.join(timeout=2.0)
            self._next_since = self._last_seen if (self._resume and self._last_seen) else None
            if reason:
                # Re-check to distinguish USB loss from app exit.
                _, pnote = self._app_pid()
                self.tracker.set_transport(False, f"{reason}; {pnote}", self._now())

    def _read(self, proc: subprocess.Popen[bytes], since: float) -> None:
        assert proc.stdout is not None
        for raw in iter(proc.stdout.readline, b""):
            recv_ns = time.monotonic_ns()
            line = raw.decode("utf-8", errors="replace")
            parsed = parse_logcat_line(line, self.tag)
            if parsed is not None and float(parsed.device_log_time) < since:
                continue  # buffered from before this session
            if parsed is not None:
                self._last_seen = parsed.device_log_time
            self.on_line(line, recv_ns)

    def _terminate(self, proc: subprocess.Popen[bytes]) -> None:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
        if proc.stdout is not None:
            proc.stdout.close()

    def stop(self) -> None:
        self.stop_event.set()
        with self._proc_lock:
            proc = self._proc
        if proc is not None:
            self._terminate(proc)
        self._thread.join(timeout=ADB_CMD_TIMEOUT_S + 3.0)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def find_adb() -> str:
    adb = shutil.which("adb")
    if adb is None:
        raise MonitorError("adb not found on PATH")
    return adb


def pick_serial(adb: str, serial: str | None) -> str:
    r = subprocess.run([adb, "devices"], capture_output=True, text=True, timeout=ADB_CMD_TIMEOUT_S)
    rows = [ln.split() for ln in r.stdout.splitlines()[1:] if ln.strip()]
    states = {row[0]: row[1] for row in rows if len(row) >= 2}
    if serial is not None:
        if states.get(serial) != "device":
            raise MonitorError(f"device {redact(serial)} not in 'device' state ({states.get(serial, 'absent')})")
        return serial
    ready = [s for s, st in states.items() if st == "device"]
    if len(ready) != 1:
        raise MonitorError(f"{len(ready)} authorized devices attached; pass --serial explicitly")
    return ready[0]


class Monitor:
    def __init__(
        self,
        tracker: FreshnessTracker,
        out: TextIO | None,
        tag: str = TAG,
        on_message: Callable[[int, int, LogLine, ControllerMessage], None] | None = None,
        on_error: Callable[[str, int], None] | None = None,
    ):
        self.tracker = tracker
        self.out = out
        self.tag = tag
        # Optional hooks for consumers of the live stream (e.g. the Step 3 sender).
        self.on_message = on_message
        self.on_error = on_error
        self.error_samples: list[str] = []
        self.buttons_seen: set[str] = set()
        self.presence_counts = {"r": 0, "l": 0, "both": 0, "none": 0}
        self._out_lock = threading.Lock()

    def handle_line(self, line: str, recv_ns: int) -> None:
        parsed = parse_logcat_line(line, self.tag)
        if parsed is None:
            return  # not an app controller line (e.g. "--------- beginning of main")
        try:
            msg = parse_message(parsed.message)
        except MessageError as e:
            self.tracker.record_error()
            if len(self.error_samples) < 20:
                self.error_samples.append(str(e))
            if self.on_error is not None:
                self.on_error(str(e), recv_ns)
            return
        seq = self.tracker.record_message(msg, recv_ns)
        p = msg.pose_present
        self.presence_counts["r"] += p["r"]
        self.presence_counts["l"] += p["l"]
        self.presence_counts["both"] += p["r"] and p["l"]
        self.presence_counts["none"] += not (p["r"] or p["l"])
        self.buttons_seen.update(msg.buttons)
        if self.out is not None:
            rec = json.dumps(message_record(seq, recv_ns, parsed, msg), separators=(",", ":"))
            with self._out_lock:
                self.out.write(rec + "\n")
        if self.on_message is not None:
            self.on_message(seq, recv_ns, parsed, msg)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serial", help="adb serial (required if more than one device)")
    ap.add_argument("--duration", type=float, help="seconds to run (default: until Ctrl+C)")
    ap.add_argument("--output", type=Path, help="JSONL file for valid messages (must not exist)")
    ap.add_argument("--summary", type=Path, help="write run summary JSON here (must not exist)")
    ap.add_argument("--stale-ms", type=float, default=DEFAULT_STALE_MS,
                    help="diagnostic stale threshold, ms (not a safety timeout)")  # fmt: skip
    ap.add_argument("--print-interval", type=float, default=DEFAULT_PRINT_INTERVAL_S)
    args = ap.parse_args(argv)

    try:
        adb = find_adb()
        serial = pick_serial(adb, args.serial)
        for p in (args.output, args.summary):
            if p is not None and p.exists():
                raise MonitorError(f"{p} already exists; refusing to overwrite")
    except (MonitorError, subprocess.TimeoutExpired) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    out: TextIO | None = None
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        out = args.output.open("x", encoding="utf-8")

    tracker = FreshnessTracker(stale_ms=args.stale_ms)
    monitor = Monitor(tracker, out)
    source = AdbLogcatSource(adb, serial, monitor.handle_line, tracker)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    start_ns = time.monotonic_ns()
    print(f"monitoring {PACKAGE} tag {TAG} on device {redact(serial)}; stale threshold "
          f"{args.stale_ms:.0f} ms (diagnostic only). Ctrl+C to stop.", flush=True)  # fmt: skip
    source.start()
    statuses: dict[str, int] = {}
    try:
        while not stop.wait(args.print_interval):
            now = time.monotonic_ns()
            snap = tracker.snapshot(now)
            statuses[snap.status] = statuses.get(snap.status, 0) + 1
            print(f"{(now - start_ns) / 1e9:7.1f}s {format_status(snap)}", flush=True)
            if args.duration is not None and now - start_ns >= args.duration * 1e9:
                break
    finally:
        end_ns = time.monotonic_ns()
        source.stop()
        elapsed_s = (end_ns - start_ns) / 1e9
        if out is not None:
            out.close()

    summary = {
        "device": redact(serial),
        "package": PACKAGE,
        "tag": TAG,
        "elapsed_s": round(elapsed_s, 3),
        "valid_messages": tracker.total,
        "parse_errors": tracker.errors,
        "parse_error_samples": monitor.error_samples,
        "homogeneous_check_failures": tracker.homogeneous_warnings,
        "mean_rate_hz": round(tracker.total / elapsed_s, 2) if elapsed_s > 0 else None,
        "inter_arrival": gap_stats_ms(tracker.gaps_ns, args.stale_ms),
        # Silence after the last message is not an inter-arrival gap; report it separately.
        "trailing_silence_ms": None
        if tracker.last_recv_ns is None
        else round((end_ns - tracker.last_recv_ns) / 1e6, 1),
        "stale_threshold_ms": args.stale_ms,
        "status_samples": statuses,
        "pose_presence_counts": monitor.presence_counts,
        "button_fields_seen": sorted(monitor.buttons_seen),
        "transport_events": [
            {"t_s": round((t - start_ns) / 1e9, 3), "event": ev} for t, ev in tracker.transport_events
        ],
        "tracking_valid": None,
        "tracking_valid_note": TRACKING_VALID_NOTE,
        "output": str(args.output) if args.output else None,
    }
    text = json.dumps(summary, indent=2)
    print(text)
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        with args.summary.open("x", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
