"""REAL gripper client: talks to the lab's gripctl HTTP service (custom Dynamixel gripper).

THIS MOVES HARDWARE. It is imported only by live_gripper_test.py, never by the dry run.

Only the loopback service is accepted (http://127.0.0.1:<port>), and only two requests exist:
    GET  /api/state                      position (ticks), connected, snapshot age
    POST /api/do/grip {"value": pct}     percent closed, 0 = open (3033 ticks), 100 = closed (1181)
No torque, calibration, stop, open/close, jog, limit or raw-register request is ever made.
(`grip` itself enables torque and sets the driver's effort cap; that is the driver's behaviour.)

One worker thread polls the state and sends at most one command per `min_interval_s`, always
the NEWEST proposal (never a queue), and only if it differs from the last sent value by at
least `min_step_pct`. submit() and cancel() return at once, so the adapter's scheduler never
waits on HTTP. If the state cannot be read, is stale, or the servo is reported disconnected,
read() says "not connected" and nothing is sent; the adapter then stops following and a new
pickup is required. Nothing is retried and nothing is sent on close.
"""

from __future__ import annotations

import json
import re
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Callable

from teleop_backend import GripperFeedback, GripperProposal
from teleop_mapping import GRIPPER_OPEN_TICKS, GRIPPER_SPAN_TICKS


def ticks_to_pct_closed(ticks: float) -> float:
    return (GRIPPER_OPEN_TICKS - ticks) / GRIPPER_SPAN_TICKS * 100.0


class GripctlClient:
    def __init__(self, base_url: str = "http://127.0.0.1:8088", min_step_pct: float = 1.0, min_interval_s: float = 0.1,
                 state_period_s: float = 0.05, stale_s: float = 0.4, http_timeout_s: float = 1.0,
                 clock: Callable[[], int] = time.monotonic_ns, on_event: Callable[[str, dict], None] | None = None):  # fmt: skip
        if not re.fullmatch(r"http://127\.0\.0\.1:\d{2,5}", base_url):
            raise ValueError("only the loopback gripctl service (http://127.0.0.1:<port>) is accepted")
        self.base, self.clock = base_url, clock
        self.min_step, self.min_interval_ns = float(min_step_pct), int(min_interval_s * 1e9)
        self.state_period_s, self.stale_ns, self.timeout = state_period_s, int(stale_s * 1e9), http_timeout_s
        self.on_event = on_event or (lambda kind, rec: None)
        self._lock = threading.Lock()
        self._pending: GripperProposal | None = None
        self._last_sent_pct: float | None = None
        self._last_send_ns = 0
        self._state: dict | None = None  # newest good state
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.sent = self.send_errors = self.state_reads = self.state_errors = self.cancelled = 0
        self.send_ms: list = []

    # -- HTTP (the only two requests) ------------------------------------------------------------
    def _get_state(self) -> dict:
        with urllib.request.urlopen(self.base + "/api/state", timeout=self.timeout) as r:
            d = json.load(r)
        v, st = d.get("values") or {}, d.get("status") or {}
        ticks = v.get("position")
        ok = bool(st.get("connected")) and isinstance(ticks, (int, float)) and isinstance(d.get("age"), (int, float))
        return {"connected": ok, "ticks": ticks, "pct": ticks_to_pct_closed(ticks) if ok else None, "age_s": d.get("age"),
                "goal": v.get("goal_position"), "load": v.get("load"), "moving": v.get("moving"), "torque": v.get("torque_enable"),
                "read_ns": self.clock()}  # fmt: skip

    def _post_grip(self, pct: float) -> dict:
        req = urllib.request.Request(self.base + "/api/do/grip", data=json.dumps({"value": pct}).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})  # fmt: skip
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            try:
                return json.load(e)
            except Exception:
                return {"ok": False, "error": f"HTTP {e.code}"}

    # -- worker ------------------------------------------------------------------------------------
    def refresh_state(self) -> dict | None:
        """One state read (also used once at start-up, before the worker runs)."""
        try:
            s = self._get_state()
            self.state_reads += 1
        except Exception as e:
            self.state_errors += 1
            s = {"connected": False, "error": f"{type(e).__name__}: {e}", "read_ns": self.clock()}
        with self._lock:
            was = None if self._state is None else self._state.get("connected")
            self._state = s
        if was is not None and was != s["connected"]:
            self.on_event("gripper_link", {"connected": s["connected"], "error": s.get("error")})
        return s

    def _usable(self, s: dict | None, now: int) -> bool:
        return bool(s and s.get("connected") and now - s["read_ns"] <= self.stale_ns and (s.get("age_s") or 0) <= self.stale_ns / 1e9)

    def _maybe_send(self) -> None:
        now = self.clock()
        with self._lock:
            prop, state = self._pending, self._state
            if prop is None or now - self._last_send_ns < self.min_interval_ns:
                return
            if self._last_sent_pct is not None and abs(prop.pct_closed - self._last_sent_pct) < self.min_step:
                return
            if not self._usable(state, now):
                self._pending = None  # never send into a gripper whose state is unknown
                return
            self._pending = None
            self._last_send_ns = now
        t0 = self.clock()
        try:
            r = self._post_grip(prop.pct_closed)
            err = None if r.get("ok") else str(r.get("error"))
        except Exception as e:
            r, err = {}, f"{type(e).__name__}: {e}"
        ms = round((self.clock() - t0) / 1e6, 1)
        if err is None:
            self.sent += 1
            self.send_ms.append(ms)
            with self._lock:
                self._last_sent_pct = prop.pct_closed
        else:
            self.send_errors += 1
            with self._lock:
                self._state = {"connected": False, "error": "command failed: " + err, "read_ns": self.clock()}
        self.on_event("gripper_command", {"pct_closed": prop.pct_closed, "trigger": prop.trigger, "ok": err is None, "error": err,
                                          "ms": ms, "target_ticks": (r.get("grip") or {}).get("target"),
                                          "from_ticks": (r.get("grip") or {}).get("from"), "direction": (r.get("grip") or {}).get("direction")})  # fmt: skip

    def _run(self) -> None:
        while not self._stop.is_set():
            self.refresh_state()
            self._maybe_send()
            self._stop.wait(self.state_period_s)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gripctl", daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stops the worker. Sends nothing."""
        self._stop.set()
        with self._lock:
            self._pending = None
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    # -- adapter-facing (all non-blocking) -----------------------------------------------------------
    def read(self) -> GripperFeedback:
        now = self.clock()
        with self._lock:
            s = self._state
        ok = self._usable(s, now)
        return GripperFeedback(s["pct"] if ok else None, ok, s["read_ns"] if s else now, False)  # type: ignore[index]

    def submit(self, proposal: GripperProposal) -> None:
        with self._lock:
            self._pending = proposal  # newest only

    def cancel(self) -> None:
        with self._lock:
            if self._pending is not None:
                self.cancelled += 1
            self._pending = None
            self._last_sent_pct = None  # after a break, the next pickup starts from the jaws' real position

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state or {})
