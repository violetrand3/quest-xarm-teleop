#!/usr/bin/env python3
"""The headset's VR web page as the pose source, in place of the headset app (YAM_HEADSET=web).

Why: the headset shows one VR program at a time, and the app (org.questxarm.reader) draws nothing. This page, opened
in the headset's own browser, shows the camera pictures in VR AND reads the controllers. It sends the same records the
app writes (QuestXarmPoseV1, build "webxr-1": WebXR's `local` space and grip pose are OpenXR's LOCAL space and grip
pose), one per displayed frame, so everything after the sender is unchanged: the same checks, the same continuity
tracking, the same link to the lab host.

Runs on the operator's PC inside quest_pose_stream_sender.py (--source web), bound to 127.0.0.1 only. The headset
reaches it over its USB cable (`adb reverse tcp:<port> tcp:<port>`; http://localhost is a secure origin, which WebXR
needs). Standard library only.
  GET /             the page (web/headset.html, with web/records.js)
  GET /config.json  the views to show
  GET /cam/<view>?p=<page>  that view's pictures, passed through from the camera relay's tunnel on this PC: only for
                    the page that holds the link (/ws?v=2&p=<page>), 409 for any other. A browser opens only a few
                    connections to one address, and the picture streams never end: tabs left open from earlier
                    sessions used them all up, and the tab in use got the status but no picture (2026-11-23).
  GET /ws?v=2       WebSocket: the page sends one record per text message; this side sends {"status": "..."} lines.
                    One page at a time, and the NEWEST one: a page that connects takes the link from the page that had
                    it (closed with code 4001; that page does not come back by itself). So a tab left open from an
                    earlier session cannot keep the link from the tab the operator is looking at (2026-11-23: it did,
                    and the headset showed "NOT CONNECTED"). Pages from before this rule (no v=2) are refused.
"""

from __future__ import annotations

import base64
import hashlib
import json
import re
import socket
import struct
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

WEB_BUILD = "webxr-1"
PAGE = Path(__file__).resolve().parent / "web" / "headset.html"
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_MESSAGE_BYTES = 8192  # a record is at most 3000 bytes (quest_pose_v1.MAX_RECORD_BYTES)
STATUS_EVERY_S = 0.25


class WsClosed(Exception):
    pass


def ws_accept(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()


def ws_frame(text: str) -> bytes:
    """One unmasked text frame (server to browser)."""
    data = text.encode("utf-8")
    n = len(data)
    head = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack(">H", n) if n < 65536 else bytes([0x81, 127]) + struct.pack(">Q", n)
    return head + data


def ws_read(rfile: Any) -> tuple[int, bytes]:
    """One whole message from the browser: (opcode, payload). Raises WsClosed at the end or on anything not allowed."""

    def take(n: int) -> bytes:
        b = rfile.read(n)
        if len(b) != n:
            raise WsClosed("connection ended")
        return b

    op, out = None, b""
    while True:
        b0, b1 = take(2)
        fin, code, masked, n = b0 & 0x80, b0 & 0x0F, b1 & 0x80, b1 & 0x7F
        if n == 126:
            n = struct.unpack(">H", take(2))[0]
        elif n == 127:
            n = struct.unpack(">Q", take(8))[0]
        if not masked or n + len(out) > MAX_MESSAGE_BYTES:
            raise WsClosed("unmasked or oversized frame")
        mask = take(4)
        data = bytes(c ^ mask[i & 3] for i, c in enumerate(take(n)))
        if code >= 8:  # control frames come whole, possibly between the parts of a message
            return code, data
        op = code if op is None else op
        out += data
        if fin:
            return op, out


class WebAcquisition:
    """The sender's acquisition object for the web page (the interface of OwnedLogcatAcquisition). One page at a time."""

    child_pid = None

    def __init__(self, slot: Any, on_event: Callable[[dict[str, Any]], None], port: int, cams_url: str,
                 views: list[str], big: list[str], status_file: Path | None = None, page: Path = PAGE,
                 stereo: list[str] | None = None, depth: list[str] | None = None):  # fmt: skip
        self.slot, self.on_event, self.port, self.cams_url = slot, on_event, port, cams_url.rstrip("/")
        self.views, self.big, self.status_file, self.page = views, big, status_file, page
        self.stereo = stereo or []  # views whose picture holds both eyes side by side (shown in 3D)
        # views whose picture holds colour | distance side by side (scripts/lab_rgbd.py), shown as a 3D scene, with
        # the camera's lens and tilt (web/depth_camera.json: `lab_rgbd.py --describe` on the lab host prints them)
        self.depth = dict(json.loads((page.parent / "depth_camera.json").read_text()), views=depth) if depth else None
        self.gen = 0
        self._lock = threading.Lock()
        self._client: Any = None  # the handler of the connected page
        self._page: str | None = None  # its name for itself (p=...): the only page that gets camera pictures
        self._running = False
        self.srv = ThreadingHTTPServer(("127.0.0.1", port), self._handler())
        self.srv.daemon_threads = True
        self._thread = threading.Thread(target=self.srv.serve_forever, name="web-source", daemon=True)

    # -- the acquisition interface ---------------------------------------------------------------------------
    def start_poll(self) -> None:
        self._thread.start()

    def start_generation(self, note: str) -> int:
        if self._running:
            raise RuntimeError("a generation is already running; abort it first")
        now = time.monotonic_ns()
        self.gen = self.slot.new_generation(now, note)
        self._running = True
        with self._lock:
            up = self._client is not None
        self.slot.set_source(up, "headset page connected" if up else "waiting for the headset page", now)
        self.on_event({"t": now, "event": "acquisition_generation_started", "gen": self.gen, "child_pid": None, "note": note})
        return self.gen

    def abort_generation(self, reason: str) -> dict[str, Any]:
        now = time.monotonic_ns()
        info = {"gen": self.gen, "reason": reason, "child_pid": None}
        self._running = False
        self.slot.set_source(False, f"acquisition aborted: {reason}", now)
        self.slot.new_generation(now, f"aborted: {reason}")  # records of the old generation are never accepted
        self.on_event(dict(info, t=now, event="acquisition_generation_aborted"))
        return info

    def stop(self) -> None:
        self._running = False
        self.srv.shutdown()
        self.srv.server_close()

    # -- the page's connection ---------------------------------------------------------------------------------
    def _status_text(self) -> str:
        try:
            return self.status_file.read_text(encoding="utf-8", errors="replace").strip()[:600] if self.status_file else ""
        except OSError:
            return ""

    def _serve_ws(self, h: BaseHTTPRequestHandler) -> None:
        q = self._query(h.path)
        if q.get("v") != "2" or not re.fullmatch(r"[0-9a-f]{8,32}", q.get("p", "")):  # also a page of this morning: v=2, no name
            h.send_error(409, "this page is from an earlier version: reload it")
            return
        with self._lock:
            old, self._client = self._client, h
            self._page = q["p"]
        if old is not None:  # the newest page takes the link
            try:
                with old.ws_send_lock:
                    old.wfile.write(bytes([0x88, 2]) + struct.pack(">H", 4001))
                    old.wfile.flush()
                old.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        send_lock = h.ws_send_lock = threading.Lock()
        done = threading.Event()

        def send(text: str) -> None:
            with send_lock:
                h.wfile.write(ws_frame(text))
                h.wfile.flush()

        def status_loop() -> None:
            last = None
            while not done.wait(STATUS_EVERY_S):
                text = self._status_text()
                if text != last:
                    last = text
                    try:
                        send(json.dumps({"status": text}))
                    except OSError:
                        return

        try:
            h.send_response(101, "Switching Protocols")
            h.send_header("Upgrade", "websocket")
            h.send_header("Connection", "Upgrade")
            h.send_header("Sec-WebSocket-Accept", ws_accept(h.headers["Sec-WebSocket-Key"]))
            h.end_headers()
            h.wfile.flush()
            if self._running:
                self.slot.set_source(True, "headset page connected", time.monotonic_ns())
            threading.Thread(target=status_loop, daemon=True).start()
            while True:
                op, data = ws_read(h.rfile)
                if op == 8:
                    break
                if op == 9:
                    with send_lock:
                        h.wfile.write(bytes([0x8A, len(data)]) + data)
                        h.wfile.flush()
                elif op == 1 and self._running:
                    self.slot.on_message(data.decode("utf-8", errors="replace"), time.monotonic_ns(), self.gen, f"{time.time():.6f}")
        except (WsClosed, OSError, KeyError):
            pass
        finally:
            done.set()
            with self._lock:
                mine = self._client is h
                if mine:
                    self._client, self._page = None, None
            if mine and self._running:
                self.slot.set_source(False, "headset page not connected", time.monotonic_ns())
            h.close_connection = True

    @staticmethod
    def _query(path: str) -> dict[str, str]:
        return dict(kv.split("=", 1) for kv in path.partition("?")[2].split("&") if "=" in kv)

    def _serve_cam(self, h: BaseHTTPRequestHandler, name: str) -> None:
        with self._lock:
            page = self._page
        if not page or self._query(h.path).get("p") != page:
            h.send_error(409, "pictures go to the page that holds the link")
            return
        try:
            r = urllib.request.urlopen(f"{self.cams_url}/cam/{name}", timeout=8)
        except OSError:
            h.send_error(502, "the camera view is not running (scripts/teleop yam-cameras)")
            return
        try:
            h.send_response(200)
            h.send_header("Content-Type", r.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=frame"))
            h.send_header("Cache-Control", "no-cache, private")
            h.end_headers()
            while self._page == page:  # the page lost the link: its pictures end too
                chunk = r.read1(65536)
                if not chunk:
                    break
                h.wfile.write(chunk)
        except OSError:
            pass
        finally:
            r.close()
            h.close_connection = True

    def _handler(self) -> type:
        acq = self

        class H(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def _send(self, code: int, ctype: str, body: bytes) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                p = self.path.split("?")[0]
                if p in ("/", "/index.html"):
                    self._send(200, "text/html; charset=utf-8", acq.page.read_bytes())
                elif p == "/records.js":
                    self._send(200, "text/javascript; charset=utf-8", (acq.page.parent / "records.js").read_bytes())
                elif p == "/config.json":
                    self._send(200, "application/json", json.dumps({"build": WEB_BUILD, "views": acq.views, "big": acq.big, "stereo": acq.stereo, "depth": acq.depth}).encode())
                elif p == "/ws" and "websocket" in self.headers.get("Upgrade", "").lower():
                    acq._serve_ws(self)
                elif p.startswith("/cam/") and re.fullmatch(r"[a-z0-9_]+", p[5:]) and p[5:] in acq.views:
                    acq._serve_cam(self, p[5:])
                else:
                    self.send_error(404)

            def log_message(self, *a: Any) -> None:
                pass

        return H
