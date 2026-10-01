#!/usr/bin/env python3
"""Low-rate, view-only relay of the lab's existing camera wall, for the remote operator.

Why: the camera wall streams every camera at full rate. Over the remote link that saturates the
path and delays the Quest control packets. This relay re-serves chosen views at a capped frame
rate, reduced width and lower JPEG quality, always sending the NEWEST frame (never a backlog).

It is a viewer of the existing camera wall (HTTP GET on loopback). It opens no camera device,
changes no camera setting, and has no robot or gripper code. Like the wall, it is on demand:
it reads a camera only while somebody is watching that view.

Runs on the lab host, bound to 127.0.0.1 only. Reach it through an SSH tunnel.
  GET /                 page with the chosen views
  GET /cam/<key>        MJPEG; each part carries X-Seq and X-Lab-Wall-Time (relay receive time)
  GET /stats            JSON counters
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

GRACE_S = 3.0


class View(threading.Thread):
    """Pulls one camera-wall stream while watched; keeps only the newest re-encoded frame."""

    def __init__(self, key: str, upstream: str, width: int, quality: int):
        super().__init__(daemon=True)
        self.key, self.url, self.width, self.quality = key, f"{upstream}/cam/{key}", width, quality
        self.cv = threading.Condition()
        self.frame: bytes | None = None
        self.seq = 0
        self.frame_wall = 0.0
        self.clients = 0
        self.last_drop = 0.0
        self.frames_in = self.frames_out = self.bytes_out = self.errors = 0
        self.start()

    def wanted(self) -> bool:
        with self.cv:
            return self.clients > 0 or time.monotonic() - self.last_drop < GRACE_S

    def acquire(self) -> None:
        with self.cv:
            self.clients += 1
            self.cv.notify_all()

    def release(self) -> None:
        with self.cv:
            self.clients -= 1
            self.last_drop = time.monotonic()

    def _encode(self, jpeg: bytes) -> bytes | None:
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        h, w = img.shape[:2]
        if w > self.width:
            img = cv2.resize(img, (self.width, int(h * self.width / w)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return buf.tobytes() if ok else None

    def run(self) -> None:
        while True:
            with self.cv:
                while self.clients == 0:
                    self.cv.wait(timeout=1.0)
            try:
                r = urllib.request.urlopen(self.url, timeout=8)
                try:
                    while self.wanted():
                        line = r.readline()
                        if not line:
                            break
                        if not line.startswith(b"--frame"):
                            continue
                        n = None
                        while True:
                            h = r.readline().strip()
                            if not h:
                                break
                            if h.lower().startswith(b"content-length"):
                                n = int(h.split(b":")[1])
                        if n is None:
                            break
                        data = r.read(n)
                        self.frames_in += 1
                        with self.cv:
                            busy = self.clients == 0
                        if busy:
                            continue
                        out = self._encode(data)
                        if out is not None:
                            with self.cv:
                                self.frame, self.frame_wall = out, time.time()
                                self.seq += 1
                                self.cv.notify_all()
                finally:
                    r.close()
            except Exception:
                self.errors += 1
                time.sleep(1.0)

    def next_frame(self, last_seq: int, timeout: float = 2.0) -> tuple:
        with self.cv:
            if self.seq == last_seq:
                self.cv.wait(timeout=timeout)
            return self.frame, self.seq, self.frame_wall


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Lab cameras (low rate, view-only)</title>
<style>body{margin:0;background:#111;color:#ccc;font:13px sans-serif}
.row{display:flex;gap:6px;padding:6px;height:calc(100vh - 34px)}
.v{flex:1;display:flex;flex-direction:column;min-width:0}.v img{width:100%;height:100%;object-fit:contain;background:#000}
.b{padding:6px 10px}</style></head><body>
<div class="b">Lab cameras, low rate, view-only (no robot controls). __INFO__</div>
<div class="row">__VIEWS__</div></body></html>"""


def make_handler(views: dict, fps: float, info: str):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _send(self, code: int, ctype: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            p = self.path.split("?")[0]
            if p in ("/", "/index.html"):
                tiles = "".join(f'<div class="v"><div>{k}</div><img src="/cam/{k}" alt="{k}"></div>' for k in views)
                self._send(200, "text/html; charset=utf-8", PAGE.replace("__VIEWS__", tiles).replace("__INFO__", info).encode())
            elif p == "/stats":
                self._send(200, "application/json", json.dumps({k: {"clients": v.clients, "frames_in": v.frames_in, "frames_out": v.frames_out,
                                                                    "bytes_out": v.bytes_out, "errors": v.errors} for k, v in views.items()}).encode())  # fmt: skip
            elif p.startswith("/cam/") and p[5:] in views:
                v = views[p[5:]]
                v.acquire()
                try:
                    self.send_response(200)
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    seq, period, last = -1, 1.0 / fps, 0.0
                    while True:
                        jpeg, seq, wall = v.next_frame(seq)
                        if jpeg is None:
                            continue
                        wait = period - (time.monotonic() - last)
                        if wait > 0:
                            time.sleep(wait)
                            jpeg, seq, wall = v.next_frame(-1, timeout=0)  # newest frame after the pause, not the one we held
                        last = time.monotonic()
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"X-Seq: {seq}\r\nX-Lab-Wall-Time: {wall:.3f}\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        v.frames_out += 1
                        v.bytes_out += len(jpeg)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    v.release()
            else:
                self.send_error(404)

        def log_message(self, *a) -> None:  # type: ignore[no-untyped-def]
            pass

    return H


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=28090, help="loopback port (host is always 127.0.0.1)")
    ap.add_argument("--upstream", default="http://127.0.0.1:8090", help="the existing camera wall (loopback)")
    ap.add_argument("--views", default="iphone,zed")
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--quality", type=int, default=55)
    ap.add_argument("--duration", type=float, help="exit after this many seconds")
    args = ap.parse_args(argv)
    if not args.upstream.startswith("http://127.0.0.1:"):
        print("error: upstream must be the loopback camera wall", file=sys.stderr)
        return 2
    keys = [k for k in args.views.split(",") if k]
    views = {k: View(k, args.upstream, args.width, args.quality) for k in keys}
    info = f"{args.fps:g} fps max, {args.width}px wide, JPEG quality {args.quality}"
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(views, args.fps, info))
    srv.daemon_threads = True
    print(f"camera relay (view-only) on 127.0.0.1:{args.port}: views {keys}, {info}", flush=True)
    if args.duration:
        threading.Timer(args.duration, srv.shutdown).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
