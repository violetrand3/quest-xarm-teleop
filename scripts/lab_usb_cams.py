#!/usr/bin/env python3
"""USB cameras on the lab host as loopback MJPEG streams, on demand (standard library + ffmpeg).

Why: the ZED 2i gives both its eyes side by side in one picture (2560x720), which the lab's camera wall does not
publish (it shows the left eye only). This serves that picture, for the 3D view in the headset (YAM_STEREO=zed;
docs/yam_teleop.md). The camera relay (lab_cam_relay.py) takes it from here like any other loopback stream.

On demand: a camera is opened only while somebody is watching its stream and is let go GRACE_S after the last one
leaves, so other programs can use it the rest of the time. If another program holds the camera, the stream stays
empty and the camera is tried again every few seconds. Bound to 127.0.0.1 only; no robot or gripper code.
  GET /stream/<name>   multipart MJPEG (each part with Content-Length)
  GET /stats           JSON counters
"""

from __future__ import annotations

import argparse
import json
import os
import re
import select
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

GRACE_S = 3.0
RETRY_S = 3.0
SOI, EOI = b"\xff\xd8", b"\xff\xd9"


def split_jpegs(buf: bytes) -> tuple[list[bytes], bytes]:
    """The whole JPEG pictures at the start of ffmpeg's output so far, and what is left over."""
    out = []
    while True:
        a = buf.find(SOI)
        b = buf.find(EOI, a + 2) if a >= 0 else -1
        if a < 0 or b < 0:
            return out, (buf[a:] if a >= 0 else b"")
        out.append(buf[a : b + 2])
        buf = buf[b + 2 :]


def ffmpeg_cmd(dev: str, size: str, fps: int, fmt: str, out_width: int, quality: int, out_fps: int) -> list[str]:
    return ["ffmpeg", "-v", "error", "-nostdin", "-f", "v4l2", "-input_format", fmt, "-video_size", size, "-framerate", str(fps),
            "-i", dev, "-vf", f"fps={min(fps, out_fps)},scale={out_width}:-2", "-q:v", str(quality), "-f", "mjpeg", "-"]  # fmt: skip


class Camera(threading.Thread):
    """One camera: ffmpeg runs while the stream is watched; only the newest picture is kept."""

    def __init__(self, name: str, cmd: list[str]):
        super().__init__(daemon=True)
        self.name_, self.cmd = name, cmd
        self.cv = threading.Condition()
        self.frame: bytes | None = None
        self.seq = 0
        self.clients = 0
        self.last_drop = 0.0
        self.frames = self.starts = self.failures = 0
        self.note = "not opened yet"
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

    def next_frame(self, last_seq: int, timeout: float = 2.0) -> tuple[bytes | None, int]:
        with self.cv:
            if self.seq == last_seq:
                self.cv.wait(timeout=timeout)
            return self.frame, self.seq

    def run(self) -> None:
        while True:
            with self.cv:
                while self.clients == 0:
                    self.cv.wait(timeout=1.0)
            self.starts += 1
            got = 0
            try:
                p = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except OSError as e:
                self.note = f"ffmpeg did not start: {e}"
                time.sleep(RETRY_S)
                continue
            err: list[str] = []  # ffmpeg's last messages (read as they come, so a full pipe never stops it)
            threading.Thread(target=lambda: err.extend(line.decode("utf-8", "replace").strip() for line in p.stderr), daemon=True).start()  # type: ignore[union-attr]
            try:
                assert p.stdout is not None
                buf = b""
                fd = p.stdout.fileno()
                while self.wanted():
                    if not select.select([fd], [], [], 0.5)[0]:
                        continue  # nothing arrived: look again whether anybody still watches (a stalled camera is let go too)
                    chunk = os.read(fd, 65536)  # what has arrived (a buffered read() would wait for a full block: several pictures late)
                    if not chunk:
                        break
                    pics, buf = split_jpegs(buf + chunk)
                    if len(buf) > 16_000_000:
                        buf = b""
                    if pics:
                        got += len(pics)
                        self.frames += len(pics)
                        self.note = "running"
                        with self.cv:
                            self.frame = pics[-1]
                            self.seq += 1
                            self.cv.notify_all()
            finally:
                p.kill()
                p.wait()
                time.sleep(0.05)
            if self.wanted():  # it ended by itself: in use elsewhere, unplugged, a size it does not have
                self.failures += 1
                self.note = ("no picture: " + (err[-1][-160:] if err else "ffmpeg ended")) if not got else "the camera stopped"
                with self.cv:
                    self.frame = None
                time.sleep(RETRY_S)
            else:
                self.note = "idle (nobody watching): the camera is free"


def make_handler(cams: dict[str, Camera]) -> type:
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self) -> None:  # noqa: N802
            p = self.path.split("?")[0]
            if p == "/stats":
                body = json.dumps({k: {"clients": c.clients, "frames": c.frames, "starts": c.starts, "failures": c.failures,
                                       "note": c.note} for k, c in cams.items()}).encode()  # fmt: skip
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif p.startswith("/stream/") and p[8:] in cams:
                c = cams[p[8:]]
                c.acquire()
                try:
                    self.send_response(200)
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
                    self.end_headers()
                    seq = -1
                    while True:
                        jpeg, new = c.next_frame(seq)
                        if jpeg is None or new == seq:  # nothing to send: has the watcher gone?
                            if select.select([self.connection], [], [], 0)[0] and not self.connection.recv(1, socket.MSG_PEEK):
                                break
                            continue
                        seq = new
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    c.release()
                    self.close_connection = True
            else:
                self.send_error(404)

        def log_message(self, *a: Any) -> None:
            pass

    return H


def parse_cam(item: str) -> tuple[str, str, str, int, str, int | None]:
    """name=<device>,<width>x<height>,<fps>,<format>[,<width served>], e.g.
    zed=/dev/v4l/by-id/usb-...-video-index0,2560x720,15,yuyv422,1280"""
    m = re.fullmatch(r"([a-z0-9_]+)=(/dev/(?:v4l/by-(?:id|path)/[A-Za-z0-9_.:+-]+|video\d+)),(\d+x\d+),(\d+),([a-z0-9]+)(?:,(\d+))?", item)
    if not m:
        raise ValueError(f"camera {item!r}: expected name=/dev/v4l/by-id/<camera>,<W>x<H>,<fps>,<format>[,<width served>]")
    return m.group(1), m.group(2), m.group(3), int(m.group(4)), m.group(5), int(m.group(6)) if m.group(6) else None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=28097, help="loopback port (host is always 127.0.0.1)")
    ap.add_argument("--cam", action="append", default=[], help=parse_cam.__doc__)
    ap.add_argument("--out-width", type=int, default=960, help="width of the pictures served when a camera does not give its own")
    ap.add_argument("--out-fps", type=int, default=15, help="at most this many pictures a second per camera")
    ap.add_argument("--quality", type=int, default=5, help="ffmpeg's JPEG scale: 2 best, 31 worst")
    ap.add_argument("--duration", type=float, help="exit after this many seconds")
    args = ap.parse_args(argv)
    try:
        specs = [parse_cam(c) for c in args.cam]
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if not specs:
        print("error: no --cam given", file=sys.stderr)
        return 2
    cams = {n: Camera(n, ffmpeg_cmd(dev, size, fps, fmt, w or args.out_width, args.quality, args.out_fps)) for n, dev, size, fps, fmt, w in specs}
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(cams))
    srv.daemon_threads = True
    print(f"USB cameras (on demand) on 127.0.0.1:{args.port}: {list(cams)}, at most {args.out_fps} pictures a second each", flush=True)
    if args.duration:
        threading.Timer(args.duration, srv.shutdown).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
