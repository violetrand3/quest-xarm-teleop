#!/usr/bin/env python3
"""Camera side of episode recording: saves the camera streams of one episode at a time. VIEW ONLY.

Runs as a child of the teleop program, on the machine the cameras are on. It only ever sends HTTP
GET to loopback MJPEG streams (the lab's camera wall); it talks to no robot and no gripper. It is a
separate process so that nothing it does can delay the arm's control loop.

Commands arrive on stdin, one JSON object per line:
    {"cmd": "start", "dir": "<episode directory>", "t0": <monotonic ns>}
    {"cmd": "stop", "t1": <monotonic ns>}      keep frames up to t1, close the files
    {"cmd": "discard"}                           close and delete this episode's camera files
    {"cmd": "quit"}                              (end of stdin does the same; an open episode is closed first)
Events go to stdout, one JSON object per line: ready, status (once a second), started, stopped, discarded.

Per camera and episode:
    cam_<name>.mjpeg          the JPEG frames exactly as the camera wall sent them, back to back
    cam_<name>.frames.jsonl   one line per frame: {"i": index, "t": monotonic ns, "off": byte offset, "len": bytes}

"t" is THIS host's monotonic clock when the last byte of the frame arrived. The camera wall supplies
no capture time, so the delay from exposure to arrival is not known and is not corrected. It is the
same clock as the robot rows of the episode. The last few seconds of frames are kept in memory, so an
episode starts at its t0 even though the command arrives a little later.
"""

from __future__ import annotations

import argparse
import collections
import json
import queue
import re
import sys
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, BinaryIO, TextIO

RING_FRAMES = 120  # about 4 s at 30 fps
MAX_QUEUE = 900  # frames waiting for the disk; beyond this they are dropped and counted
STOP_GRACE_S = 0.5
URL_RE = re.compile(r"http://127\.0\.0\.1:\d{2,5}/[\w./?=&-]*")


def emit(ev: str, **fields: Any) -> None:
    sys.stdout.write(json.dumps(dict(ev=ev, **fields), separators=(",", ":")) + "\n")
    sys.stdout.flush()


def read_frames(resp: Any):  # type: ignore[no-untyped-def]
    """Yields the JPEG of each part of a multipart/x-mixed-replace stream (parts carry Content-Length)."""
    while True:
        length = None
        while True:
            line = resp.readline(4096)
            if not line:
                return
            line = line.strip()
            if line.lower().startswith(b"content-length:"):
                length = int(line.split(b":", 1)[1])
            elif not line and length is not None:
                break
        if not 100 <= length <= 8_000_000:
            raise ValueError(f"implausible frame length {length}")
        data = resp.read(length)
        if len(data) != length:
            return
        if data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
            raise ValueError("part is not a JPEG")
        yield data


class Camera(threading.Thread):
    def __init__(self, name: str, url: str, out: "queue.Queue", stop: threading.Event):
        super().__init__(name=f"cam-{name}", daemon=True)
        self.cam, self.url, self.out, self.stop_ev = name, url, out, stop
        self.frames = self.dropped = self.connects = 0
        self.last_t: int | None = None
        self.error: str | None = None

    def run(self) -> None:
        while not self.stop_ev.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=5.0) as resp:  # GET; nothing else is ever sent
                    self.connects += 1
                    self.error = None
                    for jpeg in read_frames(resp):
                        t = time.monotonic_ns()
                        self.frames, self.last_t = self.frames + 1, t
                        if self.out.qsize() > MAX_QUEUE:
                            self.dropped += 1
                        else:
                            self.out.put(("frame", self.cam, t, jpeg))
                        if self.stop_ev.is_set():
                            return
                self.error = "stream ended"
            except Exception as e:
                self.error = f"{type(e).__name__}: {e}"[:120]
            self.stop_ev.wait(1.0)


class Episode:
    def __init__(self, directory: Path, t0: int, cams: list):
        self.dir, self.t0, self.t1 = directory, t0, None
        self.stop_deadline: float | None = None
        self.files: dict[str, tuple[BinaryIO, TextIO]] = {}
        self.stats = {c: {"frames": 0, "bytes": 0, "first_t": None, "last_t": None, "max_gap_ms": 0.0} for c in cams}
        self.past_end: set = set()
        for c in cams:
            self.files[c] = ((directory / f"cam_{c}.mjpeg").open("xb"), (directory / f"cam_{c}.frames.jsonl").open("x", encoding="utf-8"))

    def add(self, cam: str, t: int, jpeg: bytes) -> None:
        if t < self.t0:
            return
        if self.t1 is not None and t > self.t1:
            self.past_end.add(cam)
            return
        s, (fv, fi) = self.stats[cam], self.files[cam]
        fi.write(json.dumps({"i": s["frames"], "t": t, "off": s["bytes"], "len": len(jpeg)}, separators=(",", ":")) + "\n")
        fv.write(jpeg)
        if s["last_t"] is not None:
            s["max_gap_ms"] = max(s["max_gap_ms"], round((t - s["last_t"]) / 1e6, 1))
        s["first_t"] = t if s["first_t"] is None else s["first_t"]
        s["frames"], s["bytes"], s["last_t"] = s["frames"] + 1, s["bytes"] + len(jpeg), t

    def close(self) -> dict:
        for fv, fi in self.files.values():
            fv.close()
            fi.close()
        for s in self.stats.values():
            span = (s["last_t"] - s["first_t"]) / 1e9 if s["frames"] > 1 else 0.0
            s["fps"] = round((s["frames"] - 1) / span, 2) if span > 0 else None
        return self.stats

    def delete(self) -> None:
        self.close()
        for c in self.files:
            for p in (self.dir / f"cam_{c}.mjpeg", self.dir / f"cam_{c}.frames.jsonl"):
                p.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cam", action="append", default=[], metavar="NAME=URL", help="loopback MJPEG stream; repeatable")
    a = ap.parse_args(argv)
    cams: dict[str, str] = {}
    for spec in a.cam:
        name, _, url = spec.partition("=")
        if not re.fullmatch(r"[a-z0-9_]{1,24}", name) or not URL_RE.fullmatch(url) or name in cams:
            print(f"error: bad camera {spec!r}: need NAME=http://127.0.0.1:<port>/<path>", file=sys.stderr)
            return 2
        cams[name] = url
    if not cams:
        print("error: no camera given", file=sys.stderr)
        return 2
    q: "queue.Queue" = queue.Queue()
    stop = threading.Event()
    threads = {n: Camera(n, u, q, stop) for n, u in cams.items()}

    def stdin_reader() -> None:
        for line in sys.stdin:
            try:
                q.put(("cmd", json.loads(line)))
            except ValueError:
                q.put(("cmd", {"cmd": "bad", "line": line[:80]}))
        q.put(("cmd", {"cmd": "quit"}))

    for th in threads.values():
        th.start()
    threading.Thread(target=stdin_reader, name="stdin", daemon=True).start()
    ring: dict[str, collections.deque] = {n: collections.deque(maxlen=RING_FRAMES) for n in cams}
    ep: Episode | None = None
    next_status, seen = time.monotonic() + 1.0, {n: 0 for n in cams}
    emit("ready", cams=sorted(cams))

    def finish() -> None:
        nonlocal ep
        if ep is not None:
            emit("stopped", dir=str(ep.dir), t0=ep.t0, t1=ep.t1, cams=ep.close())
            ep = None

    while True:
        try:
            item = q.get(timeout=0.1)
        except queue.Empty:
            item = None
        now = time.monotonic()
        if item is not None and item[0] == "frame":
            _, cam, t, jpeg = item
            ring[cam].append((t, jpeg))
            if ep is not None:
                ep.add(cam, t, jpeg)
        elif item is not None:
            c = item[1]
            cmd = c.get("cmd")
            if cmd == "start" and ep is None:
                try:
                    d = Path(c["dir"])
                    ep = Episode(d, int(c["t0"]), sorted(cams))
                    for cam in sorted(cams):
                        for t, jpeg in ring[cam]:
                            ep.add(cam, t, jpeg)
                    emit("started", dir=str(d), t0=ep.t0)
                except Exception as e:
                    ep = None
                    emit("error", what="start", error=f"{type(e).__name__}: {e}"[:200])
            elif cmd == "stop" and ep is not None:
                ep.t1, ep.stop_deadline = int(c["t1"]), now + STOP_GRACE_S
            elif cmd == "discard" and ep is not None:
                d = ep.dir
                ep.delete()
                ep = None
                emit("discarded", dir=str(d))
            elif cmd == "quit":
                if ep is not None and ep.t1 is None:
                    ep.t1 = time.monotonic_ns()
                finish()
                break
            else:
                emit("error", what=str(cmd), error="command not valid now")
        if ep is not None and ep.stop_deadline is not None and (now >= ep.stop_deadline or len(ep.past_end) == len(cams)):
            finish()
        if now >= next_status:
            next_status = now + 1.0
            t_ns = time.monotonic_ns()
            emit("status", t=t_ns, recording=None if ep is None else str(ep.dir), cams={
                n: {"fps": th.frames - seen[n], "age_ms": None if th.last_t is None else round((t_ns - th.last_t) / 1e6),
                    "dropped": th.dropped, "connects": th.connects, "error": th.error} for n, th in threads.items()})  # fmt: skip
            seen = {n: th.frames for n, th in threads.items()}
    stop.set()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:  # the parent is gone; an open episode's files stay for inspection
        sys.exit(0)
