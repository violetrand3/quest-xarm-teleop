#!/usr/bin/env python3
"""The RealSense's colour AND depth as loopback MJPEG streams, on demand (the lab host's pyrealsense2, numpy, cv2).

Why: for the 3D scene in the headset (YAM_STEREO=depth; docs/yam_teleop.md). The headset page turns every pixel of
the colour picture into a point at its measured distance, so the operator looks INTO the scene and it shifts as the
head moves. One picture carries both: the colour picture on the left half, the distance on the right half as grey.
  grey 0        no distance measured there
  grey 24..255  distance z in metres:  z = NEAR + (grey - 24) / 231 * (FAR - NEAR)   (about 1 cm a step; far is bright;
                nearer than NEAR reads NEAR, further than FAR reads FAR). The gap between 0 and 24 keeps JPEG's
                small errors from turning "none" into a distance: the page takes anything below 12 as none.
NEAR and FAR are repeated in web/headset.html (tests/test_lab_rgbd.py checks they agree).

The colour and the depth sensor are opened together through librealsense, so in this mode the plain colour stream
comes from here too (lab_usb_cams.py's ffmpeg cannot hold the colour camera at the same time):
  GET /stream/rgbd       colour | distance, side by side
  GET /stream/rs_color   the colour picture alone (the overview on the PC's page; recorded with the demos)
  GET /stats             JSON counters
  --describe             print the lens data and the camera's tilt (for web/depth_camera.json) and exit
On demand like lab_usb_cams.py: the camera is opened only while a stream is watched and let go GRACE_S after.
Bound to 127.0.0.1 only; no robot or gripper code.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from typing import Any

from lab_usb_cams import GRACE_S, RETRY_S, make_handler

NEAR, FAR = 0.35, 2.9  # metres


def encode_depth(z: Any, np: Any) -> Any:
    """Distances in metres (0 = none) -> the grey values described above (uint8)."""
    g = 24.0 + 231.0 * (z - NEAR) / (FAR - NEAR)
    return np.where(z > 0, np.clip(np.rint(g), 24, 255), 0).astype(np.uint8)


def decode_depth(grey: float) -> float:
    return NEAR + (grey - 24.0) / 231.0 * (FAR - NEAR)


class View:
    """One stream: what lab_usb_cams.make_handler serves (the newest picture only)."""

    def __init__(self, source: "Source"):
        self.source = source
        self.cv = source.cv
        self.frame: bytes | None = None
        self.seq = 0
        self.clients = 0
        self.last_drop = 0.0
        self.frames = 0

    starts = property(lambda self: self.source.starts)
    failures = property(lambda self: self.source.failures)
    note = property(lambda self: self.source.note)

    def wanted(self) -> bool:
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

    def put(self, jpeg: bytes | None) -> None:
        with self.cv:
            self.frame = jpeg
            if jpeg is not None:
                self.seq += 1
                self.frames += 1
            self.cv.notify_all()


class Source(threading.Thread):
    """The camera: open while any view is watched."""

    def __init__(self, size: tuple[int, int], depth_size: tuple[int, int], fps: int, out_width: int, color_width: int, quality: int,
                 fill: bool = True):  # fmt: skip
        super().__init__(daemon=True)
        self.fill = fill
        self.size, self.depth_size, self.fps = size, depth_size, fps
        self.out_width, self.color_width, self.quality = out_width, color_width, quality
        self.cv = threading.Condition()
        self.starts = self.failures = 0
        self.note = "not opened yet"
        self.views = {"rgbd": View(self), "rs_color": View(self)}
        self.start()

    def wanted(self) -> bool:
        with self.cv:
            return any(v.wanted() for v in self.views.values())

    def run(self) -> None:
        import cv2
        import numpy as np
        import pyrealsense2 as rs

        w, h = self.size
        while True:
            with self.cv:
                while not any(v.clients for v in self.views.values()):
                    self.cv.wait(timeout=1.0)
            self.starts += 1
            pipe, got = rs.pipeline(), 0
            try:
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, self.fps)
                cfg.enable_stream(rs.stream.depth, self.depth_size[0], self.depth_size[1], rs.format.z16, self.fps)
                scale = pipe.start(cfg).get_device().first_depth_sensor().get_depth_scale()
                align = rs.align(rs.stream.color)  # the distance of every COLOUR pixel
                # Where the sensor measured nothing (beside every object: one of its two eyes cannot see there; shiny
                # and black parts) the page would show a hole. Keep the last measurement of a point that flickers,
                # then give what is still empty the distance of the furthest neighbour: the background goes on behind.
                steady, fill = rs.temporal_filter(), rs.hole_filling_filter(1)
                half = (self.out_width // 2, self.out_width // 2 * h // w)
                while self.wanted():
                    f = align.process(pipe.wait_for_frames(2000))
                    c, d = f.get_color_frame(), f.get_depth_frame()
                    if not c or not d:
                        continue
                    if self.fill:
                        d = fill.process(steady.process(d))
                    color = np.asanyarray(c.get_data())
                    got += 1
                    self.note = "running"
                    if self.views["rgbd"].wanted():
                        z = cv2.resize(np.asanyarray(d.get_data()), half, interpolation=cv2.INTER_NEAREST).astype(np.float32) * scale
                        grey = cv2.cvtColor(encode_depth(z, np), cv2.COLOR_GRAY2BGR)
                        both = np.hstack([cv2.resize(color, half, interpolation=cv2.INTER_AREA), grey])
                        self.views["rgbd"].put(cv2.imencode(".jpg", both, [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1].tobytes())
                    if self.views["rs_color"].wanted():
                        small = cv2.resize(color, (self.color_width, self.color_width * h // w), interpolation=cv2.INTER_AREA)
                        self.views["rs_color"].put(cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes())
                self.note = "idle (nobody watching): the camera is free"
            except Exception as e:  # in use elsewhere, unplugged, a mode it does not have
                self.failures += 1
                self.note = ("no picture: " if not got else "the camera stopped: ") + str(e)[:160]
                for v in self.views.values():
                    v.put(None)
                time.sleep(RETRY_S)
            finally:
                try:
                    pipe.stop()
                except Exception:
                    pass


def describe(size: tuple[int, int], fps: int) -> dict[str, Any]:
    """The colour lens (as fractions of the picture) and the camera's tilt from its accelerometer (at rest)."""
    import pyrealsense2 as rs

    out: dict[str, Any] = {}
    dev = rs.context().query_devices()[0]
    for s in dev.query_sensors():
        for p in s.get_stream_profiles():
            if p.stream_type() == rs.stream.color and p.format() == rs.format.bgr8 and p.fps() == fps:
                i = p.as_video_stream_profile().get_intrinsics()
                if (i.width, i.height) == size:
                    out.update(focal=round(i.fx / i.width, 4), cx=round(i.ppx / i.width, 4), cy=round(i.ppy / i.height, 4))
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.accel)
    pipe.start(cfg)
    a, n, t0 = [0.0, 0.0, 0.0], 0, time.monotonic()
    try:
        while n < 100 and time.monotonic() - t0 < 10:
            for fr in pipe.wait_for_frames(2000):
                if fr.is_motion_frame():
                    m = fr.as_motion_frame().get_motion_data()
                    a, n = [a[0] + m.x, a[1] + m.y, a[2] + m.z], n + 1
    finally:
        pipe.stop()
    g = math.sqrt(sum(v * v for v in a)) or 1.0
    # the accelerometer's axes are the picture's: x right, y down, z forward; at rest it reads straight up
    out.update(pitch_deg=round(math.degrees(math.asin(-a[2] / g)), 1), roll_deg=round(math.degrees(math.atan2(-a[0], -a[1])), 1))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=28098, help="loopback port (host is always 127.0.0.1)")
    ap.add_argument("--size", default="1280x720", help="the colour picture read from the camera")
    ap.add_argument("--depth-size", default="848x480", help="the depth picture read from the camera")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--out-width", type=int, default=1280, help="width of the colour | distance picture (both halves together)")
    ap.add_argument("--color-width", type=int, default=960, help="width of the colour picture alone")
    ap.add_argument("--quality", type=int, default=90, help="JPEG quality of the colour | distance picture")
    ap.add_argument("--no-fill", action="store_true", help="leave the places with no measured distance empty (holes in the scene)")
    ap.add_argument("--describe", action="store_true", help="print the lens data and the tilt as JSON and exit")
    ap.add_argument("--duration", type=float, help="exit after this many seconds")
    args = ap.parse_args(argv)
    size = tuple(int(v) for v in args.size.split("x"))
    dsize = tuple(int(v) for v in args.depth_size.split("x"))
    if args.describe:
        print(json.dumps(describe(size, args.fps)))  # type: ignore[arg-type]
        return 0
    src = Source(size, dsize, args.fps, args.out_width, args.color_width, args.quality, not args.no_fill)  # type: ignore[arg-type]
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(src.views))  # type: ignore[arg-type]
    srv.daemon_threads = True
    print(f"RealSense colour and depth (on demand) on 127.0.0.1:{args.port}: {list(src.views)}", flush=True)
    if args.duration:
        threading.Timer(args.duration, srv.shutdown).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
