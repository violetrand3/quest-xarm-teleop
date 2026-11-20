#!/usr/bin/env python3
"""The lab host's on-demand USB camera server (lab_usb_cams.py): its camera descriptions, how it cuts ffmpeg's output
into pictures, and that a camera is opened only while watched. Runs anywhere (a stand-in program plays ffmpeg)."""

from __future__ import annotations

import json
import sys
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import lab_usb_cams as uc  # noqa: E402

PIC = lambda body: b"\xff\xd8" + body + b"\xff\xd9"  # noqa: E731
# plays ffmpeg: four pictures a little apart, then goes on waiting (as a camera does)
FAKE = [sys.executable, "-u", "-c", "import sys,time\nfor i in range(4):\n sys.stdout.buffer.write(b'\\xff\\xd8pic%d\\xff\\xd9' % i); sys.stdout.buffer.flush(); time.sleep(0.05)\ntime.sleep(60)"]


class PartsTests(unittest.TestCase):
    def test_camera_descriptions(self) -> None:
        self.assertEqual(uc.parse_cam("zed=/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0,2560x720,15,yuyv422,1280"),
                         ("zed", "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0", "2560x720", 15, "yuyv422", 1280))  # fmt: skip
        self.assertEqual(uc.parse_cam("left_wrist=/dev/v4l/by-path/pci-0000:46:00.1-usb-0:2.4.2:1.0-video-index0,1280x720,30,mjpeg")[5], None)
        for bad in ("zed=/etc/passwd,1x1,1,mjpeg", "zed=/dev/v4l/by-id/x;rm,1x1,1,mjpeg", "Zed=/dev/video0,1x1,1,mjpeg", "zed=/dev/video0,1x1,1"):
            with self.assertRaises(ValueError):
                uc.parse_cam(bad)
        cmd = uc.ffmpeg_cmd("/dev/video0", "2560x720", 15, "yuyv422", 1280, 5, 30)
        self.assertIn("fps=15,scale=1280:-2", cmd)  # never more pictures than the camera gives

    def test_pictures_are_cut_whole(self) -> None:
        a, b, c = PIC(b"one"), PIC(b"two"), PIC(b"three")
        pics, rest = uc.split_jpegs(a + b + c[:4])
        self.assertEqual((pics, rest), ([a, b], c[:4]))
        pics, rest = uc.split_jpegs(rest + c[4:])
        self.assertEqual((pics, rest), ([c], b""))
        self.assertEqual(uc.split_jpegs(b"junk"), ([], b""))


class ServerTests(unittest.TestCase):
    def test_opened_only_while_watched(self) -> None:
        cam = uc.Camera("zed", FAKE)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), uc.make_handler({"zed": cam}))
        srv.daemon_threads = True
        import threading

        threading.Thread(target=srv.serve_forever, daemon=True).start()
        base = f"http://127.0.0.1:{srv.server_address[1]}"
        stats = lambda: json.loads(urllib.request.urlopen(base + "/stats", timeout=5).read())["zed"]  # noqa: E731
        try:
            self.assertEqual(stats()["starts"], 0)  # nobody watching: the camera was never opened
            r = urllib.request.urlopen(base + "/stream/zed", timeout=5)
            got = b""
            while got.count(b"Content-Length") < 2:
                got += r.read1(4096)
            self.assertIn(b"\xff\xd8pic", got)
            self.assertEqual(stats()["starts"], 1)
            r.close()
            end = time.monotonic() + uc.GRACE_S + 6
            while time.monotonic() < end and "idle" not in stats()["note"]:
                time.sleep(0.2)
            self.assertIn("idle", stats()["note"])  # let go after the last watcher left
            self.assertEqual(stats()["clients"], 0)
        finally:
            srv.shutdown()
            srv.server_close()


if __name__ == "__main__":
    unittest.main()
