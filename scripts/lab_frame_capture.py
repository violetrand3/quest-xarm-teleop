#!/usr/bin/env python3
"""Save frames from the lab's existing camera wall for a fixed time (view-only GET on loopback).

Runs on the lab host. Opens no camera device and changes no setting. Each frame is saved as
<out>/<key>/<lab monotonic ns>.jpg, at most --fps per second, so that what the cameras saw can be
compared with what the robot or gripper reported.
Usage: lab_frame_capture.py <out_dir> <seconds> <fps> <key> [<key> ...]
"""
import os
import sys
import threading
import time
import urllib.request


def pull(key: str, out: str, seconds: float, fps: float) -> None:
    os.makedirs(os.path.join(out, key), exist_ok=True)
    end, last = time.monotonic() + seconds, 0.0
    try:
        r = urllib.request.urlopen(f"http://127.0.0.1:8090/cam/{key}", timeout=10)
        while time.monotonic() < end:
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
            data = r.read(n)
            if time.monotonic() - last >= 1.0 / fps and len(data) > 9000:  # skip the wall's small placeholder frames
                last = time.monotonic()
                with open(os.path.join(out, key, f"{time.monotonic_ns()}.jpg"), "wb") as f:
                    f.write(data)
        r.close()
    except Exception as e:
        print(key, "capture ended:", e)


if __name__ == "__main__":
    out, seconds, fps, keys = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4:]
    th = [threading.Thread(target=pull, args=(k, out, seconds, fps)) for k in keys]
    [t.start() for t in th]
    [t.join() for t in th]
