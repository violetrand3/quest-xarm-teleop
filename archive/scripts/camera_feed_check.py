#!/usr/bin/env python3
"""Check that existing MJPEG camera feeds deliver UPDATING video. View-only (HTTP GET).

Reads each /cam/<key> stream of the lab's existing camera wall through a local tunnel, for a
few seconds, and records per frame the LOCAL receive time, size and hash. The streams carry no
source timestamps or frame counters, so only local receive times exist: frame rate and frame
intervals say nothing about end-to-end latency.
"""
from __future__ import annotations

import hashlib
import json
import struct
import sys
import time
import urllib.request
from pathlib import Path


def jpeg_size(b: bytes):  # type: ignore[no-untyped-def]
    i = 2
    while i + 9 < len(b) and b[i] == 0xFF:
        marker, length = b[i + 1], struct.unpack(">H", b[i + 2:i + 4])[0]
        if marker in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack(">HH", b[i + 5:i + 9])
            return [w, h]
        i += 2 + length
    return None


def check(base: str, key: str, seconds: float, save: Path | None) -> dict:
    out: dict = {"url": f"<tunnel>/cam/{key}", "requested_s": seconds}
    t_open = time.monotonic()
    try:
        r = urllib.request.urlopen(f"{base}/cam/{key}", timeout=10)
    except Exception as e:
        out["error"] = f"{type(e).__name__}: {e}"
        return out
    out["content_type"] = r.headers.get("Content-Type")
    out["source_timestamp_headers"] = [h for h in r.headers.keys() if h.lower() in ("date", "last-modified", "x-timestamp")]
    frames, part_headers = [], set()
    try:
        while time.monotonic() - t_open < seconds:
            line = r.readline()
            if not line:
                break
            if not line.startswith(b"--frame"):
                continue
            length = None
            while True:
                h = r.readline().strip()
                if not h:
                    break
                part_headers.add(h.split(b":")[0].decode().lower())
                if h.lower().startswith(b"content-length"):
                    length = int(h.split(b":")[1])
            if length is None:
                break
            data = r.read(length)
            frames.append((time.monotonic(), len(data), hashlib.md5(data).hexdigest(), jpeg_size(data), data if save else None))
    except Exception as e:
        out["stream_error"] = f"{type(e).__name__}: {e}"
    finally:
        r.close()
    out["per_frame_headers"] = sorted(part_headers)
    out["frames"] = len(frames)
    if not frames:
        return out
    t = [f[0] for f in frames]
    gaps = sorted((b - a) * 1000 for a, b in zip(t, t[1:]))
    sizes = [tuple(f[3]) if f[3] else None for f in frames]
    live = [f for f in frames if f[3] != [640, 360]]  # the service's "connecting..." placeholder is 640x360
    out.update(first_frame_after_open_ms=round((t[0] - t_open) * 1000), span_s=round(t[-1] - t[0], 2),
               distinct_frames=len({f[2] for f in frames}), distinct_live_frames=len({f[2] for f in live}),
               placeholder_frames=len(frames) - len(live), image_sizes=sorted({str(s) for s in sizes}),
               mean_fps=round((len(frames) - 1) / (t[-1] - t[0]), 1) if t[-1] > t[0] else None,
               interval_ms={"median": round(gaps[len(gaps) // 2], 1), "p95": round(gaps[int(0.95 * (len(gaps) - 1))], 1), "max": round(gaps[-1], 1)} if gaps else None,
               frames_in_first_200ms=sum(1 for x in t if x - t[0] <= 0.2),
               updating=len({f[2] for f in live}) >= 5)  # fmt: skip
    if save and live:
        save.write_bytes(live[-1][4])
        out["sample_frame_saved"] = save.name
    return out


def main() -> int:
    base, outdir, seconds = sys.argv[1], Path(sys.argv[2]), float(sys.argv[3])
    res = {"note": "local receive times only; no source capture timestamps or counters are available in these streams",
           "host_wall_time": time.time(), "feeds": {}}  # fmt: skip
    for key in sys.argv[4:]:
        res["feeds"][key] = check(base, key, seconds, outdir / f"sample_{key}.jpg")
        print(key, json.dumps({k: v for k, v in res["feeds"][key].items() if k not in ("url",)}), flush=True)
    (outdir / "camera_feed_check.json").write_text(json.dumps(res, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
