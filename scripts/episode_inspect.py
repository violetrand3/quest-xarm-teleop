#!/usr/bin/env python3
"""Checks recorded episodes and, on request, derives action labels from the arm poses. Reads files only.

    episode_inspect.py <session or episode directory>... [--labels K] [--ref-cam iphone]

For each episode: row rate and gaps, rows without a pose, per-camera frame count, rate, largest gap,
index/file consistency (every indexed frame is a whole JPEG), and whether the video covers the rows.

--labels K writes labels_k<K>.json into the episode: one entry per frame of the reference camera, with
the arm pose and jaw aperture interpolated to the frame's time, and for frame i the action

    inv(T_i) @ T_(i+K)   ->   translation (metres) and rotation vector (radians), expressed in the TOOL frame at i
    aperture at i+K            0 = closed, 1 = open (from the measured jaw position, absolute)

which is how the lab's flowbc pipeline builds its labels (ACTION_FRAME_CONTRACT.md), EXCEPT that the
frame here is the arm's tool (flange) frame, not the wrist camera's optical frame. Turning one into
the other needs the camera mounting transform, which is not known here.
"""

from __future__ import annotations

import argparse
import bisect
import json
import sys
from pathlib import Path

from teleop_rotation import q_conj, q_mul, q_norm, q_to_rotvec


def rows(path: Path) -> list:
    with path.open(encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def q_rotate(q: tuple, v: tuple) -> tuple:
    return q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))[:3]


def relative_action(p_i: tuple, q_i: tuple, p_j: tuple, q_j: tuple) -> list:
    """inv(T_i) @ T_j as [dx, dy, dz (m), rx, ry, rz (rad)] in the frame of pose i. Positions in metres."""
    d = q_rotate(q_conj(q_i), tuple(p_j[k] - p_i[k] for k in range(3)))
    return [*d, *q_to_rotvec(q_mul(q_conj(q_i), q_j))]


def interpolate(steps: list, ts: list, t: int) -> dict | None:
    """Pose and jaw position at time t from the two rows around it. None outside the rows or across a missing pose."""
    j = bisect.bisect_right(ts, t)
    if j == 0 or j == len(ts):
        return None
    a, b = steps[j - 1], steps[j]
    if a["tcp_mm"] is None or b["tcp_mm"] is None or a["tcp_q"] is None or b["tcp_q"] is None:
        return None
    w = (t - a["t"]) / (b["t"] - a["t"])
    qa, qb = a["tcp_q"], b["tcp_q"]
    if sum(x * y for x, y in zip(qa, qb)) < 0.0:
        qb = [-v for v in qb]
    lerp = lambda u, v: None if u is None or v is None else u + (v - u) * w  # noqa: E731
    return {"p_m": tuple((a["tcp_mm"][k] + (b["tcp_mm"][k] - a["tcp_mm"][k]) * w) / 1000.0 for k in range(3)),
            "q": q_norm(tuple(qa[k] + (qb[k] - qa[k]) * w for k in range(4))),  # rows are 20 ms apart: the chord is the arc
            "grip_pct": lerp(a["grip_pct"], b["grip_pct"])}  # fmt: skip


def check(ep: Path) -> tuple:
    meta = json.loads((ep / "meta.json").read_text())
    steps = rows(ep / "steps.jsonl")
    problems, ts = [], [r["t"] for r in steps]
    if any(v <= u for u, v in zip(ts, ts[1:])):
        problems.append("row times are not increasing")
    dts = [(v - u) / 1e6 for u, v in zip(ts, ts[1:])]
    no_pose = sum(r["tcp_mm"] is None or r["tcp_q"] is None for r in steps)
    if no_pose:
        problems.append(f"{no_pose} rows without a pose")
    if dts and max(dts) > 60:
        problems.append(f"gap of {max(dts):.0f} ms between rows")
    out = {"episode": ep.name, "duration_s": meta["duration_s"], "rows": len(steps), "row_hz": meta["step_rate_hz"],
           "max_row_gap_ms": round(max(dts), 1) if dts else None, "arm_following_s": meta["arm_following_s"],
           "tcp_travel_mm": meta["tcp_travel_mm"], "joints": sum(r["joints"] is not None for r in steps),
           "targets": sum(r["cmd_mm"] is not None for r in steps), "grip_range_pct": meta["grip_range_pct"],
           "closed_by": meta["closed_by"], "cameras": {}}  # fmt: skip
    for idx in sorted(ep.glob("cam_*.frames.jsonl")):
        name = idx.name[4:-len(".frames.jsonl")]
        fr, video = rows(idx), ep / f"cam_{name}.mjpeg"
        size = video.stat().st_size
        if any(b["off"] != a["off"] + a["len"] for a, b in zip(fr, fr[1:])) or (fr and fr[-1]["off"] + fr[-1]["len"] != size):
            problems.append(f"camera {name}: index does not match the video file")
        with video.open("rb") as f:
            for r in fr[:: max(1, len(fr) // 50)] + fr[-1:]:
                f.seek(r["off"])
                d = f.read(r["len"])
                if d[:2] != b"\xff\xd8" or d[-2:] != b"\xff\xd9":
                    problems.append(f"camera {name}: frame {r['i']} is not a whole JPEG")
                    break
        gaps = [(b["t"] - a["t"]) / 1e6 for a, b in zip(fr, fr[1:])]
        span = (fr[-1]["t"] - fr[0]["t"]) / 1e9 if len(fr) > 1 else 0.0
        out["cameras"][name] = {"frames": len(fr), "fps": round((len(fr) - 1) / span, 2) if span else None,
                                "max_gap_ms": round(max(gaps), 1) if gaps else None, "MB": round(size / 1e6, 1)}  # fmt: skip
        if not fr:
            problems.append(f"camera {name}: no frames")
            continue
        if gaps and max(gaps) > 200:
            problems.append(f"camera {name}: gap of {max(gaps):.0f} ms")
        if ts and (fr[0]["t"] - ts[0] > 300e6 or ts[-1] - fr[-1]["t"] > 300e6):
            problems.append(f"camera {name}: video does not cover the rows (starts {(fr[0]['t'] - ts[0]) / 1e6:+.0f} ms, ends {(fr[-1]['t'] - ts[-1]) / 1e6:+.0f} ms)")
    if not out["cameras"]:
        problems.append("no camera files")
    return out, problems, steps, ts


def labels(ep: Path, steps: list, ts: list, k: int, ref_cam: str) -> dict:
    frames = rows(ep / f"cam_{ref_cam}.frames.jsonl")
    states = [(f["i"], f["t"], interpolate(steps, ts, f["t"])) for f in frames]
    states = [s for s in states if s[2] is not None]
    aperture = lambda s: None if s["grip_pct"] is None else round(min(1.0, max(0.0, 1.0 - s["grip_pct"] / 100.0)), 4)  # noqa: E731
    out = {"format": "quest-xarm-labels-v1", "k": k, "ref_cam": ref_cam, "n": len(states),
           "frame": "arm tool (flange) frame; NOT the wrist camera's optical frame of the lab's flowbc contract",
           "action": "inv(T_i) @ T_(i+k): [dx, dy, dz in m, rx, ry, rz rotation vector in rad, aperture at i+k (0 closed .. 1 open)]",
           "frames": [{"i": i, "t": t, "xyz_m": [round(v, 6) for v in s["p_m"]], "quat_xyzw": [round(v, 6) for v in s["q"]],
                       "aperture": aperture(s)} for i, t, s in states],
           "actions": [[round(v, 6) for v in relative_action(a[2]["p_m"], a[2]["q"], b[2]["p_m"], b[2]["q"])] + [aperture(b[2])]
                       for a, b in zip(states, states[k:])]}  # fmt: skip
    with (ep / f"labels_k{k}.json").open("w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("paths", nargs="+", type=Path)
    ap.add_argument("--labels", type=int, metavar="K", help="also write labels_k<K>.json (action = pose change over K frames of --ref-cam)")
    ap.add_argument("--ref-cam", default="iphone")
    a = ap.parse_args(argv)
    eps: list = []
    for p in a.paths:
        eps += [p] if (p / "meta.json").exists() else sorted(d for d in p.glob("episode_[0-9][0-9][0-9][0-9]") if (d / "meta.json").exists())
    if not eps or (a.labels is not None and a.labels < 1):
        print("error: no saved episode found (or bad --labels)", file=sys.stderr)
        return 2
    bad = 0
    for ep in eps:
        out, problems, steps, ts = check(ep)
        if a.labels is not None and (ep / f"cam_{a.ref_cam}.frames.jsonl").exists():
            lab = labels(ep, steps, ts, a.labels, a.ref_cam)
            out["labels"] = {"file": f"labels_k{a.labels}.json", "frames": lab["n"], "actions": len(lab["actions"])}
        out["problems"] = problems
        bad += bool(problems)
        print(json.dumps(out))
    print(f"{len(eps)} episode(s), {bad} with problems")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
