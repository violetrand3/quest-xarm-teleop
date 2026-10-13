"""Episode recording for teleop: robot rows + camera video, one directory per episode. NO HARDWARE ACCESS.

Lives in the teleop program, on the machine that talks to the arm (the "server"; it may also be the
operator's own PC). It never talks to the arm or the gripper itself: it is handed every record the
teleop adapter writes (Recorder tap) plus a small snapshot of what the program already read in that
tick. Cameras are saved by a child process (episode_cameras.py), so camera work cannot delay the
control loop. Robot rows and camera frames carry the same host monotonic clock.

Operator controls (right controller, thumb):
    A   start an episode / stop and SAVE it
    B   DISCARD the episode being recorded
A press counts only after the button was seen released; a missing value is never a release.
request("toggle" | "start" | "stop" | "discard") does the same from the program (signal, tests).

Layout:
    <session>/session.json                        what, where, units, settings, clock reference
    <session>/episode_0001/meta.json              times, counts, how it ended, camera statistics, problems
    <session>/episode_0001/steps.jsonl            one row per control tick (see FIELDS)
    <session>/episode_0001/events.jsonl           clutch transitions, faults, gripper commands during the episode
    <session>/episode_0001/cam_<name>.mjpeg       frames as sent by the camera
    <session>/episode_0001/cam_<name>.frames.jsonl  per-frame time and byte offset
While it is being written the directory is called episode_0001.recording; a discarded episode is deleted.

on_record() only puts the record on a queue; one writer thread does all file work.
"""

from __future__ import annotations

import json
import math
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, TextIO

from episode_cameras import URL_RE

FORMAT = "quest-xarm-episode-v1"
EVENT_KINDS = frozenset({"transition", "fault", "stop", "ack", "gripper_command", "gripper_link"})
BUTTON_REARM_NS = 400_000_000
FIELDS = {
    "t": "host monotonic clock, ns, at the start of the control tick; the same clock as the camera frame times",
    "state": "clutch state; ACTIVE = the arm is following the hand",
    "tcp_mm": "measured tool position x, y, z in the arm's base frame, millimetres, read at the start of this tick",
    "tcp_rpy": "measured tool orientation roll, pitch, yaw, radians, as the controller reports them: R = Rz(yaw) * Ry(pitch) * Rx(roll), base frame",
    "tcp_q": "the same orientation as a unit quaternion x, y, z, w",
    "joints": "measured joint angles J1..J6, radians",
    "arm_state": "controller state code (1 moving, 2 ready, 4 stopped)",
    "fb_t": "monotonic ns when this tick's reads from the arm completed",
    "fb_read_ms": "how long those reads took",
    "cmd_mm": "target position sent to the controller in this tick, after the box clamp and the speed limit; null = no target sent",
    "cmd_rpy": "target orientation sent with it, radians, same convention as tcp_rpy",
    "grip_pct": "measured jaw position, percent closed (0 = calibrated open, 100 = calibrated closed; calibration in session.json)",
    "grip_ticks": "the same in servo ticks",
    "grip_load": "servo load as the gripper driver reports it",
    "grip_t": "monotonic ns when the gripper state was read",
    "grip_cmd_pct": "jaw target proposed in this tick, percent closed; null = none",
    "trig": "index trigger 0..1 (drives the jaws)",
    "clutch": "side grip 0..1 (middle finger; held = arm follows)",
    "hand_mm": "right controller position in the headset's LOCAL space, millimetres",
    "hand_q": "right controller orientation, quaternion x, y, z, w, LOCAL space",
    "input_age_ms": "age of the newest controller sample at this tick",
    "dt_ms": "time since the previous tick",
}


def free_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / 1e9


class EpisodeRecorder:
    def __init__(self, session_dir: Path, session_meta: dict, cams: dict[str, str], extras: Callable[[], dict] | None = None,
                 min_free_gb: float = 5.0, max_episode_s: float = 300.0, clock: Callable[[], int] = time.monotonic_ns,
                 log: Callable[[str], None] = print):  # fmt: skip
        if session_dir.exists() and any(session_dir.iterdir()):
            raise ValueError(f"{session_dir} already exists and is not empty")
        for name, url in cams.items():
            if not re.fullmatch(r"[a-z0-9_]{1,24}", name) or not URL_RE.fullmatch(url):
                raise ValueError(f"bad camera {name}={url}: need NAME=http://127.0.0.1:<port>/<path>")
        if not cams:
            raise ValueError("episode recording needs at least one camera")
        session_dir.mkdir(parents=True, exist_ok=True)
        self.dir, self.cams, self.extras, self.clock, self.log = session_dir, dict(cams), extras, clock, log
        self.min_free_gb, self.max_episode_ns = float(min_free_gb), int(max_episode_s * 1e9)
        self.saved = self.discarded = self.refused = 0
        self.last_refusal: str | None = None
        self.error: str | None = None
        self.episodes: list = []
        self.q: "queue.SimpleQueue" = queue.SimpleQueue()
        self._cam_ev: "queue.Queue" = queue.Queue()
        self._cam_status: dict | None = None
        self._cam_status_at = 0.0
        self._btn: dict = {}
        self._btn_t: dict = {}
        self._ep: dict | None = None
        self._finishing: int | None = None  # episode number being saved or discarded right now
        self._ticks = 0
        mono, wall = clock(), time.time_ns()
        with (session_dir / "session.json").open("x", encoding="utf-8") as f:
            json.dump({"format": FORMAT, "created_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                       "clock": {"note": "all times are this host's monotonic clock; wall = wall_ns + (t - mono_ns)", "mono_ns": mono, "wall_ns": wall},
                       "cameras": self.cams, "camera_time": "arrival of the frame on this host; the camera supplies no capture time",
                       "steps_fields": FIELDS, **session_meta}, f, indent=1, default=str)  # fmt: skip
        self._err = (session_dir / "cameras.stderr.txt").open("x", encoding="utf-8")
        cmd = [sys.executable, "-u", str(Path(__file__).with_name("episode_cameras.py"))]
        for name, url in self.cams.items():
            cmd += ["--cam", f"{name}={url}"]
        self.child = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._err, text=True, bufsize=1)
        threading.Thread(target=self._read_child, name="episode-cam-events", daemon=True).start()
        self._writer = threading.Thread(target=self._run, name="episode-writer", daemon=True)
        self._writer.start()

    # -- called from the teleop program (never blocks) ------------------------------------------------
    def on_record(self, rec: dict) -> None:
        try:
            kind = rec.get("k")
            if kind == "tick":
                self.q.put((rec, self.extras() if self.extras is not None else None))
            elif kind in EVENT_KINDS:
                self.q.put((rec, None))
        except Exception as e:  # recording must never disturb the control loop
            self.error = f"tap: {type(e).__name__}: {e}"[:160]

    def request(self, what: str) -> None:
        self.q.put(({"k": "request", "t": self.clock(), "what": what}, None))

    def status(self) -> str:
        ep, tail = self._ep, f"saved={self.saved} disc={self.discarded} ref={self.refused}"
        if self.error:
            return f"error {tail}"
        if ep is not None:
            return f"rec#{ep['n']}:{(self.clock() - ep['t0']) / 1e9:.0f}s {tail}"
        if self._finishing is not None:  # files are still being closed: not idle yet, and not counted as saved yet
            return f"saving#{self._finishing} {tail}"
        return ("idle " if self._cams_problem() is None else "nocam ") + tail

    # -- camera child ------------------------------------------------------------------------------------
    def _read_child(self) -> None:
        assert self.child.stdout is not None
        for line in self.child.stdout:
            try:
                ev = json.loads(line)
            except ValueError:
                continue
            if ev.get("ev") == "status":
                self._cam_status, self._cam_status_at = ev, time.monotonic()
            else:
                self._cam_ev.put(ev)

    def _cams_problem(self) -> str | None:
        if self.child.poll() is not None:
            return "the camera recorder is not running"
        st = self._cam_status
        if st is None or time.monotonic() - self._cam_status_at > 3.0:
            return "no status from the camera recorder"
        for name, c in st["cams"].items():
            if c.get("age_ms") is None or c["age_ms"] > 1000:
                return f"camera {name} is not delivering frames ({c.get('error') or 'no frame yet'})"
        return None

    def _tell(self, **cmd: Any) -> bool:
        try:
            assert self.child.stdin is not None
            self.child.stdin.write(json.dumps(cmd, separators=(",", ":")) + "\n")
            self.child.stdin.flush()
            return True
        except (OSError, ValueError):
            return False

    def _await(self, kind: str, timeout_s: float) -> dict | None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            try:
                ev = self._cam_ev.get(timeout=0.05)
            except queue.Empty:
                if self.child.poll() is not None:
                    return None
                continue
            if ev.get("ev") == kind:
                return ev
            if ev.get("ev") == "error":
                self.log(f"episode cameras: {ev.get('what')}: {ev.get('error')}")
        return None

    # -- writer thread ---------------------------------------------------------------------------------------
    def _run(self) -> None:
        while True:
            rec, extra = self.q.get()
            if rec is None:
                return
            try:
                self._handle(rec, extra)
            except Exception as e:
                self.error = f"{type(e).__name__}: {e}"[:160]
                self.log(f"EPISODE RECORDING STOPPED WORKING: {self.error}")
                if self._ep is not None:  # leave what was written, marked unfinished by its directory name
                    for f in (self._ep["steps_f"], self._ep["events_f"]):
                        try:
                            f.close()
                        except Exception:
                            pass
                    self._tell(cmd="stop", t1=self.clock())
                    self._ep = None
                self._finishing = None

    def _edge(self, name: str, v: Any, t: int) -> bool:
        prev, self._btn[name] = self._btn.get(name), v if isinstance(v, bool) else None
        if v is True and prev is False and t - self._btn_t.get(name, -BUTTON_REARM_NS) >= BUTTON_REARM_NS:
            self._btn_t[name] = t
            return True
        return False

    def _handle(self, rec: dict, extra: dict | None) -> None:
        kind, t = rec["k"], rec["t"]
        if self.error:
            return
        if kind == "request":
            what = rec["what"]
            if what in ("start", "toggle") and self._ep is None:
                self._start(t)
            elif what in ("stop", "toggle") and self._ep is not None:
                self._stop(t, "operator")
            elif what == "discard" and self._ep is not None:
                self._discard()
            elif what == "end" and self._ep is not None:
                self._stop(t, "session_end")
            return
        if kind != "tick":
            if self._ep is not None:
                self._ep["events_f"].write(json.dumps(rec, separators=(",", ":"), default=str) + "\n")
                if kind == "fault":
                    self._ep["faults"].append(str(rec.get("reason"))[:120])
                elif kind == "transition" and rec.get("new") == "ACTIVE":
                    self._ep["engagements"] += 1
            return
        a, b = self._edge("a", rec.get("btn_a"), t), self._edge("b", rec.get("btn_b"), t)
        if b and self._ep is not None:
            self._discard()
        elif a and self._ep is None:
            self._start(t)
        elif a:
            self._stop(t, "operator")
        ep = self._ep
        if ep is None:
            return
        row = self._row(rec, extra)
        ep["steps_f"].write(json.dumps(row, separators=(",", ":")) + "\n")
        ep["steps"] += 1
        if row["state"] == "ACTIVE":
            ep["active_ns"] += int((rec.get("dt_ms") or 0.0) * 1e6)
        p = row["tcp_mm"]
        if p is not None:
            if ep["last_p"] is not None:
                ep["travel_mm"] += math.dist(p, ep["last_p"])
            ep["last_p"] = p
        else:
            ep["rows_without_pose"] += 1
        g = row["grip_pct"]
        if g is not None:
            ep["grip_min"], ep["grip_max"] = min(ep["grip_min"], g), max(ep["grip_max"], g)
        self._ticks += 1
        if t - ep["t0"] >= self.max_episode_ns:
            self._stop(t, "max_duration")
        elif self._ticks % 250 == 0 and free_gb(self.dir) < self.min_free_gb - 1.0:
            self._stop(t, "low_disk")

    @staticmethod
    def _row(rec: dict, extra: dict | None) -> dict:
        have_fb = rec.get("fb_mm") is not None
        arm = ((extra or {}).get("arm") or {}) if have_fb else {}  # a failed read must not be filled with the previous tick's values
        g = (extra or {}).get("gripper") or {}
        pose = arm.get("pose")
        ok = bool(g.get("connected"))
        return {
            "t": rec["t"], "state": rec.get("state"),
            "tcp_mm": [round(v, 3) for v in pose[:3]] if pose else rec.get("fb_mm"),
            "tcp_rpy": [round(v, 6) for v in pose[3:6]] if pose else None, "tcp_q": rec.get("fb_q"),
            "joints": None if arm.get("joints") is None else [round(v, 6) for v in arm["joints"]],
            "arm_state": arm.get("state"), "fb_t": arm.get("t"), "fb_read_ms": arm.get("read_ms"),
            "cmd_mm": rec.get("cmd_mm"), "cmd_rpy": None if rec.get("cmd_ori") is None else [round(v, 6) for v in rec["cmd_ori"]],
            "grip_pct": round(g["pct"], 2) if ok and g.get("pct") is not None else rec.get("gripper_fb_pct"),
            "grip_ticks": g.get("ticks") if ok else None, "grip_load": g.get("load") if ok else None, "grip_t": g.get("read_ns") if ok else None,
            "grip_cmd_pct": rec.get("gripper_pct"), "trig": rec.get("trig"), "clutch": rec.get("grip"),
            "hand_mm": rec.get("ctrl_mm"), "hand_q": rec.get("ctrl_q"), "input_age_ms": rec.get("input_age_ms"), "dt_ms": rec.get("dt_ms"),
        }  # fmt: skip

    def _next_number(self) -> int:
        nums = [int(m.group(1)) for p in self.dir.iterdir() if (m := re.fullmatch(r"episode_(\d{4})(\.recording)?", p.name))]
        return max(nums, default=0) + 1

    def _start(self, t: int) -> None:
        why = self._cams_problem()
        if why is None and free_gb(self.dir) < self.min_free_gb:
            why = f"only {free_gb(self.dir):.1f} GB free on the disk (limit {self.min_free_gb:g} GB)"
        if why is not None:
            self.refused, self.last_refusal = self.refused + 1, why
            self.log(f"episode NOT started: {why}")
            return
        n = self._next_number()
        d = self.dir / f"episode_{n:04d}.recording"
        d.mkdir()
        while not self._cam_ev.empty():
            self._cam_ev.get_nowait()
        if not self._tell(cmd="start", dir=str(d.resolve()), t0=t) or self._await("started", 2.0) is None:
            shutil.rmtree(d, ignore_errors=True)
            self.refused, self.last_refusal = self.refused + 1, "the camera recorder did not start the episode"
            self.log(f"episode NOT started: {self.last_refusal}")
            return
        steps: TextIO = (d / "steps.jsonl").open("x", encoding="utf-8")
        events: TextIO = (d / "events.jsonl").open("x", encoding="utf-8")
        self._ep = {"n": n, "dir": d, "t0": t, "steps_f": steps, "events_f": events, "steps": 0, "active_ns": 0, "engagements": 0,
                    "faults": [], "travel_mm": 0.0, "last_p": None, "rows_without_pose": 0, "grip_min": math.inf, "grip_max": -math.inf}  # fmt: skip
        self.log(f"episode {n} started")

    def _close_files(self) -> dict:
        ep = self._ep
        assert ep is not None
        self._finishing, self._ep = ep["n"], None
        ep["steps_f"].close()
        ep["events_f"].close()
        return ep

    def _stop(self, t: int, closed_by: str) -> None:
        ep = self._close_files()
        cam = self._await("stopped", 3.0) if self._tell(cmd="stop", t1=t) else None
        problems = []
        if cam is None:
            problems.append("no camera summary: the camera recorder did not answer")
        else:
            problems += [f"camera {k}: no frames" for k, v in cam["cams"].items() if not v["frames"]]
            problems += [f"camera {k}: gap of {v['max_gap_ms']:.0f} ms" for k, v in cam["cams"].items() if v["max_gap_ms"] > 200]
        if ep["rows_without_pose"]:
            problems.append(f"{ep['rows_without_pose']} rows without an arm pose")
        dur = (t - ep["t0"]) / 1e9
        meta = {"format": FORMAT, "episode": ep["n"], "outcome": "saved", "closed_by": closed_by, "t0": ep["t0"], "t1": t,
                "duration_s": round(dur, 3), "steps": ep["steps"], "step_rate_hz": round(ep["steps"] / dur, 2) if dur > 0 else None,
                "arm_following_s": round(ep["active_ns"] / 1e9, 2), "engagements_started": ep["engagements"],
                "tcp_travel_mm": round(ep["travel_mm"], 1),
                "grip_range_pct": None if ep["grip_min"] > ep["grip_max"] else [ep["grip_min"], ep["grip_max"]],
                "faults": ep["faults"], "cameras": None if cam is None else cam["cams"], "problems": problems}  # fmt: skip
        with (ep["dir"] / "meta.json").open("x", encoding="utf-8") as f:
            json.dump(meta, f, indent=1)
        final = self.dir / f"episode_{ep['n']:04d}"
        ep["dir"].rename(final)
        self.saved += 1
        self.episodes.append({k: meta[k] for k in ("episode", "closed_by", "duration_s", "steps", "arm_following_s", "problems")}
                             | {"frames": None if cam is None else {k: v["frames"] for k, v in cam["cams"].items()}})  # fmt: skip
        self._finishing = None
        self.log(f"episode {ep['n']} SAVED: {dur:.1f} s, {ep['steps']} rows, closed by {closed_by}" + (f"; PROBLEMS: {problems}" if problems else ""))

    def _discard(self) -> None:
        ep = self._close_files()
        if self._tell(cmd="discard"):
            self._await("discarded", 2.0)
        shutil.rmtree(ep["dir"], ignore_errors=True)
        self.discarded += 1
        self._finishing = None
        self.log(f"episode {ep['n']} DISCARDED")

    def close(self) -> dict:
        """Saves an episode that is still open (marked closed_by session_end), stops the camera recorder."""
        self.request("end")
        self.q.put((None, None))
        self._writer.join(timeout=8.0)
        self._tell(cmd="quit")
        try:
            self.child.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            self.child.kill()
        self._err.close()
        out = {"dir": str(self.dir), "saved": self.saved, "discarded": self.discarded, "refused": self.refused,
               "last_refusal": self.last_refusal, "error": self.error, "episodes": self.episodes,
               "camera_recorder_exit": self.child.poll(), "free_gb_after": round(free_gb(self.dir), 1)}  # fmt: skip
        with (self.dir / "session_end.json").open("w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        return out
