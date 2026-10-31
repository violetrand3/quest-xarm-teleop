"""Teleop adapter core: configuration, the scheduler tick, the independent monitor, recording.

One tick (called at a fixed period on the lab host, on its monotonic clock, independent of
network reads):

  1. take the newest accepted input view (no queue, no backlog, no extrapolation)
  2. read backend feedback
  3. reference notices and identity/continuity checks      -> before any target is computed
  4. controller discontinuity check                         -> before the rate limiter
  5. anchored-translation target, fixed-workspace and displacement checks
  6. velocity/acceleration limiter, target-to-feedback error check
  7. backend.command_target(limited target, orientation captured at engagement)

Any failure in 3-7 disengages: the desired motion is dropped, the references are cleared and
backend.stop() is called. Nothing re-engages by itself.

monitor() runs in its own thread. It does not depend on tick() returning: if the tick is stuck
(for example inside a blocking backend call) it raises a fault and calls backend.stop() itself.

Nothing in this module can construct a hardware client. The backend is injected.
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, TextIO

from teleop_backend import DRY_RUN_BANNER, BackendError, RobotBackend, ToolFeedback
from teleop_input import ACTIVE, ARMED, FAULT_LATCHED, WAIT_RELEASE, Engagement, InputChannel
from teleop_limits import Limits, RateLimiter, WristGuard
from teleop_mapping import AnchoredTranslation, ConfigError, GripperFollower
from teleop_rotation import RotationConfig, RotationFollower

__all__ = ["TeleopConfig", "TeleopAdapter", "Recorder", "load_config", "ACTIVE", "ARMED", "FAULT_LATCHED", "WAIT_RELEASE"]

_SCHEMA = {
    "engagement": {"grip_released_max", "grip_pressed_min", "grip_hold_min", "input_fresh_ms", "feedback_fresh_ms"},
    "mapping": {"label", "verified", "scale", "R_backend_from_local"},
    "limits": {"workspace_min_m", "workspace_max_m", "max_anchor_displacement_m", "max_controller_step_m",
               "max_controller_speed_m_s", "v_max_m_s", "a_max_m_s2", "max_target_feedback_error_m"},
    "scheduler": {"period_ms", "max_tick_dt_ms", "monitor_period_ms", "stall_ms"},
    "gripper": {"pickup_tolerance_pct", "max_rate_pct_s"},
}  # fmt: skip
_OPTIONAL = {"limits": {"workspace_edge"}}  # "fault" (default): leaving the workspace latches a fault; "clamp": hold at the edge
# Optional top-level section "rotation" (absent = the tool orientation is frozen at each engagement, as before).
_ROTATION = {"mode", "scale", "max_angle_deg", "w_max_deg_s", "max_error_deg", "max_step_deg"}
_WRIST = {"j5_zone_deg", "j46_zone_deg", "j46_stop_deg"}  # optional top-level section "wrist": teleop_limits.WristGuard
_ROTATION_TILT_TURN = {"tilt_max_deg", "turn_max_deg"}  # optional, together: they replace max_angle_deg (teleop_rotation.limit_tilt_turn)


@dataclass(frozen=True)
class TeleopConfig:
    label: str
    approved_for_live: bool
    raw: dict
    grip_released_max: float
    grip_pressed_min: float
    grip_hold_min: float
    input_fresh_ns: int
    feedback_fresh_ns: int
    mapping: AnchoredTranslation
    mapping_label: str
    mapping_verified: bool
    limits: Limits
    period_ns: int
    max_tick_dt_ns: int
    monitor_period_ns: int
    stall_ns: int
    pickup_tolerance_pct: float
    gripper_max_rate_pct_s: float
    rotation: RotationConfig | None = None
    wrist: tuple | None = None  # (j5_zone_deg, j46_zone_deg, j46_stop_deg) for teleop_limits.WristGuard; None = no guard


def _parse_rotation(r: Any) -> RotationConfig:
    if not isinstance(r, dict) or set(r) - _ROTATION_TILT_TURN != _ROTATION or r["mode"] not in ("yaw", "full"):
        raise ConfigError(f"rotation must have exactly: {', '.join(sorted(_ROTATION))}, with mode 'yaw' or 'full'")
    tt = _ROTATION_TILT_TURN & set(r)
    if tt and tt != _ROTATION_TILT_TURN:
        raise ConfigError("rotation.tilt_max_deg and rotation.turn_max_deg go together")
    for k in (_ROTATION | tt) - {"mode"}:
        if isinstance(r[k], bool) or not isinstance(r[k], (int, float)) or not r[k] > 0:
            raise ConfigError(f"rotation.{k} must be a positive number")
    if r["scale"] > 2.0 or r["max_angle_deg"] > 120.0:
        raise ConfigError("rotation.scale must be <= 2 and rotation.max_angle_deg <= 120")
    if tt and (r["tilt_max_deg"] > 90.0 or r["turn_max_deg"] > 180.0):
        raise ConfigError("rotation.tilt_max_deg must be <= 90 and rotation.turn_max_deg <= 180")
    rad = math.radians
    return RotationConfig(r["mode"], float(r["scale"]), rad(r["max_angle_deg"]), rad(r["w_max_deg_s"]), rad(r["max_error_deg"]),
                          rad(r["max_step_deg"]), *((rad(r["tilt_max_deg"]), rad(r["turn_max_deg"])) if tt else ()))  # fmt: skip


def parse_config(d: Any) -> TeleopConfig:
    if not isinstance(d, dict) or set(d) - {"rotation", "wrist"} != {"label", "approved_for_live", *_SCHEMA}:
        raise ConfigError(f"config must have exactly: label, approved_for_live, {', '.join(sorted(_SCHEMA))} (and optionally rotation, wrist)")
    w = d.get("wrist")
    if w is not None and (not isinstance(w, dict) or set(w) != _WRIST or not all(
            isinstance(w[k], (int, float)) and not isinstance(w[k], bool) and 0 < w[k] <= 90 for k in _WRIST) or w["j46_stop_deg"] >= w["j46_zone_deg"]):
        raise ConfigError(f"wrist must have exactly: {', '.join(sorted(_WRIST))}, each 0..90 deg, with j46_stop_deg < j46_zone_deg")
    for sec, keys in _SCHEMA.items():
        if not isinstance(d[sec], dict) or not (keys <= set(d[sec]) <= keys | _OPTIONAL.get(sec, set())):
            raise ConfigError(f"config section {sec!r} must have exactly: {', '.join(sorted(keys))}")
    edge = d["limits"].get("workspace_edge", "fault")
    if edge not in ("fault", "clamp"):
        raise ConfigError("limits.workspace_edge must be 'fault' or 'clamp'")
    if not isinstance(d["approved_for_live"], bool) or not isinstance(d["mapping"]["verified"], bool):
        raise ConfigError("approved_for_live and mapping.verified must be booleans")

    def pos(sec: str, key: str) -> float:
        v = d[sec][key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not v > 0:
            raise ConfigError(f"{sec}.{key} must be a positive number")
        return float(v)

    def vec(key: str) -> tuple:
        v = d["limits"][key]
        if not (isinstance(v, list) and len(v) == 3 and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)):
            raise ConfigError(f"limits.{key} must be [x, y, z] in metres")
        return tuple(float(x) for x in v)

    lo, hi = vec("workspace_min_m"), vec("workspace_max_m")
    if any(lo[k] >= hi[k] for k in range(3)):
        raise ConfigError("workspace_min_m must be below workspace_max_m on every axis")
    e = d["engagement"]
    for k in ("grip_released_max", "grip_pressed_min", "grip_hold_min"):
        if isinstance(e[k], bool) or not isinstance(e[k], (int, float)):
            raise ConfigError(f"engagement.{k} must be a number")
    if not (0.0 <= e["grip_released_max"] < e["grip_hold_min"] <= e["grip_pressed_min"] <= 1.0):
        raise ConfigError("need 0 <= grip_released_max < grip_hold_min <= grip_pressed_min <= 1")
    limits = Limits(lo, hi, pos("limits", "max_anchor_displacement_m"), pos("limits", "max_controller_step_m"),
                    pos("limits", "max_controller_speed_m_s"), pos("limits", "v_max_m_s"), pos("limits", "a_max_m_s2"),
                    pos("limits", "max_target_feedback_error_m"), clamp_to_workspace=edge == "clamp")  # fmt: skip
    ms = lambda sec, key: int(pos(sec, key) * 1e6)  # noqa: E731
    return TeleopConfig(
        str(d["label"]), d["approved_for_live"], d, float(e["grip_released_max"]), float(e["grip_pressed_min"]),
        float(e["grip_hold_min"]), ms("engagement", "input_fresh_ms"), ms("engagement", "feedback_fresh_ms"),
        AnchoredTranslation(d["mapping"]["scale"], d["mapping"]["R_backend_from_local"]), str(d["mapping"]["label"]),
        d["mapping"]["verified"], limits, ms("scheduler", "period_ms"), ms("scheduler", "max_tick_dt_ms"),
        ms("scheduler", "monitor_period_ms"), ms("scheduler", "stall_ms"), pos("gripper", "pickup_tolerance_pct"),
        pos("gripper", "max_rate_pct_s"), _parse_rotation(d["rotation"]) if "rotation" in d else None,
        None if w is None else (float(w["j5_zone_deg"]), float(w["j46_zone_deg"]), float(w["j46_stop_deg"])))  # fmt: skip


def load_config(path: Path) -> TeleopConfig:
    with open(path, encoding="utf-8") as f:
        return parse_config(json.load(f))


class Recorder:
    """Diagnostic JSONL recorder (thread-safe). Every line belongs to a dry run and says so in the header."""

    def __init__(self, stream: TextIO | None = None, tap: Callable[[dict], None] | None = None):
        self._stream, self._lock = stream, threading.Lock()
        self._tap = tap  # optional observer of every record (episode recording); must return at once and never raise
        self.counts: dict = {}
        self.transitions: list = []
        self.faults: list = []
        self.stops: list = []
        self.acks: list = []

    def write(self, kind: str, t_ns: int, **fields: Any) -> None:
        with self._lock:
            self.counts[kind] = self.counts.get(kind, 0) + 1
            rec = dict(k=kind, t=t_ns, **fields)
            keep = {"transition": self.transitions, "fault": self.faults, "stop": self.stops, "ack": self.acks}.get(kind)
            if keep is not None and len(keep) < 2000:
                keep.append(rec)
            if self._stream is not None:
                self._stream.write(json.dumps(rec, separators=(",", ":")) + "\n")
        if self._tap is not None:
            try:
                self._tap(rec)
            except Exception:  # an observer that fails is dropped; it must never reach the control loop
                self._tap = None
                self.counts["tap_failed"] = 1


LIMIT_FAULTS = frozenset({"workspace_violation", "anchor_displacement_violation", "controller_discontinuity",
                          "target_feedback_error", "scheduler_overrun"})  # fmt: skip


def _mm(p: Any) -> Any:
    return None if p is None else [round(v * 1000.0, 2) for v in p]


class TeleopAdapter:
    def __init__(self, cfg: TeleopConfig, channel: InputChannel, backend: RobotBackend, recorder: Recorder | None = None,
                 clock: Callable[[], int] | None = None, gripper_executor: Any = None):  # fmt: skip
        self.cfg, self.channel, self.backend = cfg, channel, backend
        # None (the default, and the only value the dry run can pass) = gripper proposals are previews only.
        # An executor is an injected object with submit(proposal) and cancel(); both must return at once.
        self.gripper_executor = gripper_executor
        self.rec = recorder or Recorder()
        self.eng = Engagement(cfg.grip_released_max, cfg.grip_pressed_min, cfg.grip_hold_min, cfg.input_fresh_ns)
        self.limiter = RateLimiter(cfg.limits.v_max_m_s, cfg.limits.a_max_m_s2)
        self.follower = GripperFollower(cfg.pickup_tolerance_pct, cfg.gripper_max_rate_pct_s)
        self.rot = RotationFollower(cfg.rotation, cfg.mapping.R) if cfg.rotation is not None else None
        self.wrist = WristGuard(*cfg.wrist) if cfg.wrist is not None else None
        self.p_req: tuple | None = None
        self.last: dict = {"state": self.eng.state}  # newest tick record, for display
        self.ticks = 0
        self._hb = threading.Lock()
        self._last_tick_ns: int | None = None  # start time of the previous tick (dt reference)
        self._tick_started_ns: int | None = None  # set while a tick is in progress
        self._tick_finished_ns: int | None = None
        self._ext_fault: str | None = None
        self._ack_requested = False
        self.rec.write("header", 0, banner=getattr(backend, "banner", DRY_RUN_BANNER if backend.synthetic else "NOT A DRY RUN"),
                       gripper_execution=gripper_executor is not None, config=cfg.raw,
                       backend=getattr(backend, "describe", lambda: {"backend": backend.name})(),
                       units="positions in mm in this log; metres internally", initial_state=self.eng.state)  # fmt: skip

    # -- operator acknowledgement (applied by the scheduler, between ticks) -------------------
    def request_acknowledge(self) -> None:
        with self._hb:
            self._ack_requested = True

    # -- helpers --------------------------------------------------------------------------------
    def _transition(self, now: int, old: str, reason: str) -> None:
        if self.eng.state != old or reason.startswith("refused"):
            self.rec.write("transition", now, old=old, new=self.eng.state, reason=reason, latch=list(self.eng.latch_reasons))

    def _stop_backend(self, now: int, reason: str, by: str, latched: bool) -> None:
        err = None
        try:
            self.backend.stop(reason, latched=latched)
        except BackendError as e:
            err = str(e)
        self.rec.write("stop", now, reason=reason, by=by, latched=latched, error=err, synthetic=self.backend.synthetic)
        if err is not None:
            self.eng.latch("stop_failed:" + err[:80])

    def _disengage(self, now: int, reason: str, latch: bool) -> None:
        old = self.eng.state
        self.p_req = None  # the outstanding desired motion is cancelled, not finished
        self.limiter.clear()
        if self.rot is not None:
            self.rot.clear()
        self.follower.release()  # no opening or closing request is made
        if self.gripper_executor is not None:
            self.gripper_executor.cancel()  # a proposal not yet sent is dropped, never sent late
        self.eng.disengage(reason, latch)
        if latch:
            self.rec.write("fault", now, reason=reason, limit=reason.split(":")[0] in LIMIT_FAULTS)
        self._stop_backend(now, reason, "scheduler", latch)
        self._transition(now, old, reason)

    def _feedback(self, now: int) -> tuple:
        try:
            fb = self.backend.read_feedback()
        except BackendError as e:
            return None, "feedback_failed:" + str(e)[:80]
        if now - fb.read_mono_ns > self.cfg.feedback_fresh_ns:
            return None, f"feedback_stale:{(now - fb.read_mono_ns) / 1e6:.0f}ms"
        return fb, None

    # -- the scheduler tick ---------------------------------------------------------------------
    def tick(self, now: int) -> dict:
        cfg, eng = self.cfg, self.eng
        with self._hb:
            dt_ns = None if self._last_tick_ns is None else now - self._last_tick_ns
            self._last_tick_ns = self._tick_started_ns = now
            ext, self._ext_fault = self._ext_fault, None
            ack, self._ack_requested = self._ack_requested, False
        view = self.channel.view(consume=True)
        fb, fb_problem = self._feedback(now)
        old = eng.state
        commanded = limited = cmd_ori = None
        grip_status, proposal, gripper_fb = "idle", None, None

        if ack:
            ok, msg = eng.acknowledge()
            self.rec.write("ack", now, accepted=ok, detail=msg)
            self._transition(now, old, msg)
            old = eng.state
        # A reference change is handled before any pose that came with it; so is a monitor fault.
        notices = list(dict.fromkeys("reference_change_notice:" + n for n in view.ref_notices))
        for why in notices + ([ext] if ext is not None else []):
            if eng.state == ACTIVE:
                self._disengage(now, why, True)
            elif why not in eng.latch_reasons:
                eng.latch(why)
                self.rec.write("fault", now, reason=why, limit=False)
                self._transition(now, old, why)
            old = eng.state

        if eng.state == ACTIVE:
            why, latch = eng.active_problem(view, now)  # identities and continuity first
            if why is None and fb is None:
                why, latch = fb_problem, True
            if why is None and dt_ns is not None and dt_ns > cfg.max_tick_dt_ns:
                why, latch = f"scheduler_overrun:{dt_ns / 1e6:.0f}ms", True
            if why is None:
                s, ref = view.sample, eng.ref
                if s.source_seq != ref.last_seq:  # a new accepted input; otherwise hold the last request
                    why = cfg.limits.controller_step_problem(ref.last_p_m, ref.last_t_query_ns, s.p_m, s.t_query_ns)
                    latch = True
                    if why is None:
                        ref.last_p_m, ref.last_t_query_ns, ref.last_seq = s.p_m, s.t_query_ns, s.source_seq
                        if self.rot is not None:
                            why = self.rot.on_sample(s.q_xyzw)  # a jump in orientation is rejected like a jump in position
                    if why is None:
                        p_req = cfg.mapping.target(ref.p_tool0_m, ref.p_ctrl0_m, s.p_m)
                        if cfg.limits.clamp_to_workspace:
                            p_req = cfg.limits.clamp(p_req)
                        why = cfg.limits.target_problem(p_req, ref.p_tool0_m)
                        if why is None:
                            self.p_req = p_req
            if why is None:
                limited = self.limiter.step(self.p_req, (dt_ns or 0) / 1e9)
                if cfg.limits.clamp_to_workspace:
                    limited = self.limiter.confine(cfg.limits.workspace_min_m, cfg.limits.workspace_max_m)
                why = cfg.limits.workspace_problem(limited, "limited") or cfg.limits.tracking_problem(limited, fb.p_m)
                latch = True
            orientation = None if eng.ref is None else eng.ref.orientation  # frozen at engagement unless rotation is on
            if why is None and self.rot is not None:
                q_lim = self.rot.step((dt_ns or 0) / 1e9)
                why, latch = self.rot.tracking_problem(fb.q_xyzw), True
                if why is None:
                    try:
                        orientation = self.backend.orientation_from_q(q_lim)
                    except BackendError as e:
                        why = "command_failed:" + str(e)[:80]
            send = True
            if why is None and self.wrist is not None:
                q_now = None if self.rot is None else fb.q_xyzw
                why, send = self.wrist.check(fb.joints_deg, (fb.p_m, q_now), limited, None if self.rot is None else self.rot.q_lim)
                latch = True
                if why is None and not send:  # held: the limited targets stay where the arm was last sent
                    self.limiter.reset(self.wrist.last[0])
                    limited = self.limiter.p
                    if self.rot is not None and self.wrist.last[1] is not None:
                        self.rot.q_lim = self.wrist.last[1]
            if why is None and send:
                try:
                    self.backend.command_target(limited, orientation)
                    commanded, cmd_ori = limited, orientation
                except BackendError as e:
                    why, latch = "command_failed:" + str(e)[:80], True
                with self._hb:
                    late, self._ext_fault = self._ext_fault, None
                if why is None and late is not None:  # the monitor fired while the command was in progress
                    why, latch = late, True
            if why is not None:
                self._disengage(now, why, latch)
            else:
                gfb = self.backend.read_gripper()
                gripper_fb = None if gfb is None or not gfb.connected else gfb.pct_closed
                if gripper_fb is None and self.follower.following:
                    self.follower.release()  # gripper feedback lost: stop following; a new pickup is required
                    if self.gripper_executor is not None:
                        self.gripper_executor.cancel()
                grip_status, proposal = self.follower.update(view.sample.trigger, gripper_fb, (dt_ns or 0) / 1e9)
                if proposal is not None:
                    self.backend.preview_gripper(proposal)  # recorded by the backend; never executed there
                    if self.gripper_executor is not None:
                        self.gripper_executor.submit(proposal)  # non-blocking; the executor rate-limits and sends
        else:
            if eng.step_disengaged(view, now) == "press":
                ready, not_ready_why = self.backend.motion_ready()
                if not ready:
                    eng.refuse_engagement("refused: " + not_ready_why)
                    self._transition(now, old, eng.blocked)
                elif fb is None:
                    eng.refuse_engagement("refused: " + str(fb_problem))
                    self._transition(now, old, eng.blocked)
                elif self.rot is not None and (fb.q_xyzw is None or view.sample.q_xyzw is None):
                    eng.refuse_engagement("refused: no orientation from the " + ("backend" if fb.q_xyzw is None else "controller"))
                    self._transition(now, old, eng.blocked)
                elif cfg.limits.workspace_problem(fb.p_m, "tool feedback") is not None:
                    eng.latch("refused: " + cfg.limits.workspace_problem(fb.p_m, "tool feedback"))  # type: ignore[operator]
                    self.rec.write("fault", now, reason=eng.latch_reasons[-1], limit=True)
                    self._transition(now, old, eng.latch_reasons[-1])
                else:
                    eng.engage(view, fb.p_m, fb.orientation, now)
                    if self.rot is not None:
                        self.rot.engage(view.sample.q_xyzw, fb.q_xyzw)  # rotation target starts AT the tool's orientation
                    self.limiter.reset(fb.p_m)  # limited target starts AT the tool: zero jump
                    if self.wrist is not None:
                        self.wrist.last = (tuple(fb.p_m), fb.q_xyzw if self.rot is not None else None)
                    self.p_req = tuple(fb.p_m)
                    self._transition(now, old, "valid_press_after_valid_release")
                    try:
                        self.backend.command_target(self.p_req, eng.ref.orientation)
                        commanded = limited = self.p_req
                        cmd_ori = eng.ref.orientation
                    except BackendError as e:
                        self._disengage(now, "command_failed:" + str(e)[:80], True)
            elif eng.state != old:
                self._transition(now, old, eng.blocked if eng.state != ARMED else "valid_release_observed")

        s = view.sample
        out = {
            "state": eng.state, "blocked": eng.blocked if eng.state != ACTIVE else "", "latch": list(eng.latch_reasons),
            "connected": view.connected, "source_up": view.source_up, "source_stale": view.source_stale,
            "gen": view.continuity_gen, "src": None if s is None else [s.identity[1], s.identity[3]],
            "seq": None if s is None else s.source_seq, "avail": None if s is None else s.available,
            "reasons": None if s is None else list(s.reasons),
            "input_age_ms": None if s is None else round((now - s.arrival_ns) / 1e6, 1),
            "grip": None if s is None else s.grip, "trig": None if s is None else s.trigger,
            "ctrl_mm": None if s is None else _mm(s.p_m), "ctrl_q": None if s is None or s.q_xyzw is None else list(s.q_xyzw),
            "req_mm": _mm(self.p_req), "lim_mm": _mm(limited), "cmd_mm": _mm(commanded),
            "fb_mm": None if fb is None else _mm(fb.p_m), "fb_problem": fb_problem,
            "ori": None if eng.ref is None else list(eng.ref.orientation),
            "cmd_ori": None if cmd_ori is None else list(cmd_ori),  # orientation sent with cmd_mm in this tick
            "btn_a": None if s is None else s.a, "btn_b": None if s is None else s.b,
            "wrist": None if self.wrist is None or eng.state != ACTIVE else self.wrist.note,
            "rot_deg": None if self.rot is None else self.rot.angles_deg()[0], "rot_req_deg": None if self.rot is None else self.rot.angles_deg()[1],
            "fb_q": None if fb is None or fb.q_xyzw is None else [round(v, 6) for v in fb.q_xyzw],
            "gripper": grip_status, "gripper_pct": None if proposal is None else proposal.pct_closed,
            "gripper_fb_pct": None if gripper_fb is None else round(gripper_fb, 2),
            "dt_ms": None if dt_ns is None else round(dt_ns / 1e6, 2), "synthetic": self.backend.synthetic,
        }  # fmt: skip
        self.rec.write("tick", now, **out)
        self.last, self.ticks = out, self.ticks + 1
        with self._hb:
            self._tick_started_ns, self._tick_finished_ns = None, now
        return out

    # -- independent monitor ----------------------------------------------------------------------
    def monitor(self, now: int) -> str | None:
        """Runs in its own thread. A scheduler that is stuck or not running for longer than stall_ms is
        itself the fault: the monitor requests a stop without waiting for tick() to return."""
        with self._hb:
            started, last = self._tick_started_ns, self._last_tick_ns
            pending = self._ext_fault
        why = None
        if started is not None and now - started > self.cfg.stall_ns:
            why = f"monitor:scheduler_blocked_in_tick:{(now - started) / 1e6:.0f}ms"
        elif last is not None and started is None and now - last > self.cfg.stall_ns:
            why = f"monitor:scheduler_not_running:{(now - last) / 1e6:.0f}ms"
        if why is None or pending is not None:
            return None
        with self._hb:
            self._ext_fault = why
        self.rec.write("fault", now, reason=why, limit=False)
        self._stop_backend(now, why, "monitor", True)  # does not wait for the scheduler
        return why
