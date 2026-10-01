#!/usr/bin/env python3
"""Step 4B Test 2 spoken phase cues (host-side helper; DIAGNOSTIC ONLY).

Reads ONLY the recorder's JSONL output (never the device) to wait until the right
controller is FRESH, focused, action-active and tracked for --ready-s continuous
seconds, then speaks the planned phase cues with the existing `spd-say` and writes
each cue's host time.monotonic_ns() (same clock as the recorder's host_ns) to
<base>.marks.txt. The planned sequence is saved separately as <base>.planned.json.
It never asks the user to touch the controller once cues have started.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import quest_pose_v1 as qp

PHASES = {
    "rotate": [("still_1", 5.0, "Hold your head still."),
               ("move", 10.0, "Now gently rotate your head. Keep looking around the controller."),
               ("still_2", 5.0, "Hold your head still again.")],
    "translate": [("still_1", 5.0, "Hold your head still."),
                  ("move", 10.0, "Now gently lean and move your head side to side and forward and back."),
                  ("still_2", 5.0, "Hold your head still again.")],
    # Supported-hand follow-up: right controller held with the side grip pressed, forearm on table.
    "held": [("held_still", 15.0, "Keep holding the grip. Keep your hand resting and your head still.")],
    "held_rotate": [("still_1", 5.0, "Keep holding the grip. Hold your head still."),
                    ("move", 10.0, "Now gently rotate your head. Keep your hand still and the grip held."),
                    ("still_2", 5.0, "Hold your head still again.")],
    "held_translate": [("still_1", 5.0, "Keep holding the grip. Hold your head still."),
                       ("move", 10.0, "Now gently lean and move your head side to side and forward and back. Keep your hand still."),
                       ("still_2", 5.0, "Hold your head still again.")],
    # Retry with spoken sub-cues inside the SAME 10 s move phase (phase boundaries unchanged).
    "held_translate_guided": [("still_1", 5.0, "Keep holding the grip. Hold your head still."),
                              ("move", 10.0, [(0.0, "Lean left."), (2.5, "Lean right."),
                                              (5.0, "Lean forward."), (7.5, "Lean back.")]),
                              ("still_2", 5.0, "Back to the middle. Hold your head still again.")],
    # Test 4: one ordinary out-of-view movement while holding the grip (no abort-on-loss).
    "occlusion": [("baseline", 5.0, "Keep holding the grip. Hold the controller in front of you, in view."),
                  ("out_of_view", 3.0, "Gently move the controller down out of view, below the table edge or toward your lap. Keep holding the grip."),
                  ("return", 5.0, "Bring the controller back into view in front of you and hold it there.")],
    # Test 4 retry: same phases, very short cues (movement briefed beforehand).
    # Spanish variant of the short-cue occlusion retry (same phases/timing).
    # Test 5: one normal recenter (Spanish, short cues). No abort-on-loss.
    "recenter_es": [("baseline", 5.0, "Quieto."),
                    ("look_left", 7.0, [(0.0, "Mira un poco a la izquierda."), (3.5, "Quieto.")]),
                    ("recenter", 12.0, [(0.0, "Centra ahora.")]),
                    ("post", 8.0, [(0.0, "Quieto.")])],
    # Test 5 retry: explicit Meta-button hold/release cues (Spanish).
    "recenter2_es": [("baseline", 5.0, "Quieto."),
                     ("look_left", 7.0, [(0.0, "Mira un poco a la izquierda."), (3.5, "Quieto.")]),
                     ("recenter", 12.0, [(0.0, "Mantén pulsado el botón Meta con el pulgar."),
                                         (5.0, "Suelta el botón Meta. Sigue con el gatillo lateral.")]),
                     ("post", 8.0, [(0.0, "Quieto.")])],
    # Test 5 attempt 3: no head-turn step; explicit Meta hold/release (Spanish).
    "recenter3_es": [("baseline", 5.0, "Quieto."),
                     ("recenter", 12.0, [(0.0, "Mantén pulsado el botón Meta con el pulgar."),
                                         (5.0, "Suelta el botón Meta. Sigue con el gatillo lateral.")]),
                     ("post", 8.0, [(0.0, "Quieto.")])],
    "occlusion_short_es": [("baseline", 5.0, "Mantén el control a la vista."), ("out_of_view", 5.0, "Abajo."),
                           ("return", 5.0, "Arriba otra vez.")],
    "occlusion_short": [("baseline", 5.0, "Hold it in view."), ("out_of_view", 5.0, "Down."),
                        ("return", 5.0, "Back up.")],
}  # fmt: skip

INTRO = {
    "held": "Put on the headset. Hold the right controller with the grip pressed, forearm resting on the table.",
}


LANG: str | None = None  # spd-say language, e.g. "es"; None = default voice

ES = {
    "intro": "Ponte el visor. Sujeta el control derecho con el gatillo lateral presionado.",
    "not_detected": "Esperando el control derecho con el gatillo lateral presionado.",
    "detected": "Control derecho detectado. Empezamos.",
    "finished": "Para. Terminado. Puedes relajarte.",
    "aborted": "No se detectó el control derecho. Paramos esta prueba.",
}


def say(text: str) -> None:
    cmd = ["spd-say", "-w"] + (["-l", LANG] if LANG else []) + [text]
    subprocess.run(cmd, timeout=30, check=False)


def mark(marks: Path, label: str) -> int:
    t = time.monotonic_ns()
    with marks.open("a") as f:
        f.write(f"{t} {label}\n")
    return t


def right_ready(jsonl: Path, max_age_s: float = 0.3, min_squeeze: float | None = None) -> bool:
    """True if the newest decoded pose record is fresh and the right grip is focused/active/tracked."""
    try:
        with jsonl.open("rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 20000))
            lines = f.read().decode("utf-8", "replace").splitlines()[1:]
    except OSError:
        return False
    for line in reversed(lines):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get("type") != "pose" or not r.get("ok"):
            continue
        if (time.monotonic_ns() - r["host_ns"]) / 1e9 > max_age_s:
            return False
        rec = qp.decode_record(r["raw"])
        ok = bool(isinstance(rec, qp.PoseRecord) and rec.focused and rec.poses["grip_r"].status == qp.TRACKED)
        if ok and min_squeeze is not None:
            sq = rec.inputs["in_r"]["squeeze"]  # type: ignore[union-attr]
            ok = sq is not None and sq >= min_squeeze
        return ok
    return False


FOCUS_MENU_PLAN = [
    {"phase": "baseline", "planned_s": 5.0, "cue": "Keep holding the grip. Baseline."},
    {"phase": "menu", "planned_s": 5.0, "cue": "Open the system menu now. Keep holding the grip."},
    {"phase": "return_held", "planned_s": "until 5 s of available right input (max 40 s)",
     "cue": "Close the menu and return to the app. Keep holding the grip."},
    {"phase": "release", "planned_s": 1.0, "cue": "Release the grip now."},
    {"phase": "repress", "planned_s": 3.0, "cue": "Press the grip again and hold."},
]


def run_focus_menu(jsonl: Path, marks: Path) -> int:
    """Test 3: records THROUGH input loss (no abort). Release is cued only after recovery."""
    def timed(phase: str, cue: str, dur: float) -> None:
        mark(marks, f"phase_start {phase}")
        t0 = time.monotonic()
        say(cue)
        time.sleep(max(0.0, dur - (time.monotonic() - t0)))

    timed("baseline", FOCUS_MENU_PLAN[0]["cue"], 5.0)
    timed("menu", FOCUS_MENU_PLAN[1]["cue"], 5.0)
    mark(marks, "phase_start return_held")
    say(FOCUS_MENU_PLAN[2]["cue"])
    t0 = time.monotonic()
    avail_since: float | None = None
    while True:
        if right_ready(jsonl, min_squeeze=0.0):  # available = focused/active/tracked + grip action active
            avail_since = avail_since or time.monotonic()
            if time.monotonic() - avail_since >= 5.0:
                mark(marks, "recovery_interval_captured")
                break
        else:
            avail_since = None
        if time.monotonic() - t0 > 40.0:
            mark(marks, "recovery_not_observed_40s")
            say("Right controller input did not recover. Stopping. You can relax.")
            mark(marks, "phase_end")
            return 5
        time.sleep(0.05)
    mark(marks, "phase_start release")
    say(FOCUS_MENU_PLAN[3]["cue"])
    time.sleep(0.8)
    mark(marks, "phase_start repress")
    say(FOCUS_MENU_PLAN[4]["cue"])
    time.sleep(3.0)
    mark(marks, "phase_end")
    say("Stop. This recording is finished. You can relax.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=Path, required=True, help="recording path without extension")
    ap.add_argument("--kind", choices=sorted(PHASES) + ["focus_menu"], required=True)
    ap.add_argument("--ready-s", type=float, default=2.0)
    ap.add_argument("--wait-s", type=float, default=90.0)
    ap.add_argument("--abort-on-loss", action="store_true",
                    help="end the measurement if right input stops being fresh/focused/active/tracked")
    ap.add_argument("--loss-grace-s", type=float, default=0.3)
    ap.add_argument("--lang", help="spoken cue language for spd-say (e.g. es); kind *_es uses Spanish phrases")
    ap.add_argument("--start-min-squeeze", type=float,
                    help="readiness only: also require right squeeze >= this before 'Starting'")
    args = ap.parse_args()
    global LANG
    LANG = args.lang
    es = args.kind.endswith("_es")
    jsonl, marks = args.base.with_suffix(".jsonl"), args.base.with_suffix(".marks.txt")
    if args.kind == "focus_menu":
        plan = FOCUS_MENU_PLAN
    else:
        plan = [{"phase": p, "planned_s": d, "cue": c} for p, d, c in PHASES[args.kind]]  # list cues = timed sub-cues
    args.base.with_suffix(".planned.json").write_text(json.dumps({"kind": args.kind, "phases": plan}, indent=1))

    mark(marks, "waiting_for_right_ready")
    say(ES["intro"] if es else INTRO["held"] if (args.kind.startswith("held") or args.kind in ("focus_menu", "occlusion", "occlusion_short")) else
        "Put on the headset and look at the right controller on the table. Do not touch it.")
    t_start = time.monotonic()
    ready_since: float | None = None
    last_prompt = time.monotonic()
    while True:
        if right_ready(jsonl, min_squeeze=args.start_min_squeeze):
            ready_since = ready_since or time.monotonic()
            if time.monotonic() - ready_since >= args.ready_s:
                break
        else:
            ready_since = None
            if time.monotonic() - last_prompt > 15:
                say(ES["not_detected"] if es else "Right controller not detected yet." if args.start_min_squeeze is None else
                    "Waiting for the right controller with the side grip pressed.")
                last_prompt = time.monotonic()
        if time.monotonic() - t_start > args.wait_s:
            mark(marks, "aborted_right_not_ready")
            say(ES["aborted"] if es else "Right controller was not detected. Stopping this trial.")
            return 3
        time.sleep(0.1)
    mark(marks, "right_ready")
    say(ES["detected"] if es else "Right controller detected. Starting.")
    if args.kind == "focus_menu":
        return run_focus_menu(jsonl, marks)
    for p, dur, cue in PHASES[args.kind]:
        mark(marks, f"phase_start {p}")
        t0 = time.monotonic()
        if not args.abort_on_loss:
            for off, text in (list(cue) if isinstance(cue, list) else [(0.0, cue)]):
                time.sleep(max(0.0, off - (time.monotonic() - t0)))
                if isinstance(cue, list):
                    mark(marks, f"cue {text}")
                say(text)
            time.sleep(max(0.0, dur - (time.monotonic() - t0)))
            continue
        subcues = list(cue) if isinstance(cue, list) else [(0.0, cue)]
        speaker: subprocess.Popen[bytes] | None = None
        lost_since: float | None = None
        while time.monotonic() - t0 < dur:
            if subcues and time.monotonic() - t0 >= subcues[0][0] and (speaker is None or speaker.poll() is not None):
                off, text = subcues.pop(0)
                if isinstance(cue, list):
                    mark(marks, f"cue {text}")
                speaker = subprocess.Popen(["spd-say", "-w", text])
            if right_ready(jsonl):
                lost_since = None
            else:
                lost_since = lost_since or time.monotonic()
                if time.monotonic() - lost_since >= args.loss_grace_s:
                    mark(marks, f"input_lost during {p}")
                    if speaker is not None:
                        speaker.wait(timeout=30)
                    say("Right controller input was lost. The measurement has ended. Keep still for a few seconds.")
                    time.sleep(3.0)
                    mark(marks, "diagnostic_tail_end")
                    say("Stop. You can relax.")
                    return 4
            time.sleep(0.05)
        if speaker is not None:
            speaker.wait(timeout=30)
    mark(marks, "phase_end")
    say(ES["finished"] if es else "Stop. This recording is finished. You can relax.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
