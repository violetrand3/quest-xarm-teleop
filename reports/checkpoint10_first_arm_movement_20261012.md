# Checkpoint 10: first real arm movement and gripper check after the repair (2026-10-12)

The arm moved under our own command link for the first time: 10 mm straight up and back, in position mode, at 10 mm/s. The streaming-mode run was **not** done, because the controller's reduced mode is off. The repaired gripper closes and reopens.

## 1. Confirmations (verbatim in `data/checkpoint10/confirm_*.txt`)
- Arm: "Labmate on site, agrees", "Labmate clear, stop in reach", "Nothing else will command the arm", "OK to enable and move the arm"; the other session that restarted `stepper.py` at 17:46 is "My other agent, now paused".
- Reduced mode off: "Position-mode run now (Recommended)", i.e. only the position-mode run.
- Gripper: "Yes, jaws clear, go ahead" for the 30 % check, then "Yes, jaws still clear" for the wider sweep.

## 2. What the rig looked like before the test
| Item | Reading (read-only probe, 17:55) | On Oct 1 |
|---|---|---|
| Pose | x 313.38, y −48.41, z 402.31 mm; roll 179.73°, pitch −8.63°, yaw −6.95° | x 257, y −291, z 496 |
| State / mode / errors / queue | 4 / 0 / none / 0 | same |
| Simulation flag | false | false |
| **Reduced mode** | **off** | on |
| Fence | on, x 180–502, y −300–270, z 270–585 | same |

- The first attempt refused to start because reduced mode was off; nothing was commanded.
- Who turned reduced mode off is unknown. Nothing in the lab's stepper program sets it.
- `stepper.py` was edited and restarted at 17:46 by another session (a new "h8" policy), and that program moved the arm twice at 17:50 (two 40 mm stage moves toward the start pose), also with reduced mode off.

## 3. Arm test, position mode (`first_move_m0_20261012-180220.json`)
Run with the explicit waiver flag `--allow-reduced-mode-off`, which only a position-mode run accepts. Hard box: within 1.5 mm of the start in x and y, z from 400.81 to 413.81 mm; speed cap 10 mm/s.

| Test | Result |
|---|---|
| A. One move up 10 mm and back | Reached 10.00 mm in 1.24 s; motion began 84 ms after the command; peak speed 10.3 mm/s; returned to 0.00 mm |
| B. Second target sent 0.4 s into a move | **Queued**: the arm went to the full 10 mm (9.999) before coming back. Expected in position mode |
| C. `set_state(4)` during a move | Requested at 4.22 mm of travel, arm stopped at 4.46 mm: **0.24 mm** after the request. The stop call returned in 1.2 ms; state 4; queue 0; it did not resume. Moving again needed an explicit re-arm |
| End | Back at the start (0.0 mm away), state 4, mode 0, no error |

- Sent to the controller by the commanding link: enable ×2, mode ×3, state ×5, line move ×6, plus reads. Refused: nothing.
- **Camera cross-check:** the wrist camera's picture changed in step with the reported travel (correlation 0.91; fixed ZED view 0.89), and the frames at the top of the move show the bin shifted. So the arm really moved; this is not the controller's word alone.
- The stop figure is at 10 mm/s only and comes from the controller's position feedback sampled every 20 ms. It says nothing yet about stopping from teleop speeds.

Still open for the arm:
- The streaming-mode (mode 7) run: whether a new target replaces the old one there. Waiting for reduced mode to be switched back on.
- Directions: only +z (up) was exercised. x and y directions are still unverified.

## 4. Gripper after the repair
- The driver's calibration changed: open 512, closed 2048 ticks (was 3033 / 1181), closing effort 80 % (was 60 %). `scripts/gripctl_client.py` now reads the calibration from the driver instead of using fixed numbers.
- Step check (4 % → 34 % → 4 %): servo followed; the jaws moved only slightly.
- Sweep (`gripper_step_grip_20261012-180500.json`), jaw opening measured in the wrist camera low on the jaws:

| Servo, % closed | Jaw opening in the picture (px) |
|---|---|
| 5 (start) | 189 |
| 34 | 185 |
| 63 | 162 |
| 89 | 147 |
| 56 (reopening) | 154 |
| 21 | 177 |
| 6 (end) | 187 |

- At 89 % the jaw **tips** are almost touching (frame `sweep_3.jpg`). The numbers above are measured nearer the jaw base, where the movement is smaller.
- The first third of the servo travel barely moves the jaws (4 px for 30 %); after that they follow.
- No errors; the servo holds at 597 ticks, torque on, 46 °C.
- `scripts/wrist_frames_measure.py` was fixed: it assumed the jaws were centred in the picture, which is no longer true.

For the trigger path this means the trigger range should be about 0–90 % closed now, not 15–85 %.

## 5. Mistakes on my side during this session
- My first run script waited on a process match that also matched itself, so it sat for 60 s after the test had already refused to start, then killed its own post-processing. No command was involved. Replaced by lab-side scripts (`scripts/ops/lab_*_run.sh`).
- The camera relay did not start on the first try for the same reason.

## 6. State left
| Item | State |
|---|---|
| Arm | where it was before the test, state 4, mode 0, no error; `stepper.py` is its only connection |
| Gripper | 597 ticks (5.5 % closed), holding |
| Reduced mode | still off (not mine to change) |
| My processes on `linux2` | none, except the view-only camera relay (exits by itself) |
| Port 8765, lab services | untouched |

## 7. Next
1. Have reduced mode switched back on (in the arm's Studio page, by the lab).
2. Streaming-mode run of the same 10 mm test.
3. Small x and y moves to fix the direction mapping.
4. First teleoperated motion, slow, inside a small box.

---

## 8. Later the same day: streaming mode and the first live teleoperation
The operator decided to go without reduced mode ("Go without reduced mode", all four conditions ticked; verbatim in `data/checkpoint10/confirm_teleop_20261012.txt`).

### Streaming-mode check (`first_move_m7_20261012-182134.json`)
| Test | Result |
|---|---|
| One move up 10 mm and back | Reached 10.00 mm in 1.2 s, peak 10.4 mm/s |
| Second target sent 0.4 s into a move | **Replaced**: the arm turned back at 4.2 mm |
| `set_state(4)` during a move | Stopped 0.24 mm after the request; did not resume |

So mode 7 does what teleoperation needs: a new target replaces the old one and the speed argument is honoured.

### Session 1 (`teleop_tele_20261012-182231`), arm only
- Engaged 13 s; the arm followed the hand over about 90 mm in y; target-to-arm distance at most 1.5 mm.
- Ended by a latched fault, `workspace_violation:limited`, when the operator pushed against the box edge. **My bug:** with the request held exactly on the edge while the hand kept moving along the other axes, the rate-limited target drifted a few hundredths of a millimetre past the edge and the adapter treated that as a violation. The arm was stopped with `set_state(4)` as designed.
- Fix: the request is held 0.2 mm inside the edge and the limited target is confined to the box. Replaying this session's recorded input against a fake arm reproduces the fault with the fix switched off and runs clean with it (`tests/test_teleop_adapter.py`, `LiveSessionReplayTests`).

### Session 2 (`teleop_tele_20261012-183044`), arm and gripper
| Measure | Value |
|---|---|
| Time engaged | 77.7 s in 7 engagements; no faults, no halts |
| Why engagements ended | 4 clutch releases, 3 input gaps over 300 ms |
| Arm travel | x 291–359, y −60…+3, z 392–435 mm (box x 264–363, y −98…+4, z 392–462) |
| Target-to-arm distance | median 0.02 mm, max 1.47 mm |
| Arm speed | up to 30.7 mm/s (limit 30) |
| Tool orientation | unchanged throughout |
| Line targets sent / refused by the link | 3886 / 0 |
| Gripper | 195 commands, 0 errors; servo 1–89 % closed |
| Camera cross-check | picture change vs arm displacement: 0.94 (wrist), 0.93 (fixed); jaw opening vs servo position: −0.97 (189 px open, 146 px closed) |
| Link | RTT median 260 ms, p95 531 ms; input age max 296 ms |

- The link was slower than it should be: the full camera wall was still being viewed somewhere (its extra streams had viewers) during both sessions.
- Directions: the operator's hand moved left and the arm moved toward +y, as the mapping intends. Whether every direction feels right to the operator on the cameras has not been confirmed yet.

### State left
Arm parked at x 339.2, y 3.4, z 396.2 mm, state 4, mode 0, no error. Gripper open at 526 ticks. Reduced mode still off. No process of mine on `linux2` except the view-only camera relay.

### What is not there yet
- Tool rotation (the wrist does not follow the hand).
- A bigger workspace, higher speed and scale: all still first-session values.
- Recording of episodes for training.
- Video in the headset.
