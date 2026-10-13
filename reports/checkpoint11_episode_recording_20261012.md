# Checkpoint 11: episode recording (2026-10-12)

## What it does

While teleoperating, the operator marks episodes with the right controller's thumb buttons:

| Button | Effect |
|---|---|
| A | start an episode; pressed again, stop and **save** it |
| B | **discard** the episode being recorded |

The laptop speaks "Grabando", "Guardado", "Descartado" (and "No se puede grabar" if a start is refused).
A press counts only after the button was seen released. The buttons never affect motion.

Episodes are written on the machine that talks to the arm and has the cameras (linux2), under
`~/quest-xarm-teleop-dryrun/episodes/<session>/`. Nothing in the design needs two machines: when the
operator's PC is also the arm's PC, the same program records there.

## What one episode contains

```
episodes/<session>/session.json            settings, units, field descriptions, clock reference, gripper calibration
episodes/<session>/episode_0001/
    meta.json                              times, row count, how it ended, camera statistics, problems
    steps.jsonl                            one row per 20 ms control tick
    events.jsonl                           clutch transitions, faults, gripper commands during the episode
    cam_iphone.mjpeg + .frames.jsonl       wrist camera, frames as sent by the camera wall, time per frame
    cam_zed.mjpeg    + .frames.jsonl       fixed camera
```

Each row of `steps.jsonl`:

| Field | Meaning |
|---|---|
| `t` | host monotonic clock, ns (same clock as the camera frame times) |
| `tcp_mm`, `tcp_rpy`, `tcp_q` | measured tool pose in the arm's base frame: mm, roll/pitch/yaw rad, quaternion |
| `joints` | measured joint angles, rad |
| `cmd_mm`, `cmd_rpy` | target sent to the controller in that tick (null when none was sent) |
| `grip_pct`, `grip_ticks`, `grip_load` | measured jaw position (percent closed, servo ticks) and load |
| `grip_cmd_pct` | jaw target proposed in that tick |
| `state`, `clutch`, `trig`, `hand_mm`, `hand_q` | clutch state and the controller inputs |

Both the measured pose and the commanded target are kept, so labels can be built either way.

## Deriving action labels

`scripts/episode_inspect.py <session> --labels K` checks every episode and writes `labels_k<K>.json`:
for each wrist-camera frame, the arm pose and jaw aperture interpolated to that frame's time, and the
action `inv(T_i) @ T_(i+K)` (translation in metres, rotation vector in radians) plus the aperture at
`i+K`. This is the construction in the lab's `ACTION_FRAME_CONTRACT.md`, with one difference: the frame
is the arm's tool (flange) frame, not the wrist camera's optical frame. Converting needs the camera
mounting transform, which is not known here.

## How it is built

- `scripts/episode_recorder.py`: receives every record the teleop adapter writes, through a queue; one
  writer thread does all file work. It has no access to the arm or gripper.
- `scripts/episode_cameras.py`: a separate process that reads the two camera streams (HTTP GET on
  loopback only) and writes the frames of the current episode. It keeps the last 4 s in memory, so the
  video starts at the episode's start time. Being a separate process, it cannot delay the control loop.
- `scripts/live_teleop.py --record-dir <dir>`: turns recording on. With it, each tick also reads the
  joint angles (one more read-only request). A recording fault can never stop the arm loop.
- An episode does not start when a camera is not delivering frames or less than 5 GB of disk is free.
  It is stopped and saved after 5 minutes, and at session end (marked `closed_by: session_end`).

## Checks done

| Check | Result |
|---|---|
| Unit tests, synthetic cameras (8) | pass |
| End-to-end test: simulated arm controller + synthetic cameras + button presses | pass on linux2 |
| Full test modules on linux2 (178 tests) | pass (one timing-dependent test failed once under load, passed on 6 reruns) |
| Real camera wall, made-up robot rows, 8 s, no arm connection | 30.0 fps on both cameras, largest gap 36 ms, all frames decode, video starts 22 ms after the start mark |

## First session on the rig (`rec_20261012-195208`)

Approved by the operator in a question form (labmate on site, area clear, other agent paused, OK to move;
headset on the table facing away). Settings as in the previous session: box 100/100/150/150/35/120 mm,
120 mm/s, full wrist rotation up to 30 deg, gripper on. 4-minute window.

| Item | Result |
|---|---|
| Session | ended normally: 321 s, 15 engagements, 6183 targets, 0 halts, no latched fault, 0 link refusals |
| Arm after | parked, state 4, no error, at x 303.4, y 52.3, z 406.9 mm |
| Episode 1 | 41.9 s, 2097 rows at 50.0 Hz (largest gap 20.8 ms), pose and joints in every row, 1248 targets |
| Arm motion in the episode | 559 mm of travel; following for 25.0 s |
| Video | wrist 1259 frames, fixed 1256 frames, 30.0 fps; largest gap 66 ms (wrist), 36 ms (fixed); 54 MB |
| Rows vs video | image change per 2 s follows the recorded arm travel (correlation 0.86 fixed camera, 0.70 wrist camera), including a 14 s pause that is still in both |
| Extra arm read for the joints | whole per-tick read took 1.0 ms median, 2.4 ms at most |
| Labels | `--labels 8` gave 1250 actions; median step 0.9 mm, largest 31.7 mm and 5.5 deg over 8 frames |

**The A and B buttons were not pressed in this session** (no press in 17,344 controller records), so
episode 1 was started and saved from the laptop with the toggle signal (SIGUSR2) while the operator was
moving the arm. The spoken cues "Grabando" and "Guardado" fired. The same app build did report A and B
presses in the button test of 2026-09-30, and the button logic passes with the simulated arm, but a
press on the real controller starting an episode has not been seen yet. B (discard) is equally unseen
on hardware. The jaws stayed open during the episode (trigger at 0), so no jaw motion is in it.

## Limits

- Camera frame times are arrival times on linux2. The camera wall gives no capture time, so the
  exposure-to-arrival delay is unknown and uncorrected.
- Video is stored as received (MJPEG): about 77 MB per minute for the two cameras. The lab disk had
  24 GB free; with the 5 GB floor that is about 4 hours of episodes.
  `scripts/ops/episodes_fetch.sh <session>` copies a session to this PC (it deletes nothing on linux2).

## Running a recording session

```
REC=1 scripts/ops/teleop_live_session.sh <name> <confirmation file> <seconds> [live_teleop flags]
```

## Second session: bigger box, buttons used by the operator (`rec_20261012-201053`)

The operator asked for a lower floor ("at least 20 cm") and more room to the left. Both limits were this
program's own box, not the controller's. New option `--box-abs x_min,x_max,y_min,y_max,z_min,z_max`
(absolute, base frame; must contain the start pose and stay 10 mm inside the controller's fence).
Box used: x 199-479, y -247...260, z 280-546 mm: floor 8.2 cm lower than before, left side 18.5 cm
further. The controller's fence (z >= 270, y <= 270) was not changed, so 20 cm lower was not available.
Approved in a question form (labmate on site, bigger box clear, other agent paused, OK to move);
operator chose 120 mm/s over the offered 60 mm/s; headset on the table facing away; 4 minutes.

| Item | Result |
|---|---|
| Session | ended normally: 278 s, 13 engagements, 5211 targets, 0 halts, no latched fault |
| Arm after | parked, state 4, no error, at x 279.5, y 180.6, z 342.6 mm |
| Space actually used | x 258-450, y -60...219, z 280-461 mm |
| Held at the new floor | 5.6 s with the hand asking for lower; never held at the left edge |
| Buttons | A pressed 3 times, B once, all acted on: episode 1 saved, episode 2 discarded; cues spoken |
| Episode 1 | 70.0 s, 3502 rows at 50.0 Hz, 1491 mm of travel, jaws 0.9-88.4 % closed, 2101 + 2098 frames at 30 fps, 86 MB, no problems |

Video at the lowest point (flange z 280.2 mm): the jaw tips are inside the clear bin, beside the white
part. The jaws were closed (to 88 %) only later, at flange z 324-343 mm, with nothing between them. So
the bin is now reachable, and no grasp is seen in this session. Whether 280 is low enough to grasp a
part lying on the bin floor is not settled: the operator pushed against the floor for 5.6 s.

## Floor: request for 15 cm more (same evening)

The operator found the 280 mm floor still too high and asked for 15 cm more. That is below the controller's
own fence floor (270 mm), which this project does not write. Asked in a question form:

- Labmate's measurement with the arm parked at flange z 342.6 mm (tool tilted about 13 deg): "About 15 cm"
  from the jaw tips straight down to the bottom of the bin. So the tips would touch near flange z 193 mm
  at that tilt, a little higher (about 200 mm) with the tool vertical; about 8 cm below the 280 mm floor.
- Who lowers the controller's floor: "My other agent (Recommended)".

Proposed first step: controller z-min 270 -> 215 mm, then this program's floor at 220 mm
(`--fence-margin-mm 5`, new option, default 10; the arm showed no overshoot at the floor), about 2 cm
above the bin bottom. A further step can follow once the remaining gap has been seen or measured.
`scripts/ops/episode_to_mp4.sh <episode>` makes viewable mp4 files from an episode's camera recordings.

## Reach test with the lower floor (`tele_20261012-203308`): the arm touched down, controller error 31

- 20:29:53: the operator's other agent lowered the controller fence floor 270 -> 215 mm (controller + stepper.py,
  other limits unchanged, collision sensitivity reported as 2, arm not moved).
- Approved in a question form (labmate on site, clear down to the bins, other agent paused, OK to move; headset
  on the table facing away; "2 minutes is enough. i just want to see if i can reach the lower part of the bucket.").
  Box x 199-479, y -247...260, z 220-546 mm (`--fence-margin-mm 5`), 120 mm/s cap, no episode recording.
- The operator's camera page was not connected when the window opened (the relay had just been restarted).
  A spoken cue was given; the page connected about 8 s later, before the descent.
- The operator went down slowly (10-25 mm/s). At flange z 229.2 mm (x 323, y 163, tool about 11 deg from
  straight down) the controller raised **error 31 (collision)**, 56.3 s into the program, after 9 s of following.
  The program latched the fault and sent state 4; it did not clear the error. The arm backed off by itself and
  is stopped at z 248.4 mm with error 31 still set. Jaws open (525 ticks).
- Wrist camera at contact: the white part is between the open jaws. Both jaws look intact in the frames after.
  What was touched (bin bottom, its raised step, or the part) is not determined.
- **The floor of 220 mm did not protect.** It was derived from the labmate's "about 15 cm" (contact expected near
  193-200 mm); contact came at 229 mm, so the real gap was about 11.5 cm. Lesson: a floor must come from a
  measured contact height, not an estimate. The slow descent kept the contact gentle.
- Result for the operator's question: the reach is enough. At 245-248 mm the open jaws already straddle the part.
- Proposed: this program's floor at 240 mm; controller floor back up from 215 to 235 mm (lab-side change).
  The error has to be cleared on the lab side after the labmate has looked at the gripper and the bin.

## Pick test at the 240 mm floor (`tele_20261012-204347`): part picked and lifted, then controller error 24

- Before it: the other agent raised the controller floor 215 -> 235 mm (20:37:52) and cleared error 31 (20:40:26),
  arm not moved; the operator reported "nothing wrong with the gripper". Approved in a question form
  ("Pick test only, 2 minutes"). Box x 199-479, y -247...260, z 240-546 mm, 120 mm/s cap. The operator's camera
  page was connected (checked through the camera wall's viewer count).
- **The pick worked.** One engagement of 43 s. The arm sat at the floor (240.2 mm) for 9.5 s without touching,
  the jaws closed to 85 % on the white part at the floor, the part was lifted to flange z 310 mm (7 cm) between
  the jaws, brought back down and released (wrist-camera frames at each step).
- Then, while lifting again at the 120 mm/s cap with the wrist turning at 20 deg/s, the controller raised
  **error 24 (speed exceeds limit)** at 75.35 s. The program latched the fault and sent state 4; the error was
  not cleared. Arm stopped at x 311.9, y 142.4, z 288.4 mm, jaws open. Nothing was hit.
- Ruled out: scheduler timing (20.0 ms +- 0.2 throughout), a jump in the target (smooth 2.4 mm steps, lag 6.7 mm),
  the wrist singularity (joint angles reconstructed with a kinematic model that matches the recorded joints of
  an earlier episode to 1.6 mm: J5 never below 22.6 deg; fastest joints 48-49 deg/s, earlier in the session).
- **Most likely cause: the controller's reduced mode is ON again.** The start-up read of both sessions after the
  fence change of 20:29:53 shows reduced mode on; every earlier session today shows it off. With it off the arm
  spent seconds above 100 mm/s in earlier sessions with no error; with it on, the error came at this session's
  fastest moment (117 mm/s). The reduced-mode limits read on 2026-10-01 were TCP 50 mm/s and joints 20 deg/s;
  the present values were not read. This is an inference from timing, not a controller statement.
- My miss: the program read "reduced mode on" at the start of both sessions and neither flagged the change nor
  compared its limits with the session's speeds.
- Fix: `live_teleop.py` now records the reduced-mode TCP and joint speed limits and, when reduced mode is on,
  refuses to start if `--ctrl-speed` is above the TCP limit or `--rot-speed` x 2.5 is above the joint limit
  (test with the simulated controller; 80 arm-related tests pass on linux2).
