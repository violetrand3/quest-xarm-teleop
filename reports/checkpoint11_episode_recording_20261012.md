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
