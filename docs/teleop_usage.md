# Using the teleop

Everything is run from the PC the Quest is plugged into. The command starts and stops the lab-host
(linux2) side by itself; nothing has to be started there by hand.

```
cd ~/quest-xarm-teleop
scripts/teleop record        # demo recording session; runs until you press Ctrl-C
```

## Before a session

1. Quest connected by USB, right controller on. Put the headset down with its **front pointing away from
   you**, the way you face (or wear it). If it faces you instead, set `OPERATOR_YAW=180` in the settings;
   with the wrong value forward/back and left/right are swapped.
2. Someone is at the rig with the physical stop in reach, and nobody is near the arm.
3. Your other agent on linux2 is paused. Nothing else may command the arm or the gripper.
4. `scripts/teleop status` shows the arm with `error [0, 0]`. An error has to be cleared on the lab side.

## Commands

| Command | What it does |
|---|---|
| `scripts/teleop record [minutes]` | Session with demo recording |
| `scripts/teleop try [minutes]` | The same without recording |
| `scripts/teleop status` | Read-only: arm state, who is connected to it, camera view, headset, disk |
| `scripts/teleop cameras` | Starts the camera view if needed and prints its address |
| `scripts/teleop episodes [session]` | Lists recorded sessions, or checks the demos of one |
| `scripts/teleop watch` | Follows the lab program's console during a session (second terminal) |
| `scripts/teleop stop` | Ends the running session from a second terminal (software stop) |

A session has no time limit unless you give minutes. It ends when you press Ctrl-C, and by itself when
the arm program has lost this PC for 60 s, or when the arm has not followed the hand for 15 min. A demo
still recording after 5 min is saved and closed. These three values are settings.

`record` and `try` check the rig, wait until the camera page (`http://127.0.0.1:18091/`) is open, show
four conditions and ask once for `yes`. What you type is saved with the session. `no` ends it with
nothing started.

Operate from that camera page: the gripper camera and the RealSense, 8 pictures a second, about 70 ms old
when they reach this PC (measured 2026-10-14), plus the cameras' own delay. Do not operate from the lab's
stepper page and preferably close it during a session: it shows the gripper camera as one still picture
every 2 s, and its RealSense stream at full rate (about 1.5 MB/s) uses roughly everything the link carries.
Which cameras are shown and recorded is the `CAMERAS` setting (the ZED is left out while it sends no picture).

## During a session

There is no sound. The terminal tells you where you are:

| Line in the terminal | Meaning |
|---|---|
| `Arm armed. workspace …` | The arm program on linux2 is up and has checked the arm |
| `Controller found.` | The headset app is running and sees the right controller |
| `Release the side grip (middle finger).` | Only shown if you are holding it at the start |
| `READY, no time limit: Ctrl-C ends the session. …` | Hold the side grip and move |
| `arm following` / `paused` | The clutch engaged / released |
| `DEMO RECORDING …` / `DEMO SAVED` / `DEMO DISCARDED` | What your A and B presses did |
| `DEMO NOT STARTED: …` | A camera is not delivering, or the disk is nearly full |
| `The arm program is ending: …` | It stopped by itself; the line says why (connection lost, idle, stopped from another terminal) |
| `20 seconds left` | Only with a time limit: save the demo in progress now |
| `FAULT: …` | A fault ended the session; the arm is stopped; read the summary |
| `Time is up. Release the side grip.` | Only with a time limit |

| Control (right controller) | Effect |
|---|---|
| Side grip, middle finger (hold) | The arm follows your hand; release and it stops |
| Index trigger | Closes the jaws |
| A, lower thumb button | Starts a demo; pressed again, stops and saves it |
| B, upper thumb button | Discards the demo in progress |

A demo still open when the session ends is saved and marked `closed_by: session_end`.
Ctrl-C ends the session cleanly: the lab program parks the arm. Neither Ctrl-C nor
`scripts/teleop stop` replaces the physical stop.

## After a session

The summary lists the demos. For each one, on this PC and on linux2:

```
data/episodes/<session>/episode_0001/view_iphone.mp4     gripper camera, for watching
data/episodes/<session>/episode_0001/view_rs_color.mp4   fixed RealSense camera (before 2026-10-14: view_zed.mp4)
data/episodes/<session>/episode_0001/steps.jsonl         arm pose, joints, targets, jaws every 20 ms
data/episodes/<session>/episode_0001/cam_*.mjpeg + .frames.jsonl   the recorded frames with their times
```

On linux2 the same folders are under `~/quest-xarm-teleop-dryrun/episodes/<session>/`.
Action labels: `python3 scripts/episode_inspect.py <session folder> --labels 8` (see the report).

## When something goes wrong

| What you see | What to do |
|---|---|
| `BLOCKED: the arm controller has error …` | Have it checked and cleared on the lab side, then start again |
| `ABORTED: reduced mode is ON with a TCP speed limit…` | Reduced mode was switched on at the controller. Have it switched off, or lower the speeds in the settings |
| `ABORTED: the box … comes within … of the fence` | The controller's fence was changed. Adjust `BOX_ABS` or have the fence restored |
| `ABORTED: the arm is at … outside the requested box` | The arm was moved out of the box by another program. Widen `BOX_ABS` or have it moved back |
| `FAULT: …` with a controller error in the summary | The arm is stopped. Someone looks at it; the error is cleared on the lab side |
| `NOT STARTED. The arm program says: …` | Read the reason; the two rows above are the usual ones |
| Camera page frozen | `scripts/teleop cameras`, then reload the page |
| Gripper picture upside down | The phone app turned its picture 180° (it did when the wrist was tilted, 2026-10-14). The camera page turns it back by itself while the blue jaws are in view. Recorded demos keep the picture as the phone sends it |
| `DEMO NOT STARTED` naming a camera | That camera sends no picture (the ZED, 2026-10-14). Take it out of `CAMERAS` in the settings and start again |
| `need exactly one headset` | Check the USB cable and allow debugging in the headset |

## Settings

`config/teleop_session.conf`: the box, speeds, rotation limit, gripper, headset direction, the cameras,
whether to wait for the camera page, default lengths. Detailed logs of each session are in `data/sessions/`. The floor
(`z_min` 240 mm) was found by touching down in one bin; other bins may be shallower.

## Not done yet

- Running with the headset plugged straight into the arm's PC (one machine). The programs allow it; this command still assumes linux2 over SSH.
- Video inside the headset.
- Moving the arm to a fixed start pose before each demo.
