# Using the teleop (xArm)

How to run an xArm6 session. The two YAM arms: `yam_teleop.md`. Setting up a new PC: `getting_started.md`.

Everything is run from the PC the Quest is plugged into. The command starts and stops the lab-host
(linux2) side by itself; nothing has to be started there by hand. `git pull` first: the session refuses to start
when the lab host runs a different version than your checkout.

```
cd quest-xarm-teleop         # wherever you cloned it
scripts/teleop record        # demo recording session; runs until you press Ctrl-C
```

## Before a session

1. Quest connected by USB, right controller on. Put the headset down with its **front pointing away from
   you**, the way you face (or wear it). If it faces you instead, set `OPERATOR_YAW=180` in the settings;
   with the wrong value forward/back and left/right are swapped.
2. Someone is at the rig with the physical stop in reach, and nobody is near the arm.
3. Nothing else is commanding the arm or the gripper: no policy or agent running from the lab's stepper page.
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
| `scripts/teleop sync` | Installs this checkout's programs on the lab host: only a committed, pushed version, not older than the lab's (see `getting_started.md`) |
| `scripts/teleop studio` | Opens UFactory Studio, the arm's own control page, at `http://127.0.0.1:18333/`: clearing a controller error, turning a wrist joint back |

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
| `Waiting for the controller: …` | The headset app cannot use the right controller yet. The line says why and what fixes it, and changes when the cause changes:<br>• **not showing the app / not sending anything**: the headset is asleep. Press its power button briefly or put it on for a moment<br>• **a system window is in front of the app**: put the headset on and close it (boundary, tracking or battery warning, menu). A stationary boundary avoids most of these<br>• **not connected**: the controller is asleep, its battery is flat, or the headset switched to hand tracking. Pick it up and press any button<br>• **cannot see it**: hold it in front of the headset, in a lit place<br>It waits until the controller is found (no time limit); Ctrl-C ends the session |
| `Controller found.` | The headset app is running and sees the right controller |
| `Release the side grip (middle finger).` | Only shown if you are holding it at the start |
| `READY, no time limit: Ctrl-C ends the session. …` | Hold the side grip and move |
| `arm following` / `paused` | The clutch engaged / released |
| `DEMO RECORDING …` / `DEMO SAVED` / `DEMO DISCARDED` | What your A and B presses did |
| `DEMO NOT STARTED: …` | A camera is not delivering, or the disk is nearly full |
| `The arm program is ending: …` | It stopped by itself; the line says why (connection lost, idle, stopped from another terminal) |
| `20 seconds left` | Only with a time limit: save the demo in progress now |
| `FAULT: …` | The arm is stopped, but the session goes on. The next lines say what to do; then press Enter to continue (Ctrl-C ends the session) |
| `Time is up. Release the side grip.` | Only with a time limit |

| Control (right controller) | Effect |
|---|---|
| Side grip, middle finger (hold) | The arm follows your hand; release and it stops |
| Index trigger | Closes the jaws |
| A, lower thumb button | Starts a demo; pressed again, stops and saves it |
| B, upper thumb button | Discards the demo in progress |

### Limits you will feel

All of them hold the arm at the edge, the way a wall would: no error, and the other directions keep working.

- **Box** (`BOX_ABS`): where the flange may go. The floor (`fence`) follows the lab's fence, 5 mm above it,
  so you can reach the tray floor; it limits the flange, not the jaw tips, so go down gently near the tray.
- **Tilt** (`TILT_MAX=60`): how far the gripper may lean from pointing straight down. If the session
  starts with the gripper leaning more than that, that lean is the limit, and you can always straighten it.
- **Turn** (`TURN_MAX=90`): how far the gripper may turn about the vertical, either way, from where it pointed
  at the start. 90 degrees is enough to put the jaws across an object lying in any direction.
- **Speed**: the target moves at most `V_MAX` mm/s, and the gripper turns at most `ROT_SPEED` degrees/s.
- **Wrist**: when the gripper points along the forearm (for example reaching close to the base, or high
  with the gripper tilted outward), joint 5 comes near 0. That is the wrist's turning point: there two
  wrist joints spin a full turn for a small move (2026-10-31: that stopped the arm with error 23). The
  program reads the joints every 20 ms and works out, from the arm's geometry, what each move would do to
  them. Within 15° of that point (or 45° of the joint 4/6 limits) it follows every move, or part of a move,
  that does not take the wrist further in, and stops the rest. You can always move away in a direction that
  helps; you do not have to retrace your path. The terminal then says, for example,
  `WRIST HELD (joint 5 at 15deg, …). Try: tilt the gripper back toward straight down.` The suggestion is
  worked out for the pose the arm is in.

A demo still open when the session ends is saved and marked `closed_by: session_end`.
Ctrl-C ends the session cleanly: the lab program parks the arm. Neither Ctrl-C nor
`scripts/teleop stop` replaces the physical stop.

## After a session

The summary lists the demos. The videos are made on linux2 and copied here with the rest of each demo, except the
raw camera files (`cam_*.mjpeg`, about 0.7 MB/s per camera), which stay on linux2: copying them took most of the
wait. A 2-demo, 8-minute session takes about 2 minutes. For each demo, on this PC and on linux2:

```
data/episodes/<session>/episode_0001/view_iphone.mp4     gripper camera, for watching
data/episodes/<session>/episode_0001/view_rs_color.mp4   fixed RealSense camera (before 2026-10-14: view_zed.mp4)
data/episodes/<session>/episode_0001/steps.jsonl         arm pose, joints, targets, jaws every 20 ms
data/episodes/<session>/episode_0001/cam_*.frames.jsonl the time of every recorded frame
```

The raw frames themselves (`cam_*.mjpeg`) are only on linux2. To copy them here too: `scripts/ops/episodes_fetch.sh <session>`.

On linux2 the same folders are under `~/quest-xarm-teleop-dryrun/episodes/<session>/`.
Action labels: `python3 scripts/episode_inspect.py <session folder> --labels 8` (see the report).

## When the arm stops with a fault

The session keeps running. The terminal shows the reason and, for a controller error, what it means.
1. Someone at the rig deals with the cause (whatever was hit, an emergency stop, a wound wrist).
2. Clear a controller error in UFactory Studio: `scripts/teleop studio` in a second terminal, then **Clear**
   on that page. For error 23 (a joint at its limit) also turn that wrist joint a full turn back toward 0
   in its joint control. Close the page afterwards: it can move the arm too.
3. Press **Enter** in the session's terminal. If the controller still has an error, it says so; otherwise
   release the side grip and hold it again to move. A demo that was recording goes on recording.

This program never clears a controller error itself.

## When something goes wrong

| What you see | What to do |
|---|---|
| `BLOCKED: the arm controller has error …` | Have it checked and cleared on the lab side, then start again |
| `ABORTED: reduced mode is ON with a TCP speed limit…` | Reduced mode was switched on at the controller. Have it switched off, or lower the speeds in the settings |
| `box fitted: …` at the start | Not an error. The box was cut back to stay `FENCE_MARGIN` mm inside the controller's fence, or stretched to where the arm was left (up to 50 mm outside it, never below the floor). The session runs |
| `ABORTED: the arm is at …` (far outside the box, below its floor, or near the controller's fence) | The arm was left far away by another program. Press "Next stage toward START" on the lab page, then start again |
| `FAULT: … wrist_limit:joint …` | The wrist still got within 15° of a joint limit; this program stopped the arm before the controller did, so there is no error to clear. Turn that joint a full turn back toward 0 in UFactory Studio, then press Enter |
| `ABORTED: the wrist is wound up: joint …` | Another program left a wrist joint a full turn from 0, close to its limit. Someone at the rig turns it back a full turn in UFactory Studio (joint control), then start again |
| `FAULT: …` with a controller error | See "When the arm stops with a fault" above |
| `NOT STARTED. The arm program says: …` | Read the reason; the two rows above are the usual ones |
| Camera page frozen | `scripts/teleop cameras`, then reload the page |
| Gripper picture upside down | The phone app turns its picture 180° when the wrist tilts. Click the picture on the camera page to turn it; your browser remembers it per camera. Recorded demos keep the picture as the phone sends it |
| `DEMO NOT STARTED` naming a camera | That camera sends no picture (the ZED, 2026-10-14). Take it out of `CAMERAS` in the settings and start again |
| `need exactly one headset` | Check the USB cable and allow debugging in the headset |
| `the lab host runs a newer version` | `git pull`, then start again |
| `this checkout is newer` | Your checkout has a version the lab host does not run yet: `scripts/teleop sync` (see `getting_started.md`) |

## Settings

`config/teleop_session.conf`: the box, speeds, tilt and turn limits, gripper, headset direction, the cameras,
whether to wait for the camera page, default lengths. Detailed logs of each session are in `data/sessions/`. The floor
(`z_min` 240 mm) was found by touching down in one bin; other bins may be shallower.

## Not done yet

- Running with the headset plugged straight into the arm's PC (one machine). The programs allow it; this command still assumes linux2 over SSH.
- Video inside the headset.
- Moving the arm to a fixed start pose before each demo.
