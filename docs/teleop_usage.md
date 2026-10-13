# Using the teleop

Everything is run from the PC the Quest is plugged into. The command starts and stops the lab-host
(linux2) side by itself; nothing has to be started there by hand.

```
cd ~/quest-xarm-teleop
scripts/teleop record 8      # 8-minute demo recording session
```

## Before a session

1. Quest connected by USB, controllers on. The headset can lie on the table or sit on your forehead.
2. Someone is at the rig with the physical stop in reach, and nobody is near the arm.
3. Your other agent on linux2 is paused. Nothing else may command the arm or the gripper.
4. `scripts/teleop status` shows the arm with `error [0, 0]`. An error has to be cleared on the lab side.

## Commands

| Command | What it does |
|---|---|
| `scripts/teleop record [minutes]` | Session with demo recording (default 8 min, at most 28) |
| `scripts/teleop try [minutes]` | The same without recording (default 2 min) |
| `scripts/teleop status` | Read-only: arm state, who is connected to it, camera view, headset, disk |
| `scripts/teleop cameras` | Starts the camera view if needed and prints its address |
| `scripts/teleop episodes [session]` | Lists recorded sessions, or checks the demos of one |
| `scripts/teleop watch` | Follows the lab program's console during a session (second terminal) |
| `scripts/teleop stop` | Ends the running session from a second terminal (software stop) |

`record` and `try` check the rig, make sure the camera page (`http://127.0.0.1:18091/`) is connected,
then ask three yes/no questions, the headset placement, and for `GO`. What you type is saved with the
session. Anything other than `yes` / `GO` ends it with nothing started.

## During a session

| You hear | Meaning |
|---|---|
| "Suelta el botón del dedo medio" | Release the side grip so the clutch can arm |
| "Aprieta y mantén el dedo medio…" | The window is open: hold the side grip and move |
| "Activo" / "Pausa" | The arm is following / has stopped following |
| "Grabando" / "Guardado" / "Descartado" | A demo started / was saved / was discarded |
| "No se puede grabar" | A demo could not start (camera not delivering, or disk nearly full) |
| "Quedan veinte segundos" | Save the demo in progress now |
| "Fallo. Paramos." | A fault ended the session; read the summary |

| Control (right controller) | Effect |
|---|---|
| Side grip, middle finger (hold) | The arm follows your hand; release and it stops |
| Index trigger | Closes the jaws |
| A, lower thumb button | Starts a demo; pressed again, stops and saves it |
| B, upper thumb button | Discards the demo in progress |

A demo still open when the window closes is saved and marked `closed_by: session_end`.
Ctrl-C ends the session cleanly: the lab program parks the arm. Neither Ctrl-C nor
`scripts/teleop stop` replaces the physical stop.

## After a session

The summary lists the demos. For each one, on this PC and on linux2:

```
data/episodes/<session>/episode_0001/view_iphone.mp4     gripper camera, for watching
data/episodes/<session>/episode_0001/view_zed.mp4        fixed camera
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
| "Fallo. Paramos." with a controller error in the summary | The arm is stopped. Someone looks at it; the error is cleared on the lab side |
| Camera page frozen | `scripts/teleop cameras`, then reload the page |
| `need exactly one headset` | Check the USB cable and allow debugging in the headset |

## Settings

`config/teleop_session.conf`: the box, speeds, rotation limit, gripper, default lengths. The floor
(`z_min` 240 mm) was found by touching down in one bin; other bins may be shallower.

## Not done yet

- Running with the headset plugged straight into the arm's PC (one machine). The programs allow it; this command still assumes linux2 over SSH.
- Video inside the headset.
- Moving the arm to a fixed start pose before each demo.
