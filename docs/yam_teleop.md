# Bimanual YAM teleop

Two I2RT YAM arms on the lab host (CAN links `can_left`, `can_right`), driven by the two Quest controllers: the left
hand drives the left arm, the right hand the right arm. It shares the Quest side, the recorder and the launcher with
the xArm teleop, and nothing else: the YAM code never imports the xArm code, and the reverse (a test checks).

**Status (2026-11-18): works on the real arms and records demos**, so far at the slowest speed level (7.5 cm/s). First run
on the arms 2026-11-17; tuned from those runs (tracking check, link stalls, base distance). Still rough: see "Known
limitations" below.

Setting up your PC (once): `getting_started.md`. The first part of this page is for running a session; the second
part (from "How it works") is the reference.

## Commands

```
scripts/teleop yam-sim       # simulated arms on the lab host: nothing moves. Watch them at http://127.0.0.1:18767/
scripts/teleop yam-try       # THE REAL ARMS MOVE, no recording
scripts/teleop yam-record    # THE REAL ARMS MOVE, recording demos
scripts/teleop stop          # from a second terminal: ends the session (the arms are lowered to rest first)
```

`git pull` first: a session refuses to start when the lab host runs a different version than your checkout
(`getting_started.md` explains `scripts/teleop sync`). Only one YAM session can run at a time; a second one refuses
to start, and the real arms are locked by the program that has them open.

## Before a session (real arms)

`yam-try` and `yam-record` ask, and you answer `yes` only when all of this is true:

- someone is at the arms right now, agrees, and has the YAM e-stop in reach;
- nobody is near the arms and nothing is under them that a dropping arm could hit;
- nothing else will command the YAM arms (no other session, agent or program);
- both arms rest folded on their bases. A gripper more than about 60% open at start-up makes the start refuse
  (the arms are switched off again: they go limp), so close it first;
- you stand behind the arms, both facing away from you; both controllers are on.

The headset goes as for the xArm: front pointing the way you face (`OPERATOR_YAW` in `config/teleop_session.conf`).

## During a session

Watch the arms on the camera page, **http://127.0.0.1:18091/**: the RealSense, which sees both arms, and the two
wrist webcams. Click a picture to turn it upside down. The 3D view (http://127.0.0.1:18767/) shows what the arms are
*commanded* to do, not what they do.

| Control | Effect |
|---|---|
| Side grip (middle finger), each hand | Hold: that arm follows your hand. Release: it stops where it is |
| Index trigger, each hand | While the side grip is held: closes that arm's gripper (released = open) |
| Right thumb **A** | `yam-record`: starts a demo; pressed again, saves it |
| Right thumb **B** | `yam-record`: discards the demo in progress |
| **Enter** (terminal) | Acknowledges a latched problem (after you looked at it) |
| **Ctrl-C** (terminal) | Ends the session: both arms are lowered to rest, then switched off |

Your hand's movement is scaled by `YAM_SCALE` (2/3) at the tool; its rotation is 1:1. The tool's top speed is the speed
level (`YAM_SPEED`, below).

The terminal prints a line whenever something changes, for example
`left: ACTIVE following grip=following | right: ARMED idle grip=holds ... | ep=recording saved=1`.
Per hand, the first word is the clutch:

| Clutch | Meaning |
|---|---|
| `WAIT_RELEASE` | Release the side grip first (it was held at the start, or the arm was let go) |
| `ARMED` | Ready: press the side grip to move |
| `ACTIVE` | The arm follows your hand |
| `STALLED` | The link from your PC to the lab host stalled. Keep holding the side grip: it resumes by itself within 2 s, from where your hand is then (no jump). Longer than 2 s: release and press again |
| `FAULT_LATCHED` | Something on the headset changed (e.g. its view was re-centred). Check, then press Enter |

The second word is the arm: `following`, `held` (a limit stopped it; the notes in brackets say which), `idle`,
`let go`. The usual notes:

| Note | What to do |
|---|---|
| `speed limit`, `waiting for the arm` | Nothing: you moved faster than the arm may; it catches up |
| `at the box edge`, `out of reach` | Each arm has a box it may move in; move back inside |
| `too close: ...` | The two arms, or an arm and the table, are within 1-3 cm (by speed level); the arm refuses moves that bring them closer, and follows moves that take them apart |
| `joint N at its end stop` | That joint is at its limit; turn or move the other way |
| `wrist at its limit: following the position, the gripper angle N deg short` | Your hand turned further than the wrist can. The gripper still goes where your hand goes; only its angle stops short. Turn your hand back to get the angle back |
| `would take the gripper away from the hand ...: held` | No pose the arm can reach is closer to your hand than where it is; the arm holds. Turn or move your hand back |
| `joint N would swing ... at once (another arm posture)` | The pose needs the arm to switch posture (a joint flipping); refused. Turn or move your hand back a little |
| `grip=waiting for the trigger to match ...` | After you grip again, the gripper waits until the trigger matches what it holds, so nothing is dropped. The note says how far to press or release |
| `... arm joint N is ... rad from its command ...: both arms stopped where they are` | A real joint did not follow (blocked, or it hit something). Look at the arms, then Enter; then grip again |

## How a session ends

Ctrl-C, `scripts/teleop stop`, no connection from your PC for 10 s, or a failure: every
ending first **lowers both arms to their rest poses** (slowly, under the same clearance rules), closes the grippers
to at most 40% open (so the next start is valid), and only then switches the motors off. The terminal ends with
`Both arms are at rest and the program has ended.`

If an arm cannot reach rest within 40 s, the program keeps holding both arms and the terminal says so. Then
`scripts/teleop stop` lets them go (they drop) or use the e-stop. On the lab host, `/home/dev/yam/yam_stop.sh` kills
the programs and switches all 14 motors off (the arms drop). **If the program dies or the power is cut, the arms go
limp and drop**: the motors' watchdog is on.

## Demos (`yam-record`)

As on the xArm: a demo still recording after `DEMO_MAX_MIN` (settings) is saved and closed; any end of the session
saves the one in progress before the arms are lowered (the lowering is never part of a demo); recording needs
`MIN_FREE_GB` free on the lab host. Afterwards the videos are made on the lab host and the demos are copied to
`data/episodes/<session>/` on your PC, without the raw camera files (which stay on the lab host,
`~/quest-xarm-teleop-dryrun/episodes/<session>/`; `scripts/ops/episodes_fetch.sh <session>` copies them).

Each demo (`episode_0001/`, ...):

| File | What |
|---|---|
| `steps.jsonl` | one row per control step (50 Hz). For each arm, `left` and `right`: clutch and arm state, commanded and measured joints with the gripper opening (`q_cmd`, `q_meas`; 0 = closed, 1 = open), the tool pose from the measured joints (`tcp_p`, `tcp_q`: metres, quaternion x y z w, world frame), the tool target, the gripper command, trigger, side grip, the controller's pose, and the notes |
| `view_rs_color.mp4` | the RealSense video, for watching |
| `cam_rs_color.frames.jsonl` (+ `.mjpeg` on the lab host) | every RealSense frame with its time on the rows' clock |
| `events.jsonl`, `meta.json` | clutch changes and faults; duration, following time, travel, gripper range, camera statistics |

`session.json` describes every field (format `quest-yam-bimanual-episode-v1`). The world frame: origin on the table
midway between the two bases, x away from the operator, y to the left, z up. Only the RealSense is recorded
(`YAM_RECORD_CAMS`); the wrist webcams are shown on the camera page but not recorded yet.

## Known limitations

- **The wrist's range is small** (joint 4: -97 to +90 degrees). Past it, the gripper follows your hand's position
  but not its full angle (since 2026-11-18; before, the whole arm was held back, 26% of the time at level 15).
- **Speed**: the real arms have run only at 7.5 cm/s so far. The faster levels pass every check in the simulation and
  are to be tried on the arms one level at a time (`YAM_SPEED`, below).
- **Slow links pause the arms.** Over a slow connection to the lab host, short stalls show as `STALLED` (it resumes
  by itself); a stall over 8 s ends the session. A wired connection helps.
- **The wrist webcams** are not recorded, and their pictures come at about 1.5 per second (from the lab host's
  `/home/dev/yam/yamcam.py`).
- **The 3D view** shows the commands, not the real arms.

## When something goes wrong

| What you see | What to do |
|---|---|
| `NOT STARTED. The YAM program says: ... gripper reads ... of its stroke` | A gripper was more than ~60% open at start-up. Close it below half open, then start again |
| `NOT STARTED ...` naming a motor that did not answer | A CAN hiccup; usually fine on the next start. If it repeats, have the CAN links checked |
| `a YAM session is already running on the lab host` | Someone's session is still running: `scripts/teleop status` shows it; it must end first |
| `another program has the real YAM arms open` | Another program (not a session of this tool) has the arms. Find out whose before doing anything |
| `The YAM program is ending by itself: no connection from the operator's PC for 10 s` | The link to the lab host was lost; the arms were lowered to rest. Start again; check your network |
| `WARNING: ... COULD NOT LOWER BOTH ARMS TO REST` | The arms are still held. Look at them; `scripts/teleop stop` lets them go (they drop), or use the e-stop |
| An arm does not follow although `ACTIVE` | Read the note in brackets (table above) |

## Settings

In `config/teleop_session.conf` (shared with the xArm): `YAM_SPEED`, `YAM_SCALE`, `OPERATOR_YAW`, `DEMO_MAX_MIN`,
`MIN_FREE_GB`.

`YAM_SPEED` is the speed level, named by the tool's top speed in cm/s (the simulation uses the same level, so you can
practise at it first):

| Level | Tool | Turn | Joints | Gripper | Gap kept (arms, table) | State |
|---|---|---|---|---|---|---|
| 7.5 | 7.5 cm/s | 22 deg/s | 0.75 rad/s | 1.0/s | 1 cm | used on the arms since 2026-11-17 |
| 15 | 15 cm/s | 45 deg/s | 1.5 rad/s | 2.0/s | 1 cm | simulation checks pass; not tried on the arms |
| 30 | 30 cm/s | 90 deg/s | 2.5 rad/s | 3.0/s | 2 cm | simulation checks pass; not tried on the arms |
| 50 | 50 cm/s | 150 deg/s | 3.5 rad/s | 4.0/s | 3 cm | simulation checks pass; not tried on the arms |

Go up one level at a time, with `yam-try` (no recording), small movements first, someone at the e-stop. The faster
levels keep a wider gap (between the arms, and from an arm to the table; fingertips may still touch the table)
because a real arm trails its command (by 0.04-0.12 s at 7.5 cm/s: up to 5 cm at
50 cm/s) and the clearance is checked on the commands. A reaching human hand moves at roughly 0.5-1 m/s.

In the
environment: `YAM_VIEWS` (cameras on the page), `YAM_RECORD_CAMS` (cameras recorded). Where the arms stand, their
boxes and every limit: `scripts/yam_setup.py`. The arms' measured gripper limits and rest poses:
`/home/dev/yam/yam_hardware.json` on the lab host (written in the bring-up; a session refuses to start without it).

---

## How it works

```
Quest (USB) -> your PC: quest_pose_stream_sender.py --SSH tunnel--> lab host: yam_live.py (I2RT's Python)
                                                                       yam_teleop:  clutch + hand-to-tool, per hand
                                                                       yam_control: limits, IK, clearance, tracking
                                                                       yam_real / yam_sim: the arms
```

Each control step (50 Hz), per hand: the clutch (`teleop_input.Engagement`, the xArm's rules) decides whether the arm
follows. While it does, the tool target is the tool pose at engagement plus the hand's movement since then (scaled
by 2/3, in the world frame) and the hand's rotation (1:1). `BimanualControl` limits that target, solves the IK
(mink) from the last command, checks both arms together against the clearance model (MuJoCo), and commands the
joints. On the real arms I2RT's own thread sends the commands to the motors.

## Files

| File | Runs on | What |
|---|---|---|
| `scripts/yam_setup.py` | any PC | where the arms stand (bases 0.50 m apart, measured), their boxes, every limit |
| `scripts/yam_limits.py` | any PC | target limiter (box, speed, jump, lag) and joint step guard (end stops, posture switch, joint speed) |
| `scripts/yam_control.py` | any PC | one control step for both arms: clutch, limits, IK, keeping clear, tracking fault, lowering to rest |
| `scripts/yam_input.py` | any PC | both controllers from the Quest stream; each judged on its own controller |
| `scripts/yam_teleop.py` | any PC | per hand: clutch, stall resume, hand-to-tool mapping, gripper pickup |
| `scripts/yam_sim.py` | lab host, I2RT's Python | two simulated arms: IK (mink), clearance model (MuJoCo), commands; the 3D view |
| `scripts/yam_real.py` | lab host, I2RT's Python | the real arms behind the same interface (opening them enables the motors) |
| `scripts/yam_live.py` | lab host, I2RT's Python | the live program: Quest stream in, simulated (`--arms sim`) or real (`--arms real`) arms, lowering to rest, the arms lock |
| `scripts/yam_recorder.py` | lab host | demo recording for two arms (the xArm's recorder with its own row) |
| `scripts/yam_replay.py` | lab host, I2RT's Python | replays a recorded session (`data/sessions/src_*.jsonl`) through the simulated arms |
| `scripts/yam_sim_check.py` | lab host, I2RT's Python | drives the simulated arms through every safety rule; PASS/FAIL per rule |
| `scripts/ops/yam_session.sh` | your PC | one session, started by `scripts/teleop yam-sim`, `yam-try` or `yam-record` |

On the lab host, from the installed copy:

```
/home/dev/yam/i2rt/.venv/bin/python scripts/yam_sim_check.py
/home/dev/yam/i2rt/.venv/bin/python -m unittest tests.test_yam_sim
```

Tests on any PC: `tests/test_yam_control.py`, `test_yam_teleop.py`, `test_yam_real.py`, `test_yam_recorder.py`.

## Safety rules (every control step)

1. A hand target that jumps more than 8 cm or 40 deg in one step is a tracking glitch: the arm is let go (release
   and grip again).
2. The target is held inside the arm's box (an arm that starts outside may come back in, never go further out),
   moves at most at the speed level's tool and turn speeds, speeds up and slows down smoothly (full speed in 0.15 s,
   slowing so it stops on the target without overshoot), and never runs more than 4 cm / 20 deg ahead
   of the arm.
3. The IK solves from the last command, keeping 0.05 rad off the arm joints' end stops. When the full pose is out of
   reach, a second answer lets the gripper angle give way to the position; of the answers that pass rule 4, the one
   closest to the hand is taken, and a step that would take the gripper further from the hand than it is (beyond
   5 mm) is not taken at all (the IK alone would trade position for angle: in simulation it drifted 60 cm away).
   When every answer is refused (a posture switch near full stretch), a target 40% of the way is tried, so the arm
   creeps on to the edge of its reach.
4. Joints stay 0.05 rad off their end stops; a step that would swing a joint more than 0.35 rad at once (the IK
   switching to another arm posture) is refused; faster than the level's joint speed is slowed, keeping
   the path.
5. The two arms are checked together: no move may bring the arms within 1-3 cm (by level) of each other, an arm (other than its
   base) that close to the table, a fingertip into the table, or an arm into itself, unless it moves them apart.
   The other arm still moves if its own move is fine.
6. No fresh hand input (older than 400 ms) for an engaged arm: it is let go and stays where it is. If that was a
   link stall and the side grip stayed held, it resumes within 2 s, re-anchored where hand and tool are then.
7. Real arms only: a joint measured further from its command than 0.10 rad plus 0.25 s of its fastest commanded
   speed in the last 0.5 s stops both arms where they are, latched (Enter clears it). Measured 2026-11-17: real
   joints trail their command by 0.04-0.12 s and stayed within half of this allowance.
8. The gripper (opening 0 = closed, 1 = open, I2RT's convention) moves at most the level's gripper speed.
9. Only one program may have the real arms open (`/tmp/yam_real_arms.lock` on the lab host).

All the numbers are in `yam_setup.py` (`Limits`, and `SPEEDS` for the levels). `yam_sim_check.py --speed <level>`
checks one level.

## The real arms: what the hardware does

Read from I2RT's code (i2rt 234d895) and, read-only, from the motors on 2026-11-17:

- Opening an arm enables its motors. `yam_real.py` opens it holding every joint where it is, so nothing moves at
  start-up; I2RT refuses to start when a joint reads outside its limits (a wrong zero position).
- The LINEAR_4310 gripper calibrates itself at start-up by DRIVING to its ends, unless its limits are given. Its
  limits were measured once in the bring-up and are given every time (`yam_hardware.json`).
- Every motor's watchdog is on (TIMEOUT 8000): when commands stop, the motor goes limp. I2RT's own thread keeps
  commanding while the program runs. I2RT's `close()` sets every torque to zero. Hence the lowering to rest.
- At every start-up I2RT shifts a motor's zero by a full turn when it reads outside +-pi. The gripper's stroke is
  ~5 rad (right arm: closed +0.001, open -5.069 rad), so jaws more than ~60% open at start-up come back 2*pi off
  and the measured limits no longer apply. `yam_real.py` reads each gripper's opening right after opening the arm,
  before any command: outside 0..1 it switches both arms off again and refuses.
- There is no controller-side fence or error stop as on the xArm: the rules above and the e-stop are all.

The bring-up (2026-11-17, by the lab host's own agent session, each step with the operator's approval): joint zero
positions checked, each arm enabled holding its pose, both grippers calibrated, rest poses confirmed, and
`/home/dev/yam/yam_hardware.json` written (format in `scripts/yam_real.py`).

## Findings

- Simulation: the IK takes under 1 ms; a whole step for both arms (IK, limits, clearance check) 1-2 ms, worst ~5 ms
  of the 20 ms available. Pointing straight down, the grasp point cannot go much above 20 cm, so top-down grasps
  happen low over the table. I2RT's simulated arm puts the joints straight at each command (no motor dynamics): the
  simulation checks the decisions, not how smoothly real motors follow.
- Replaying 332 s of a 2026-11-16 xArm session through the simulated arms: every release honoured, no clearance
  problem; the operator's wrist turns reach the YAM's joint 4 end stop (its wrist range is smaller than the xArm's).
- First real run (2026-11-17): stopped by the tracking check on a fixed 0.15 rad (right joint 4 trailing at
  0.75 rad/s); replaced by the speed-aware allowance (rule 7), then tightened from the measured joints.
- The app's start-up reference notices latched both hands; the session now acknowledges them once by itself.
- The gripper waited for a 60% trigger press after the lowering left it 40% open; a gripper not yet moved in the
  session now follows at once.
- Link stalls (2026-11-17 evening, the operator's PC to the lab host): answer times up to 2 s and more, while the lab
  program ran normally (every step 20 ms). The PC's sender gave up at 2 s, ending four sessions in a row; YAM
  sessions now allow 8 s (`--ack-timeout 8`; the xArm keeps 2 s). Stalls over 400 ms also let the arm go mid-motion
  (23 times in 5 minutes for one hand): hence the stall resume (rule 6).
- "Too close" stops were frequent while the model had the bases 0.35 m apart (an estimate); measured 0.50 m.
- Level 15 on the arms (2026-11-18, 9 min): real joints within 58% of the tracking allowance. Engaged time spent at a
  joint end stop 28% (joint 4, the right wrist sat on its -94 deg limit over half the time), at the speed limit 26%,
  out of reach 24%, at a box edge 16%, the gripper waiting for the trigger 12%. Hence the position-first IK (rule 3)
  and the smooth starts and stops (rule 2).
