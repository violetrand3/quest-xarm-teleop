# Bimanual YAM teleop (simulation working; real arms written, waiting for the bring-up)

Two I2RT YAM arms on the lab host (`can_left`, `can_right`), driven by the two Quest controllers: the left
hand drives the left arm, the right hand the right arm. It shares the Quest side, the recorder and the launcher
with the xArm teleop, and nothing else: the YAM code never imports the xArm code, and the reverse (a test checks).

**Status (2026-11-17): steps 1-3 done, in simulation; the real-arm path is written but has never run on hardware.**
`yam_sim.py` creates every robot with `sim=True` and has no path to the CAN links; only `yam_real.py` opens the real
arms, used only by `scripts/teleop yam-try` after its questions, and it refuses until the bring-up is recorded.

| Step | What | State |
|---|---|---|
| 1 | Two simulated arms driven to tool targets through the IK, with the safety rules below | done |
| 2 | Replay recorded hand motion (xArm sessions) through the simulated arms | done |
| 3 | Live Quest -> simulated arms: both controllers, one clutch per hand, a browser view | done; tried with the headset on 2026-11-17 |
| 4 | Real arms: the code is written (`scripts/teleop yam-try`, below), never run on hardware; waits for the bring-up | needs the lab |

## Trying it (simulation)

```
scripts/teleop yam-sim
```

On the PC the headset is plugged into, after `scripts/teleop sync` has installed this version on the lab host.
It starts the simulation on the lab host, the sender and the headset app, and prints when the controllers are
found. Watch the two simulated arms at **http://127.0.0.1:18767/** (a 3D view: drag to turn it).

- Switch BOTH controllers on. Left controller -> left arm, right controller -> right arm.
- Hold the side grip (middle finger) to move that arm, release to stop. A grip already held when the session
  starts must be released once first.
- While the side grip is held, the index trigger closes that arm's gripper (as on the xArm); released, the gripper
  stays as it is. The first time in a session it follows at once; after that, when you grip again it waits until
  the trigger matches what the gripper holds (the terminal says how far to press), so nothing held is dropped.
- Enter acknowledges a latched problem (e.g. after the headset's view was re-centred); Ctrl-C ends. The
  simulation on the lab host also ends by itself after 60 s without a connection from this PC.
- The terminal prints a line whenever a hand's state changes, with what limited it (box edge, speed limit,
  "too close", an end stop, ...). Everything is also logged on the lab host: `~/quest-xarm-teleop-dryrun/logs/yam_<session>.*`.

## Files

| File | Runs on | What |
|---|---|---|
| `scripts/yam_setup.py` | any PC | where the arms stand, their boxes, every limit (one place to change them) |
| `scripts/yam_limits.py` | any PC | target limiter (box, speed, jump, lag) and joint step guard (end stops, posture switch, joint speed) |
| `scripts/yam_control.py` | any PC | one control step for both arms: clutch, limits, IK, keeping clear, tracking fault |
| `scripts/yam_sim.py` | lab host, I2RT's environment | two simulated arms: IK (mink), clearance model (MuJoCo), commands |
| `scripts/yam_sim_check.py` | lab host, I2RT's environment | drives the simulated arms through every safety rule; PASS/FAIL per rule |
| `scripts/yam_input.py` | any PC | both controllers from the Quest stream; each judged on its own controller |
| `scripts/yam_teleop.py` | any PC | per hand: clutch (the xArm's rules), hand-to-tool mapping, gripper pickup |
| `scripts/yam_replay.py` | lab host, I2RT's environment | replays a recorded session (`data/sessions/src_*.jsonl`) through the simulated arms |
| `scripts/yam_live.py` | lab host, I2RT's environment | the live program: Quest stream in, simulated (`--arms sim`) or real (`--arms real`) arms, browser view, lowering to rest |
| `scripts/yam_real.py` | lab host, I2RT's environment | the real arms behind the same interface (opening them enables the motors) |
| `scripts/yam_recorder.py` | lab host | demo recording for two arms (the xArm's recorder with its own row) |
| `scripts/ops/yam_session.sh` | operator's PC | one session, started by `scripts/teleop yam-sim`, `yam-try` or `yam-record` |

Run the check on the lab host (from a synced copy):

```
/home/dev/yam/i2rt/.venv/bin/python scripts/yam_sim_check.py
```

Tests: `tests/test_yam_control.py`, `tests/test_yam_teleop.py`, `tests/test_yam_real.py` (any PC) and
`tests/test_yam_sim.py` (lab host, skipped elsewhere).

## Safety rules (per control step, 50 Hz)

1. A hand target that jumps more than 8 cm or 40 deg in one step is a tracking glitch: the arm is let go
   (release and grip again).
2. The target is held inside the arm's box (an arm that starts outside may come back in, never go further out),
   moves at most 0.15 m/s and 45 deg/s, and never runs more than 4 cm / 20 deg ahead of the arm.
3. The IK solves from the last command; a target out of reach is followed as far as it goes.
4. Joints stay 0.05 rad off their end stops; a step that would swing a joint more than 0.35 rad at once (the IK
   switching to another arm posture) is refused; faster than 1.5 rad/s is slowed, keeping the path.
5. The two arms are checked together: no move may bring the arms within 1 cm of each other, an arm (other than
   its base) within 1 cm of the table, a fingertip into the table, or an arm into itself, unless it moves them
   apart. The other arm still moves if its own move is fine.
6. No hand input for an engaged arm: it is let go and stays where it is.
7. Real arms only (sim reads back its command): a joint measured more than 0.15 rad from its command stops both
   arms where they are, latched.
8. The gripper (opening 0 = closed, 1 = open, I2RT's convention) moves at most 2.0 per second.

All the numbers are in `yam_setup.py`. They are first guesses, to be tuned in step 3 and checked on the arms.

## Assumptions to confirm on the rig

- Arm model: the standard YAM (I2RT's default). The gripper, LINEAR_4310, was confirmed on 2026-11-17.
- Placement: both bases on the table, both facing away from the operator, who stands behind them (confirmed);
  0.50 m apart centre to centre (measured by the operator 2026-11-17; BASE_SPACING in
  `yam_setup.py`). The boxes follow from it.

## The real arms (`scripts/teleop yam-try`): NOT RUN YET

What the hardware does (read from I2RT's code and, read-only, from the motors on 2026-11-17):

- Opening an arm enables its motors. `yam_real.py` opens it holding every joint where it is, so nothing moves at
  start-up; I2RT refuses to start when a joint reads outside its limits (a wrong zero position).
- The LINEAR_4310 gripper calibrates itself at start-up by DRIVING to its ends, unless its limits are given; and
  its zero is unreliable today. So its limits are measured once in the bring-up and given every time.
- Every motor's watchdog is on (TIMEOUT 8000): when commands stop, the motor goes limp. I2RT's own thread keeps
  commanding while the program runs; **if the program dies or the power is cut, the arms go limp and drop.**
- I2RT's `close()` sets every torque to zero: the arms go limp. So every way a session ends (Ctrl-C,
  `scripts/teleop stop`, the time limit, no connection from the operator's PC for 10 s, a failure of the control
  step) first **lowers both arms to their rest poses** at 0.3 rad/s, under the same clearance rules as teleop,
  and only then lets them go. If that cannot finish within 40 s, the arms stay held where they are and the
  terminal says so; `scripts/teleop stop` then lets them go (they drop) or the e-stop is used.
- At every start-up I2RT shifts a motor's zero by a full turn when it reads outside +-pi. The gripper's stroke is
  ~5 rad (right arm, measured 2026-11-17: closed +0.001, open -5.069 rad), so jaws more than ~60% open at start-up
  come back 2*pi off and the measured limits no longer apply. `yam_real.py` reads each gripper's opening right
  after opening the arm, before any command: outside 0..1 it switches both arms off again and refuses (close
  that gripper and start again). Lowering to rest closes each gripper to at most 40% open, so the next start-up
  is valid.
- There is no controller-side fence or error stop as on the xArm: the rules above and the YAM e-stop are all.

**Bring-up first** (done by the linux2 session, each step with the user's explicit approval given to it):

1. Check each arm's joint zero positions: the joint readings against what the arm looks like.
2. Enable one arm holding its pose with someone watching, then the other.
3. Calibrate both grippers (I2RT's calibration, which drives them) and note their [closed, open] limits.
4. Confirm each arm's rest pose: where it sits safely when it goes limp.
5. Write `/home/dev/yam/yam_hardware.json` on the lab host (format in `scripts/yam_real.py`); `yam-try` refuses
   to start without it.

**Then the first run:** `scripts/teleop yam-try`, half the simulation's speed limits (the tool at most 7.5 cm/s),
after its go/no-go questions: someone at the arms with the e-stop in reach, nobody near the arms and nothing under
them, nothing else commanding them. Start with small, slow movements of one hand; the arms work low over the table.

Watch the arms on the camera page (`yam-try` and `yam-sim` start it and print its address, http://127.0.0.1:18091/):
the RealSense, which sees both arms, and the two wrist webcams (`wrist_c920`, `wrist_emeet`, served on the lab host
by /home/dev/yam/yamcam.py on 127.0.0.1:8096; about 1.5 frames/s from that program on 2026-11-17). YAM_VIEWS in the
environment changes the list. The 3D page shows what the arms are COMMANDED to do, not what they do.

Tracking: a joint measured too far from its command stops both arms where they are (the only check that notices a
blocked arm). A moving joint trails its command, so the allowance is 0.10 rad plus 0.25 s of the joint's fastest
commanded speed in the last 0.5 s. Set from the measured joints of a real run (2026-11-17, 86 s at half speed):
still within 0.044 rad, lag 0.04-0.12 s per joint, at worst 51% of the allowance. (The first real run had stopped
on a fixed 0.15 rad: right joint 4 trailing at 0.75 rad/s.) Enter clears the stop once you have looked; the arms keep holding, and a hand is released and pressed
again to move its arm. Each step logs the commanded ("q") and measured ("meas") joints, to tune this from data.

Stopping: release a side grip (that arm stops where it is); Ctrl-C or `scripts/teleop stop` (both arms are lowered
to rest, then let go); on the lab host `/home/dev/yam/yam_stop.sh` (the linux2 session's emergency software stop:
it kills the programs and switches all 14 motors off, so the arms drop); the e-stop (the arms drop).
Before `yam-try`, both arms must rest folded on their bases: a refused start-up switches them off again.

## Recording demos (`scripts/teleop yam-record`)

`yam-try` with demo recording, as `record` on the xArm: right thumb **A** starts a demo and, pressed again, saves it;
**B** discards the one in progress. A demo still recording after DEMO_MAX_MIN (settings) is saved and closed; any end
of the session saves the one in progress before the arms are lowered (the lowering is never part of a demo).
Recording needs MIN_FREE_GB free on the lab host. Afterwards the videos are made on the lab host and the demos copied
to `data/episodes/<session>/` (without the raw camera files), as for the xArm.

Each demo (`episodes/<session>/episode_0001/` on the lab host):

| File | What |
|---|---|
| `steps.jsonl` | one row per control step (50 Hz): for each arm, `left` and `right`: clutch and arm state, commanded and measured joints with the gripper opening (`q_cmd`, `q_meas`), the tool pose from the measured joints (`tcp_p`, `tcp_q`, metres, world frame), the tool target, the gripper command, trigger, side grip, the controller's pose and the notes (what limited the arm) |
| `cam_rs_color.mjpeg` + `.frames.jsonl` | the RealSense (sees both arms), every frame with its time on the rows' clock |
| `events.jsonl`, `meta.json` | clutch changes and faults; duration, following time, travel, gripper range, camera statistics, problems |

`session.json` describes every field (format `quest-yam-bimanual-episode-v1`). The cameras saved are YAM_RECORD_CAMS
(default `rs_color`; the wrist webcams are shown on the camera page but not recorded yet).

Tested 2026-11-17 in simulation with a recorded xArm demo session played back in place of the headset (its own A/B
presses): 1 demo saved, 2 discarded, as pressed; the saved one 23.9 s, 1194 rows at 50 Hz, the arms following 18.4 s,
804 RealSense frames.

## Hand to arm

While a hand's grip is held, its arm's tool follows the hand from where both were when the grip was pressed:
the hand's movement scaled by 0.667 (as on the xArm), the hand's rotation 1:1. The axes are the xArm's (the
operator faces the way the arms face; OPERATOR_YAW in config/teleop_session.conf applies as on the xArm). The gripper follows the
trigger only while the side grip is held (as on the xArm). Once you have moved a gripper with the trigger, it follows
again after a release only when the trigger has come within 0.1 of it, so an arm holding something does not drop it
when the hand comes back with the trigger released; a gripper you have not moved yet in the session follows at once.

## Found in simulation

- The IK takes under 1 ms; a whole step for both arms (IK, limits, clearance check) 1-2 ms, worst ~5 ms of the
  20 ms available.
- Pointing straight down, the grasp point cannot go much above 20 cm (the wrist would be out of reach), so
  top-down grasps happen low over the table.
- Replaying 332 s of the 2026-11-16 xArm session (the right hand, mirrored for the left): both arms engaged
  221 s, every release honoured, no clearance problem; the operator's wrist turns reach the YAM's joint 4 end
  stop (its wrist range is smaller than the xArm's). End to end through the real sender and tunnel (a recorded
  session played back in place of the headset): all 3131 packets accepted, both arms followed, both let go
  when the stream stopped.
- I2RT's simulated arm puts the joints straight at each command (no motor dynamics): the simulation checks the
  decisions, not how smoothly real motors follow.

Link stalls (2026-11-17 evening): four yam-record sessions in a row ended with "no connection from the operator's PC
for 10 s". The lab program ran normally throughout (every control step 20 ms); the operator's PC to the lab host had
answer times up to 1.6-1.9 s and then over 2 s, with the camera page closed too. The PC's sender gave up at 2 s ("ack
timeout"). YAM sessions now pass `--ack-timeout 8` (the xArm keeps 2 s). During a stall the poses go stale, the
clutch lets go and the arms hold; press the side grip again afterwards.

Short stalls also let the arm go mid-motion (input older than 400 ms): in yamrec_20261117-223800 the left hand was let
go 23 times in ~5 min this way, each needing a re-grip. Since then (YAM only, the xArm still needs the re-press) a
hand let go by a stall shows `STALLED` and resumes by itself when fresh input returns within 2 s from the same headset
session and continuity with the side grip held. It re-anchors where the hand and the tool are then (no catching up);
the gripper keeps following the trigger. A release, a longer stall or any other problem needs the usual release
and press.
