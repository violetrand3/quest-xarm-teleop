# Bimanual YAM teleop (in progress: simulation only)

Two I2RT YAM arms on the lab host (`can_left`, `can_right`), driven by the two Quest controllers: the left
hand drives the left arm, the right hand the right arm. It shares the Quest side, the recorder and the launcher
with the xArm teleop, and nothing else: the YAM code never imports the xArm code, and the reverse (a test checks).

**Status (2026-11-17): step 1 of 4 done, in simulation. Nothing here can move a real arm:** the arm layer
(`yam_sim.py`) creates every robot with `sim=True` and has no path to the CAN links.

| Step | What | State |
|---|---|---|
| 1 | Two simulated arms driven to tool targets through the IK, with the safety rules below | done |
| 2 | Replay recorded hand motion (xArm sessions) through the simulated arms | next |
| 3 | Live Quest -> simulated arms: both controllers, one clutch per hand, a browser view | |
| 4 | Real arms: safety review, gains, gripper calibration, base placement, first engagement with someone at the arms | needs the lab |

## Files

| File | Runs on | What |
|---|---|---|
| `scripts/yam_setup.py` | any PC | where the arms stand, their boxes, every limit (one place to change them) |
| `scripts/yam_limits.py` | any PC | target limiter (box, speed, jump, lag) and joint step guard (end stops, posture switch, joint speed) |
| `scripts/yam_control.py` | any PC | one control step for both arms: clutch, limits, IK, keeping clear, tracking fault |
| `scripts/yam_sim.py` | lab host, I2RT's environment | two simulated arms: IK (mink), clearance model (MuJoCo), commands |
| `scripts/yam_sim_check.py` | lab host, I2RT's environment | drives the simulated arms through every safety rule; PASS/FAIL per rule |

Run the check on the lab host (from a synced copy):

```
/home/dev/yam/i2rt/.venv/bin/python scripts/yam_sim_check.py
```

Tests: `tests/test_yam_control.py` (any PC) and `tests/test_yam_sim.py` (lab host, skipped elsewhere).

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

- Arm model and gripper: standard YAM with the LINEAR_4310 gripper (I2RT's defaults).
- Placement: both bases on the table, 0.60 m apart, both facing away from the operator. The boxes in
  `yam_setup.py` follow from that.

## Found in simulation

- The IK takes under 1 ms; a whole step for both arms (IK, limits, clearance check) 1-2 ms, worst ~5 ms of the
  20 ms available.
- Pointing straight down, the grasp point cannot go much above 20 cm (the wrist would be out of reach), so
  top-down grasps happen low over the table.
- I2RT's simulated arm puts the joints straight at each command (no motor dynamics): the simulation checks the
  decisions, not how smoothly real motors follow.
