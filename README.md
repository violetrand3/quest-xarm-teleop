# quest-xarm-teleop

Remote teleoperation of the lab's robot arms from a Meta Quest 3S, with demo recording:

- **xArm6** with the lab's Dynamixel gripper: the right controller drives the arm.
- **Two I2RT YAM arms** (bimanual): the left controller drives the left arm, the right controller the right arm.

The operator can be anywhere: the Quest is plugged into the operator's PC by USB, and the robots are reached
through the lab host (linux2) over SSH. One command, `scripts/teleop`, runs a whole session from that PC.

**Status (2026-11-18):** both work on the real robots and record demos (arm pose, joints, gripper, camera video):
the xArm since 2026-10-12, the YAM arms since 2026-11-17 (`docs/yam_teleop.md` says what is still rough).

## Start here

1. `docs/getting_started.md`: setting up your PC, the headset app and SSH to the lab host, and how the lab host's
   copy of the programs is kept in step with GitHub. Once per PC.
2. `docs/teleop_usage.md`: an xArm session.
3. `docs/yam_teleop.md`: a YAM session.

```
scripts/teleop status        # read-only: robot, cameras, headset, disk, is the lab copy up to date
scripts/teleop record        # xArm, recording demos           (scripts/teleop try: the same without recording)
scripts/teleop yam-sim       # YAM in simulation: nothing moves (a good first try)
scripts/teleop yam-record    # YAM, recording demos            (scripts/teleop yam-try: without recording)
scripts/teleop sync          # install this checkout on the lab host (only a committed, pushed version)
```

`scripts/teleop` with no arguments lists every command.

## Safety

The robots are shared. Every command that moves a robot (`record`, `try`, `yam-record`, `yam-try`) asks go/no-go
questions first: someone is at the robot with its stop button in reach, nobody is near it, and nothing else is
commanding it. What you answer is saved with the session. Ctrl-C and `scripts/teleop stop` are software stops, not
the physical stop.

The software never clears an xArm controller error and never changes a controller's safety settings (fence, reduced
mode, the YAM motors' watchdog); those are lab-side decisions. The YAM arms go limp when their program stops, so
every YAM session lowers both arms to rest before it ends.

## Layout

| Path | What |
|---|---|
| `scripts/teleop`, `config/teleop_session.conf` | the one command for sessions, and its settings |
| `scripts/ops/` | the session scripts `scripts/teleop` runs (`teleop_live_session.sh` and `lab_teleop_live_run.sh` for the xArm, `yam_session.sh`, `episodes_*.sh`) |
| **Shared by both robots** | |
| `scripts/quest_pose_*.py`, `quest_transport.py`, `quest_stream_*.py`, `quest_input_monitor.py` | the headset app's controller data, and the versioned link to the lab host (`docs/checkpoint5_pose_transport_v2.md`) |
| `scripts/controller_wait.py` | waits for the controllers at the start of a session and says what is wrong |
| `scripts/teleop_input.py`, `teleop_rotation.py` | the clutch (side grip) and the rotation maths |
| `scripts/episode_recorder.py`, `episode_cameras.py`, `episode_inspect.py` | demo recording (folders, A/B buttons, cameras, limits) and checking recorded demos |
| `scripts/lab_cam_relay.py` | the camera page the operator watches |
| **xArm** | |
| `scripts/live_teleop.py` | the live program on the lab host |
| `scripts/teleop_adapter.py`, `teleop_mapping.py`, `teleop_limits.py`, `teleop_backend*.py`, `teleop_replay.py`, `teleop_dryrun.py` | hand-to-arm mapping, limits, the arm backends, offline replay |
| `scripts/xarm_command_link.py`, `xarm_live_backend.py`, `arm_kinematics.py`, `xarm_readonly_preflight.py` | the commanding link to the arm (a few validated writes only), its wrist geometry, the read-only inspector |
| `scripts/gripctl_client.py`, `live_gripper_test.py` | the gripper, through the lab's `gripctl` service |
| `scripts/arm_first_move_test.py`, `gripper_step_check.py`, `lab_frame_capture.py`, `frames_refdiff.py`, `wrist_frames_measure.py` | commissioning tests and measurements (still used by tests and lab scripts) |
| **YAM** | |
| `scripts/yam_*.py` | the bimanual YAM teleop: setup and limits, control, simulation, real arms, live program, recorder (each listed in `docs/yam_teleop.md`) |
| **Other** | |
| `tests/` | offline tests; none touches hardware |
| `docs/` | setting up and using it; the link protocol |
| `reports/` | step-by-step reports and measurements from building the xArm teleop |
| `patches/` | our changes to the Quest app (an `oculus_reader` fork) |
| `archive/` | one-off test sessions and analyses the reports were made with; not used any more |

Not in the repository: `data/` (recordings and session logs), `external/` (the Quest app sources and other cloned
repositories). The YAM teleop uses I2RT's package on the lab host (`/home/dev/yam/i2rt`, with its own Python).

## The lab host's copy

The lab host runs its own copy of the programs, `~/quest-xarm-teleop-dryrun` (the name is historical), shared by
everyone using the lab account. It is **not a git checkout**, on purpose: the lab host would need a GitHub login for
this private repository, and nobody should edit the programs there. `scripts/teleop sync` installs it from your
checkout, and only a version that is committed and pushed, so anyone can match it with `git pull`. The session
commands refuse to start when it differs from your checkout. Its `README.txt` says which commit it is. Details:
`docs/getting_started.md`.

## Tests

```
python3 -m unittest discover -s tests
```

Tests that replay saved recordings are skipped when `data/` is absent, and the YAM simulation tests off the lab host
(on it: `/home/dev/yam/i2rt/.venv/bin/python -m unittest tests.test_yam_sim`). The SDK loopback tests need the xArm
Python SDK (or `XARM_SDK_SRC` pointing at a copy of its source).
