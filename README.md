# quest-xarm-teleop

Remote teleoperation of an xArm6 with a custom (Dynamixel) gripper from a Meta Quest 3S controller.
The operator is remote: the Quest is connected by USB to a local PC, and the robot is reached
through a lab host over SSH tunnels.

**Status (2026-10-12):** live teleoperation of the real arm and gripper works and records demos
(arm pose, joints, targets, jaws and two camera videos per episode). Day-to-day use is one command,
`scripts/teleop`; see `docs/teleop_usage.md`. Step-by-step history and measurements are in `reports/`.

## Safety
The rig is shared. `scripts/teleop record` and `scripts/teleop try` MOVE THE ARM. They ask go/no-go
questions first: someone must be on site, clear of the robot, with the physical stop in reach, and nothing
else may be commanding the arm. The software never clears controller errors and never changes the
controller's safety settings (fence, reduced mode); those are lab-side decisions. Ctrl-C and
`scripts/teleop stop` are software stops, not the physical stop.

## Layout
| Path | What |
|---|---|
| `scripts/quest_pose_*.py`, `quest_transport.py`, `quest_stream_*.py` | Quest telemetry decoder and the versioned link to the lab host (`docs/checkpoint5_pose_transport_v2.md`) |
| `scripts/teleop_*.py` | Adapter: engagement clutch, anchored-translation mapping, limits, scheduler, fake backend, dry-run entry point |
| `scripts/gripctl_client.py`, `live_gripper_test.py` | Index trigger to the real gripper through the lab's `gripctl` service |
| `scripts/xarm_readonly_preflight.py` | Read-only inspector of the arm's configuration (wire-level allowlist of read registers) |
| `scripts/xarm_command_link.py`, `xarm_live_backend.py` | Commanding link to the arm (four validated writes only: enable, mode, state, line move inside a hard box) and the adapter backend that uses it |
| `scripts/arm_first_move_test.py` | First supervised movement test: one small move, queue-or-replace, and stop behaviour |
| `scripts/lab_cam_relay.py` | Low-rate, view-only relay of the lab cameras for the remote operator |
| `scripts/teleop`, `config/teleop_session.conf` | The one command for sessions, and its settings |
| `scripts/live_teleop.py`, `episode_*.py` | Live teleop program (lab side) and episode recording |
| `scripts/ops/` | Session scripts with spoken cues |
| `tests/` | Offline tests; none of them touches hardware |
| `patches/` | Our changes to the Quest OpenXR reader app (`oculus_reader` fork), as patches |
| `reports/`, `docs/` | Step-by-step reports and protocol documentation |

Not in the repository: `data/` (recordings), `external/` (the Quest app sources and other cloned repositories).

## Tests
```
python3 -m unittest discover -s tests
```
Tests that replay saved recordings are skipped when `data/` is absent. The SDK loopback tests
need the xArm Python SDK (or `XARM_SDK_SRC` pointing at a copy of its source).
