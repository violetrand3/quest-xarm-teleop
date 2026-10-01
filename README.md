# quest-xarm-teleop

Remote teleoperation of an xArm6 with a custom (Dynamixel) gripper from a Meta Quest 3S controller.
The operator is remote: the Quest is connected by USB to a local PC, and the robot is reached
through a lab host over SSH tunnels.

**Status (2026-10-01):** controller input, the link to the lab, the clutch/limits adapter and the
gripper path work. The adapter has only driven a fake arm; the real arm command path is deliberately
blocked (`scripts/teleop_backend_xarm.py`). See `reports/` for what was measured at each step.

## Safety
The rig is shared. Nothing here enables motors, clears errors or moves the arm. Any hardware
command needs someone on site, clear of the robot, with the physical stop. Read
`reports/checkpoint6_commissioning_checklist_20261001.md` before changing that.

## Layout
| Path | What |
|---|---|
| `scripts/quest_pose_*.py`, `quest_transport.py`, `quest_stream_*.py` | Quest telemetry decoder and the versioned link to the lab host (`docs/checkpoint5_pose_transport_v2.md`) |
| `scripts/teleop_*.py` | Adapter: engagement clutch, anchored-translation mapping, limits, scheduler, fake backend, dry-run entry point |
| `scripts/gripctl_client.py`, `live_gripper_test.py` | Index trigger to the real gripper through the lab's `gripctl` service |
| `scripts/xarm_readonly_preflight.py` | Read-only inspector of the arm's configuration (wire-level allowlist of read registers) |
| `scripts/lab_cam_relay.py` | Low-rate, view-only relay of the lab cameras for the remote operator |
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
