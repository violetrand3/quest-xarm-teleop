# Robot-backend source inventory (read-only)

Date: 2026-10-01. Source read as **text only** on `linux2` (user `dev`). Nothing was imported or executed, no SDK object was constructed, and no robot, gripper or camera was contacted.

**Everything below is documented code behaviour. None of it was verified on hardware by this inspection**, and configured values (fence, reduced mode, collision sensitivity, TCP offset, gripper calibration) are not assumed to be active on the robot right now.

Network details are redacted (`<xarm-ip>`); service ports are loopback ports named in the code.

**Sources** (plain directories, paths given by you):
- `/home/dev/robotdash/`: the xArm execution code used on this rig.
- `/home/dev/gripctl_src/`: the gripper driver, a copy of the running service's source.
- Not inspected: `/home/dev/cap_train/…/robot-server` (reference only, not what runs here).

Files read, with the first 16 hex characters of their SHA-256 at inspection time:

| File | SHA-256 prefix |
|---|---|
| `robotdash/actuator.py` | `3f8cf6ff202f1311` |
| `robotdash/stepper.py` | `2042b129f035e33e` |
| `robotdash/step_long.py` | `8a2d7d5bdf597d3e` |
| `gripctl_src/gripctl.py` | `e7afc89167f4707e` |

Also read: `ACTION_FRAME_CONTRACT.md`, `verify_move.py`, `reset_and_sweep.py`, `calibrate_handeye.py`, `record.py`, `server.py` (outline), `reconfigure*.py` (SDK calls), `gripper_maps/*`, and the gripctl `README.md` and systemd unit.

## 1. Entry points and the execution wrapper
| Role | Where | Evidence |
|---|---|---|
| Step-by-step web UI (the main path) | `stepper.py`, HTTP on `127.0.0.1:8093` | `stepper.py:387-391` |
| The only place the arm moves in that path | `Robot.execute()` | `stepper.py:171-213` ("the only place the robot moves") |
| Proposals (compute and check, no motion) | `propose_jaws`, `propose_stage`, `propose_nudge`, `propose_policy` | `stepper.py:112-169` |
| Policy action → target pose (pure math) | `Actuator.plan()` | `actuator.py:204-234` |
| Simpler closed-loop CLI runner | `step_long.py main()` | `step_long.py:103-250` |
| Calibration and utility scripts that move the arm | `verify_move.py`, `calibrate_handeye.py`, `reset_and_sweep.py`, `reconfigure.py`, `reconfigure_step.py`, `step_once.py`, `step_seq*.py` | see section 10 |
| Read-only policy dry run | `dryrun.py` (reads the pose, issues no command) | `dryrun.py:2-8, 43-46` |
| Camera dashboard / gripper proxy | `server.py` on `:8090` | `server.py:1-23` |

Inference lives inside the same process: `FlowBCPolicy.predict_ensemble(o, seed=0, n=8).mean(0)` (`stepper.py:154`, `step_long.py:176`).

## 2. Motion commands actually used
- **Cartesian, position mode, blocking:** `arm.set_position(x, y, z, roll, pitch, yaw, speed=…, mvacc=…, wait=True)`. Evidence: `stepper.py:192`; `step_long.py:160, 189`; `verify_move.py:111, 119`; `reset_and_sweep.py:52`.
- **Mode and state sequence around each move:** `motion_enable(True); set_mode(0); set_state(0)` → move → `set_state(4)` (stopped). Evidence: `stepper.py:191-194`.
- **Joint moves** (`set_servo_angle`) appear only in the reconfiguration scripts (`reconfigure.py:43-51`, `reconfigure_step.py:18`).
- **No streaming/servo mode anywhere:** there is no `set_servo_cartesian`, `set_servo_angle_j`, or mode 1/7 usage in `robotdash`.
- **Connection:** `XArmAPI("<xarm-ip>", is_radian=False, do_not_open=False)`. The constructor connects immediately (`stepper.py:78`).

## 3. Absolute or relative, and in which frame
- The SDK call is always given an **absolute base-frame target**. `relative=True` is never used.
- Relative motion is composed in software as a **right-multiply in the end-effector frame**: `T_new = T_cur @ A_ee` (`actuator.py:216-220`), where `A_ee = T_ec @ A_cam @ inv(T_ec)` (`actuator.py:9-15, 200-202`).
- The policy's action frame is the **ARKit camera optical frame** (+x right, +y up, **−z forward**), composed as `T_new = T_old @ delta` (`ACTION_FRAME_CONTRACT.md:20-55`).
- Manual nudges add millimetres directly in the **base frame**: "+x away from base, +y toward the LEFT bin, +z up" (`stepper.py:137-144`).

## 4. Units and orientation
- **xArm pose:** `[x, y, z, roll, pitch, yaw]` in **mm and degrees** (`is_radian=False`). Evidence: `stepper.py:78, 93`; `actuator.py:134-144`.
- **Internal transforms:** translation in metres (`actuator.py:134-137`).
- **Euler convention:** the code *assumes* `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`, and labels it an assumption (`actuator.py:66-69, 102-107`).
- **Policy action:** `[dx, dy, dz]` in metres, rotation as an axis-angle vector in radians, gripper label 0..1 absolute (`actuator.py:147-150`; `ACTION_FRAME_CONTRACT.md:41-55, 78-90`).
- **Angle interpolation** goes "the short way" (`stepper.py:35, 102`). `step_long.py:151-156` records that a raw angle difference once spun the wrist 240° into a collision.

## 5. Tool-centre-point and camera/gripper transforms
- **`T_ec` (camera pose in the EE frame):** only the **rotation** is measured. `R_ce.npy` is saved by `calibrate_handeye.py` ("maps an EE-frame vector into CAMERA coords"). It is loaded as `T_ec[:3,:3] = R_ce.T` with **zero translation** (`stepper.py:80`; `step_long.py:123-124`).
- **The lever arm is unmeasured.** The code says a commanded rotation therefore causes unintended translation at the tool (`actuator.py:159-170`; `calibrate_handeye.py` docstring).
- **No TCP configuration in this code:** `robotdash` never calls `set_tcp_offset`, `set_tcp_load` or `set_world_offset`. Whatever TCP offset the controller holds is not visible in the source and is **unverified**.
- **Reference start pose:** `START = [299.83, 3.43, 461.18, 179.86, -15.1, -6.64]` (`stepper.py:28`; `reset_and_sweep.py:16`).

## 6. Control frequency, blocking and queuing
- Every move is **blocking** (`wait=True`), one step at a time. The arm is put in state 4 between steps (`stepper.py:5-6, 194`).
- **Speeds:** nudge 10 mm/s, policy 15 mm/s, stage 20 mm/s; `mvacc` 100–150 (`stepper.py:132, 143, 166, 192`; `step_long.py:160, 189`).
- **Cycle time:** auto-run is described as "~2 s/step". An observation is 3 frames taken 0.267 s apart, followed by inference (`stepper.py:148-154, 294`).
- **No command queue.** There is one `pending` proposal at a time, and execution is refused if the arm moved more than 1 mm since the proposal (`stepper.py:173-178, 188-189`).

Per-step limits in code:

| Limit | Value | Evidence |
|---|---|---|
| Policy sanity gate (reject) | 12 cm / 0.35 rad | `actuator.py:30-31, 178-185` |
| Scale applied to policy deltas | 0.25 | `actuator.py:32, 187-197`; `stepper.py:81-82` |
| Hard clamp after scaling | 2 cm / 0.05 rad | `actuator.py:33-34` |
| Policy step cap in stepper | 20 mm / 3° | `stepper.py:31, 156` |
| Manual stage cap | 50 mm / 30° | `stepper.py:30, 128` |
| Nudge cap | 10 mm, no rotation | `stepper.py:140` |

## 7. Fence and reduced mode (as written in code)
| Where | x (mm) | y (mm) | z (mm) | Evidence |
|---|---|---|---|---|
| `actuator.FENCE` default | 202–502 | −150–150 | 400–560 | `actuator.py:64` |
| `stepper.FENCE_RUN` (the UI path) | **180–502** | **−300–270** | **270–585** | `stepper.py:29` (comments record user-authorised widenings) |
| `step_long` local fence | 202–502 | −300–150 | 360–585 | `step_long.py:125` |
| Controller reduced-mode TCP boundary | 160–502 | −300–270 | 270–585 | `reconfigure.py:55`: `set_reduced_tcp_boundary([502,160,270,-300,585,270])` + `set_reduced_mode(True)` |

- The software fence in `stepper.py` matches the limits you stated (x 180–502, y −300…270, z ≥ 270).
- The controller boundary written by `reconfigure.py` has **x-min 160, not 180**.
- Collision sensitivity is set to 5 in `reconfigure.py:25`.
- **Whether reduced mode and these boundaries are currently active on the controller is not verified.**

## 8. Custom gripper
- **Driver:** `gripctl.py`, an aiohttp HTTP service that owns the Dynamixel serial bus through `dynamixel_sdk`. The systemd unit runs it with `--serve --host 127.0.0.1 --port 8088`. The code's own default host is `0.0.0.0` (`gripctl.py:382`); the unit overrides it.
- **Command used by the robot code:** `POST /api/do/grip {"value": <percent closed>}`, with `pct = (3033 − ticks) / 1852 × 100` (`stepper.py:117, 183, 196`; `step_long.py:196-199`).
- **Meaning of that command** (`gripctl.py:1566-1610`):
  - Percent closed is linear between the calibrated open position (0 %) and closed position (100 %).
  - It enables torque, sets an effort cap, and writes `goal_position`.
  - **There is deliberately no stall cancel on `grip`.**
- **Effort caps:** closing 60 %, moving 35–40 % (`calibration.example.json`; `gripctl.py:1046-1048`).
- **Other actions used:** `/api/do/torque {"value":"on"}` (`step_long.py:133`) and `/api/do/open` (`reset_and_sweep.py:57`).
- **`stop`** rewrites the goal to the present position (`gripctl.py:1646-1650`).
- **Ticks:** closed **1181**, open **3033** (`actuator.py:44-45`); the "in-distribution" open limit is 1950 for the `sep24_visual` scale (`actuator.py:53-61`).
- **Policy label → ticks:** `gripper_to_ticks` (`actuator.py:245-250`). A newer per-checkpoint `label_to_ticks` needs `gripper_maps/hw_raw_vs_ticks.json`, which was **not present** in the directory listing (`actuator.py:253-267`).
- **Feedback:** `GET /api/state` returns:
  - `values.position` (ticks), `values.force`, `values.torque_enable`, `values.moving`, and others;
  - an `age` field, the snapshot age in seconds;
  - `status.connected`.

  The README says position refreshes about 20–30 times a second (README "How fresh the readings are").
- **`force` is an estimate** (`effort % × force_scale`), not a measurement (README section 3).
- **Known behaviour in comments:** the U2D2 drops off USB for 1–3 s under motion. Callers wait and retry (`stepper.py:235-239`; `step_long.py:197-203`).
- **Force abort used by the runners:** 6 N (`stepper.py:223, 240`; `step_long.py:111, 217`).

## 9. Stop, disconnect, error handling, reconnect
- **Stop:** `Robot.stop()` calls `arm.emergency_stop()`, then `set_state(4)`, clears the pending step, and sets `auto_stop` (`stepper.py:215-220`). It is exposed as `POST /api/stop` (`stepper.py:373`) and the "STOP ARM" button.
- **Always-stop on exit paths:** `finally: arm.set_state(4)` after every execute and auto-run (`stepper.py:207-209, 250-252`) and in `step_long.py:221-226`.
- **Pre-move checks** (`stepper.py:100-110, 186-189, 204`):
  - `get_err_warn_code() == [0, 0]`;
  - distance and rotation caps;
  - the fence;
  - "arm moved since the proposal";
  - SDK return code `c == 0`.
- **No error clearing or automatic arm reconnect:** there is no `clean_error` / `clean_warn` call and no reconnect logic for the arm anywhere in `robotdash`.
- **Disconnect:** the one-shot scripts call `arm.disconnect()` (`step_long.py:228`). The long-lived `stepper.py` never disconnects.
- **Auto-run abort conditions:** STOP pressed, gripper disconnected for more than 12 s, force above 6 N, any failed check, a failed step (`stepper.py:232-246`).

## 10. Robot-state and wrist-camera recording
- **Robot state:** `arm.get_position()` rounded to 0.01 (`stepper.py:93`), `get_state()`, `get_err_warn_code()`. **There is no robot-side timestamp.**
- **Logs:**
  - `motion_log.txt`: host wall-clock ISO time, 1-s resolution (`stepper.py:27, 33-34`).
  - `actions_stepper_<tag>.jsonl`: `{t, ckpt, action, pose, jaw, force, obs}` per executed policy step (`stepper.py:90, 199`).
  - `step_long` writes `actions_<tag>.jsonl` with `t` as seconds since start (`step_long.py:207-211`).
- **Wrist camera (iPhone):** JPEG frames from `http://127.0.0.1:8099/frame.jpg` (`stepper.py:63-72`) and an MJPEG `/stream` (`record.py:30`). **Frames carry no timestamp.** The saved observation is 3 RGB frames at 256×256 (`stepper.py:148-153, 201`).
- **External cameras:** RealSense and ZED via the `:8090` dashboard (`server.py:13-18`). `record.py` writes an ~8 fps side-by-side mp4 with only an elapsed-seconds overlay (`record.py:80-107`).

## 11. Initialisation and import-time side effects
| File | On import | When run |
|---|---|---|
| `actuator.py` | Pure: numpy, json, os only (`actuator.py:26-27, 254`) | n/a (library) |
| `stepper.py` | Imports `policy.FlowBCPolicy`, `actuator`, `xarm.wrapper.XArmAPI`, `cv2`; inserts `sys.path` entries (`stepper.py:14-24`). No connection at import. | `Robot()` under `__main__` **connects to the arm, sets state 4, loads `R_ce.npy` and both checkpoints, and opens a new jsonl file** (`stepper.py:76-91, 387-391`). |
| `step_long.py` | Same imports (`step_long.py:16-23`) | `main()` turns gripper torque on, connects, enables motion (`step_long.py:133-140`). |
| `reset_and_sweep.py`, `reconfigure.py`, `reconfigure_step.py` | **Top-level scripts with no `main` guard: importing or running them connects and moves the arm immediately.** `reconfigure*.py` also rewrite the reduced-mode boundary and collision sensitivity (`reset_and_sweep.py:47-54`; `reconfigure.py:21-56`; `reconfigure_step.py:9-22`). | same |
| `lightcheck.py` | Top-level camera fetch | same |
| `gripctl.py` | `main` is guarded; `--serve` opens the serial port | service |

## 12. Notes for the next milestone (not implemented)
- The existing execution interface is **propose → check → blocking `set_position`**, with a fence, per-step caps, an error check, STOP, and state 4 between steps.
  - It runs at about 10–20 mm/s and is **not a streaming servo interface**.
  - A teleop adapter that reuses this interface would be step-wise by construction.
- **Your requirement:** any teleop path must go through the same fence limits (x 180–502, y −300…270, z ≥ 270 mm, controller reduced mode) and a deadman/stop. Every arm motion so far has required explicit approval with nobody near the arm.
- **Unknowns the adapter must not paper over:**
  - the controller's TCP offset;
  - the unmeasured camera lever arm (which matters for the policy path, not for a controller-pose teleop target);
  - the Euler convention, which the code marks as an assumption;
  - whether the controller boundary's x-min is 160 or 180;
  - the gripper's intermittent USB dropouts and estimated force.

**No robot or gripper connection was attempted during this inspection.**
