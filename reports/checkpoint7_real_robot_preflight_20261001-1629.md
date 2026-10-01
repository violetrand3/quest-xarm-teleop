# Checkpoint 7: real-robot preflight (2026-10-01 16:29)

**No robot connection was made.** Two of the on-site checks were not confirmed (a qualified person present who agrees, and that person clear of the arm with physical stop access), so the inspector was prepared and tested but not run against the arm. No motion, enable, state change, stop or gripper command was issued by anything in this checkpoint.

## 1. SDK/firmware, mode/state, errors, command count
| Item | Value |
|---|---|
| Installed SDK | **1.18.4** (`xarm_python_sdk`, user `dev`, Python 3.10.12 on `linux2`) |
| Robot firmware | **not collected** (no connection) |
| Model / axis count, simulation flag | **not collected** |
| Mode, state, error and warning codes | **not collected** |
| Pending-command count | **not collected** |

## 2. Cartesian pose, TCP/world offsets, payload
| Item | Value |
|---|---|
| Cartesian pose (mm, degrees) | **not collected** |
| Joint angles (degrees) | **not collected** |
| TCP offset | **not collected** |
| World/base offset | **not collected** |
| TCP payload and centre of gravity | **not collected** |
| Motor enable / brake status | **not collected** |

The SDK holds initialisation defaults for all of these before any report arrives (pose `[201.5, 0, 140.5, π, 0, 0]`, state 4, mode 0, zero offsets, zero payload). The inspector never reports them as measurements.

## 3. Reduced mode and fence
| Item | Value |
|---|---|
| Reduced-mode status, TCP boundary, speed and joint limits | **not collected** |
| Fence (safety boundary) status | **not collected** |
| x-min 160 vs 180 | **still unresolved**; `reconfigure.py:55` wrote 160, `stepper.py:29` uses 180 |

Boundary order in this SDK, for when the value is read: `[x_max, x_min, y_max, y_min, z_max, z_min]` (docstrings of `set_reduced_tcp_boundary` and `get_reduced_states`). The inspector keeps the raw structure from both the getter and the report.

## 4. On-site supervision and command ownership
| Check (your answers) | Status |
|---|---|
| Qualified person present and agrees | **not confirmed** |
| Arm stationary, no task running | confirmed |
| Person outside the working area with physical stop access | **not confirmed** |
| No commands will be issued during the inspection | confirmed |
| Intended robot address (from `stepper.py:78`) | confirmed |

- **Command ownership:** `stepper.py` (pid 4060276, running since 2026-09-28) holds the only connections to the arm address: one control (port 502) and one report (port 30002). Its web UI listens on loopback 8093. It was not stopped or touched. An idle connection is not exclusive ownership; whoever starts teleop must first decide what happens to this process.
- **Manual enabling:** nobody is assigned. The adapter never enables; the existing code enables inside `Robot.execute()` (`stepper.py:191`).
- **Other services seen, not touched:** gripctl (loopback 8088), DroidCam relay (8099), camera wall (8090), two recording viewers (8091, 8092), the unrelated program on 8765.
- The fixed ZED view showed a person seated at a desk behind the arm at the time of the camera check. I can't tell from the image who that is or how far they are from the arm.

## 5. Camera viewing
| View | Status |
|---|---|
| Desktop, third-person (fixed ZED and RealSense) | **verified**: you identified the robot and work area and confirmed live updating |
| Desktop, wrist (iPhone via DroidCam) | **not verified**: one real frame, then "503 – open DroidCam on the phone and leave it in the foreground" |
| Wearing the Quest with the input app focused | **unsupported by the current setup**; not tested in the headset in this checkpoint |

- **Access method:** the lab's existing camera wall, `/home/dev/camview/server.py`, on `linux2` loopback port 8090. It serves MJPEG at `/cam/zed`, `/cam/rs_color`, `/cam/rs_depth`, `/cam/c920`, `/cam/iphone`. I opened it in your local browser through `ssh -N -L 127.0.0.1:18090:127.0.0.1:8090 linux2`. No credentials are involved.
- The page is view-only (GET routes only). I did not use the `robotdash` dashboard, which carries gripper buttons.
- The wall opens each camera only while someone is watching and releases it afterwards. After I closed the tunnel it reported 0 viewers and held no video devices. No second capture process was started and no camera setting or service was changed.

Measured on the local PC over 8 s per feed (local receive times only):

| Feed | Frames | Distinct | Rate | Interval median / p95 / max | Size |
|---|---|---|---|---|---|
| ZED (fixed) | 221 | 221 | 29.4 fps | 33 / 53 / 270 ms | 672×376 |
| RealSense colour (fixed) | 215 | 215 | 28.4 fps | 33 / 57 / 442 ms | 640×480 |
| C920 | 217 | 217 | 28.6 fps | 33 / 59 / 390 ms | 1280×720 |
| iPhone (wrist) | 9 | 2 | not updating | about 1.9 s between placeholder frames | 640×480, then 640×360 placeholder |

- The streams carry no source capture timestamp and no frame counter. End-to-end latency was **not** measured; frame rate does not give it.
- No start-up burst was seen (one frame in the first 200 ms), so no obvious buffering. Single gaps of 270–440 ms occurred in each fixed feed.
- The C920 is labelled "gripper-mounted" on the wall but currently shows a wall.
- Headset viewing: the camera wall is a web page and the diagnostic Quest app shows no video. Leaving the app for a browser takes focus away, and loss of focus disengages the adapter (measured in Step 4B Test 3).

## 6. Remaining requirements for one supervised arm movement
1. **Run the read-only inspection** with a qualified person on site who agrees, is clear of the arm and has the physical stop. Command (on `linux2`):
   `python3 scripts/xarm_readonly_preflight.py --address-from-file /home/dev/robotdash/stepper.py --connect --human-confirmation "<who confirmed what>" --out logs/preflight_<ts>.json`
2. From that snapshot: firmware version, TCP and world offsets, payload, reduced-mode and fence status with exact bounds, and the 160/180 question.
3. **Command ownership:** decide whether `stepper.py` is stopped or kept, and by whom. Name who enables the motors.
4. **Clear area:** not identified. Nobody on site has confirmed a clear volume for a first small move. A clear TCP path would not show that all links, the phone mount and cables have clearance.
5. **Stop and queue behaviour**, to be verified physically (see section E below).
6. **Wrist camera:** someone on site must bring DroidCam to the foreground on the phone, then the wrist view needs verifying.
7. **Operator's view in the headset:** no solution exists yet.
8. Approved workspace and limits, as in the checkpoint-6 checklist. No workspace was derived from a measured pose.

The adapter's real command paths remain blocked. The gripper remains command-disabled; its dropout investigation can stay separate from a first arm-only test.

---

## A. The read-only inspector
`scripts/xarm_readonly_preflight.py`, deployed to `~/quest-xarm-teleop-dryrun/scripts/` (sha256 `e783365b…1627`, identical on both hosts). It uses the installed SDK directly, not `Robot()`, not the adapter backend.

Reading the SDK source first showed three side effects in its own connection path:

| SDK behaviour (1.18.4) | Where | What the inspector does |
|---|---|---|
| `connect()` calls `clean_warn()` if a warning is present | `x3/base.py` `_check_version` | Refused before it runs |
| The report thread reconnects the report socket by itself | `x3/base.py` `_report_thread_handle` | Reconnect disabled after the first connect |
| The control connection is switched to heartbeat protocol 3; `connect()` re-arms this | same thread, `_init()` | Switch refused; stays on plain protocol 2 |

Three layers:
- **API guard:** the object is built with `do_not_open=True`. Before `connect()`, every public SDK method outside a short allowlist is replaced on that instance by a stub that records the attempt and does nothing.
- **Wire guard:** only payload-free reads of nine registers can be sent: `GET_VERSION`, `GET_ROBOT_SN`, `GET_STATE`, `GET_CMDNUM`, `GET_ERROR`, `GET_TCP_POSE`, `GET_JOINT_POS`, `GET_REDUCED_MODE`, `GET_REDUCED_STATE`. Anything else is refused locally and recorded.
- **Lifecycle:** no keep-alive thread, no heartbeat, TCP only, connect bounded to 8 s, no reconnect. Cleanup closes this connection and calls no stop function.

SDK operations it uses: `XArmAPI(..., do_not_open=True)`, `connect`, `get_version`, `get_robot_sn`, `get_state`, `get_cmdnum`, `get_err_warn_code`, `get_position`, `get_servo_angle`, `get_reduced_mode`, `get_reduced_states`, the rich report subscription (port 30002), `disconnect`. Every call, frame and refusal goes into the snapshot's ledger.

Values:
- A value is kept only if the SDK return code is 0.
- TCP offset, world offset, payload, motor bits, collision settings, gravity direction, simulation flag and fence flag come from the rich report. They are accepted only after at least 5 reports arrive on this connection with the report counter advancing; otherwise they are marked unavailable.
- Every value carries its source and host query or receive times. No robot timestamps exist, and separate reads are not synchronised.
- Mount direction is not collected: the SDK gets it from the Studio HTTP API, a different interface. The report's gravity direction is recorded instead.
- If the joints or TCP change during the window it prints an alert and exits. It does not stop the robot.

### Tests (no robot involved)
| Where | Result |
|---|---|
| `linux2`, installed SDK, Python 3.10.12 | 20 tests OK |
| Local, copied SDK source | 20 tests OK; whole suite 171 OK |

- 12 tests use an injected fake SDK: start-up, reads, a non-zero return code, an exception, a connect timeout and cleanup issue no changing call.
- 8 tests run the **real SDK code** against a synthetic controller on loopback that records every frame. In every scenario (normal, warning, error, silent controller, no controller, no report socket, movement) it received only the nine read registers on protocol 2.
- One of those tests shows the hazard directly: unguarded, the SDK's `connect()` sends `CLEAN_WAR` (register 17) to the fake controller; with either guard it does not.

A fake controller shows what this program sends. It shows nothing about how the real controller behaves.

## E. Commissioning decisions from the findings
| Question | Finding |
|---|---|
| Does the SDK/firmware documentation support mode 7? | The installed SDK lists "7: cartesian online trajectory planning mode" in the `set_mode` docstring. The package documents no firmware requirement and no queue-or-replace behaviour for it. Firmware version is unread. **Not established.** No `set_mode` was called. |
| Do TCP/world offsets match the frame assumptions? | **Unknown**; not read. |
| Actual safety-boundary values | **Unknown**; not read. The 160/180 conflict stands. |
| Exclusive command ownership, manual enabling | Unassigned. `stepper.py` is connected now. |
| Clear area for a first small movement | **Not identified** by anyone on site. |
| Stop and queue behaviour still to verify physically | (a) whether non-blocking `set_position` queues or replaces in mode 0 and in mode 7; (b) what `set_state(4)` does to a move in progress, with stopping time and distance; (c) what is needed to move again afterwards; (d) whether the pending-command count returns to 0 after a stop. |

One source finding changes the checkpoint-6 picture: in SDK 1.18.4 `emergency_stop()` only repeats `set_state(4)` for up to 3 s (`x3/xarm.py:1109`). Its docstring says it also re-enables and sets state 0; the code does not. So the rig's `Robot.stop()` is `set_state(4)` twice. It is a software state change, not a hardware E-stop.

## Final state
| Item | State |
|---|---|
| Observer connections to the arm | 0 (never opened) |
| Connections to the arm address | `stepper.py` only, unchanged |
| Camera tunnel | closed; camera wall back to 0 viewers |
| Existing robot, camera and gripper services | running, untouched |
| Port 8765 | untouched |
| Quest APKs | unchanged (`57f2936c…`, `ea988329…`) |
| Adapter real command paths | still blocked |

Files: `data/checkpoint7/preflight_snapshot_20261001-1629.json` (machine-readable), `camera_feed_check.json`, sample frames, `user_confirmation_20261001-1629.txt`, `deploy_20261001-162110.log`.

Stopped before any connection, motion or enable command.
