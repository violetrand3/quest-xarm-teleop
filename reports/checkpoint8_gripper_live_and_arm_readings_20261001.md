# Checkpoint 8: real gripper driven from the Quest trigger, and the arm's real configuration (2026-10-01)

**The arm did not move and was never commanded.** The gripper was commanded, with the user's authorisation and a labmate on site. The arm was only read, once, through the read-only inspector.

## 1. Outcome
- The Quest index trigger opens and closes the real jaws through the lab's `gripctl` driver. In the second run the jaws followed the trigger for 40 s over the full test range, twice closed and twice opened, with 103 commands and 0 errors.
- The arm's actual configuration is now read from the controller (section 4). The fence x-min is 180, which settles the 160/180 question.
- Opening the camera page at the same time as teleoperating appears to degrade the control link (section 3).

## 2. Gripper tests
Confirmations are recorded verbatim in `data/checkpoint8/user_confirmation_gripper_and_arm_read_20261001.txt` and `confirm_live_*.txt`: on-site person agrees, hands and objects clear, nobody else commands it, OK to move the jaws; on-site person "A labmate who works with this rig".

| Step | What happened |
|---|---|
| State read | Connected, calibration valid (open 3033, closed 1181 ticks), torque already on, jaws at 1950 ticks (58 % closed) |
| Step 1, direct command | `POST /api/do/grip` to open by 5 % and return. Each call took about 85–90 ms; the jaws settled in about 0.2 s |
| Run 1 (`live_20261001-182636`) | Jaws stayed still for 8 s with the trigger at rest, picked up at trigger 0.62, closed 58 % → 84 % in about 0.5 s (5 commands). 5 s later a 294 ms network gap made the input stale and the clutch let go; the rest of the cues did nothing, by design |
| Run 2 (`live_20261001-183142`) | Engaged for 40.5 s. Pickup on the first squeeze, then the jaws tracked the trigger between 15.4 % and 84.1 % closed, two full close/open cycles. 103 commands, 0 errors, command time median 87 ms |
| Clutch released, trigger squeezed to 1.0 | 0 commands sent in both runs; the jaws stayed where they were |
| End of each run | One restore command returned the jaws to the starting position |

Details of run 2:
- Difference between the requested value and the jaw position while following: median 2.0 %, 95th percentile 12.7 %, maximum 25 % (during a fast squeeze; the request is rate-limited to 50 %/s and the jaws lag behind it).
- Input age at a scheduler tick while engaged: median 19 ms, maximum 230 ms, against a 250 ms limit.
- The "halfway" cue was held at trigger 0.03–0.12, so a mid-range hold was not really exercised.
- Wrist-camera frames (`wrist_live_20261001-183142_closed.jpg`, `_open.jpg`) show the jaws narrow and wide.

Final gripper state: 1968 ticks, goal 1963, torque on, load −2.8 %, 41 °C (37 °C before), connected. The closing direction settles a few ticks short of its goal against the return spring; the same was seen in step 1 (1963 for a goal of 1950). The driver's effort cap register now reads 531 (60 %, the closing value) instead of 345 (39 %); the driver rewrites it on every move.

No USB dropout occurred during the tests, so the dropout path (stop following, require a new pickup) is covered only by synthetic tests.

### How it is built
- `scripts/gripctl_client.py`: loopback-only client. Two requests exist: `GET /api/state` and `POST /api/do/grip`. A worker thread sends the newest value at most 10 times a second and only when it changed by 1 % or more; it never queues. If the state is unknown, stale or disconnected, nothing is sent.
- `scripts/live_gripper_test.py`: the same link, clutch and scheduler as the dry run, a fake arm, and the real gripper. It needs `--moves-real-gripper` and a confirmation text.
- `scripts/teleop_adapter.py`: optional injected gripper executor (default none, so the dry run still cannot command anything). Clutch release, input loss or a fault cancels any unsent value.
- `scripts/teleop_mapping.py`: trigger 0..1 maps to a configurable range; the tests used 15–85 % closed to stay off both end stops.
- `scripts/ops/gripper_live_session.sh`: the session script with the Spanish cues.

Tests: 17 new ones against a synthetic gripper server (`tests/test_live_gripper.py`); 188 in the whole local suite. On `linux2` (Python 3.10.12) the gripper and adapter tests pass (73, 3 skipped). One of five full local runs reported a single failure that did not recur in the next three runs; I did not identify which test it was.

## 3. Control link and cameras
| Run | Camera page open | RTT median / p95 / max (sender clock) | Arrival gaps over 250 ms at the lab |
|---|---|---|---|
| Checkpoint 6 dry run | no | 236 / 308 / 387 ms | not counted |
| Gripper run 1 | yes (5 MJPEG streams plus a capture) | 593 / 955 / 1744 ms | 18 |
| Gripper run 2 | no | 222 / 304 / 503 ms | none that disengaged the clutch |

- In run 1 the local USB side was fine (largest gap 48 ms) and the sender sent steadily; the delay was on the network path.
- This is a comparison of two runs, not a controlled test. It points at the camera streams sharing the link.
- In run 2 the wrist frames were captured on `linux2` and fetched afterwards, so no video crossed the network during the run.
- The wrist camera works once DroidCam is in the foreground: about 11 frames per second through the camera wall.

## 4. Arm: hardware readings (read-only inspector, one connection, `preflight_20261001-180130.json`)
Confirmation recorded in the snapshot: labmate on site, "Yes, conditions are met" (qualified, agrees, outside the working area, physical stop access).

| Item | Reading |
|---|---|
| Firmware | 2.7.1 (`6,6,XI1305,AC1304,v2.7.1`): xArm 6, six axes |
| SDK | 1.18.4 |
| Simulation flag | false (real robot) |
| State / mode | 4 (stopped) / 0 (position) |
| Error / warning codes | 0 / 0 |
| Pending commands | 0 |
| Pose (mm, degrees) | x 257.36, y −290.89, z 495.78, roll −179.42, pitch −32.62, yaw −47.73 |
| Joints (degrees) | −48.70, −26.00, −49.70, 1.50, 43.10, −1.74 |
| TCP offset | all zero |
| World offset | all zero |
| Payload | 0.8 kg, centre of gravity (0, 0, 25) mm |
| Motor enable / brake bits | 1 / 1 on all six joints |
| Reduced mode | on |
| TCP boundary, raw | `[502, 180, 270, -300, 585, 270]` in the SDK order x_max, x_min, y_max, y_min, z_max, z_min |
| TCP boundary, decoded | x 180–502, y −300–270, z 270–585 mm |
| Fence (safety boundary) | on |
| Reduced-mode limits | TCP 50 mm/s, joints 20 °/s; J2 −117…116°, J3 −219…10°, J5 −97…180° |
| Collision | detection on, sensitivity 5, rebound on |
| Gravity direction | (0, 0, −1) |

- The inspector sent only read registers (27 frames), refused 0 attempts, and closed its connection. `stepper.py` remained the only process connected to the arm.
- No movement was seen during the read (0.0 mm, 0.0°).
- Explicit queries: five rounds, every return code 0, each answered in under 0.5 ms.
- Report-backed values (TCP and world offset, payload, motor bits, collision settings, fence flag): 10 reports arrived at 200 ms intervals on this connection. The firmware leaves the report counter at 0, so the inspector's automatic freshness rule did not pass. I accept the values because they are decoded from the received frames and agree with the explicit queries wherever both exist (pose, joints, state, command count, error codes, boundary).

### Readings against what the lab's code assumes
| Assumption in code | Hardware reading |
|---|---|
| Fence x 180–502, y −300–270, z 270–585 (`stepper.py:29`) | Same. The controller holds x-min 180, not the 160 written in `reconfigure.py:55` |
| Start pose `[299.83, 3.43, 461.18, 179.86, −15.1, −6.64]` | The arm is somewhere else: y is −290.9 mm, **9 mm from the y-min boundary** |
| No TCP offset is set in code | Confirmed zero. The reported pose is the flange, not the jaw tips; the fence applies to that point |
| mm and degrees | Consistent with the values read |
| Speeds of 10–20 mm/s | Reduced mode caps the TCP at 50 mm/s |

Not read: mount angles (the SDK gets them from a different interface), and anything about stop or queue behaviour, which needs motion.

## 5. What this changes for the first supervised arm movement
- Settled: firmware version, offsets, payload, fence bounds and that reduced mode is on.
- New: the arm sits 9 mm from the y-min edge. A first move must go away from that edge, or the arm should first be brought to the start pose with the lab's existing tool.
- New: the camera page should not share the link during teleoperation until that is understood.
- Still open: queue-or-replace and stop behaviour, who owns commands (`stepper.py` is still connected) and who enables the motors, an agreed clear area, the mapping of hand directions to robot axes, and how the operator sees the robot while wearing the headset.

## 6. Final state
| Item | State |
|---|---|
| Arm | untouched; `stepper.py` is its only connection |
| Gripper | 1968 ticks, torque on, connected |
| My processes on `linux2` | none; port 28765 closed; port 8765 untouched |
| Camera tunnel | closed; camera wall at 0 viewers |
| Quest app | stopped; both APKs unchanged |
| Adapter's real arm command path | still blocked |

## Correction added 2026-10-01 19:10: the jaws stopped following the servo during run 2
Section 2 judged run 2 by the servo's position feedback. The wrist-camera frames, measured afterwards (`data/checkpoint8/frames_live_20261001-183142.json`, jaw gap in pixels), show that the jaws themselves stopped following partway through:

| Moment in run 2 | Servo position | Jaw gap in the picture |
|---|---|---|
| Start | 58 % closed | 212 px |
| First close | 84 % | 190 px |
| First open | 15 % | 228 px |
| Second close | 84 % | 182 px |
| Second open, at about 35 % on the way down | 35 % → 15 % | jumps to 234 px |
| After the end-of-run restore | 57.5 % | still 234 px |

In the next run (`data/checkpoint9/…185805`) the servo again travelled 15–84 % and the gap stayed at 232–236 px. So since the second opening of run 2 the servo moves but the jaws stay open: the link between the servo and the jaws appears to have come loose. This is inferred from the camera; nobody has inspected the mechanism. The statement above that the jaws "tracked the trigger between 15.4 % and 84.1 % closed" is true of the servo, and of the jaws only for the first one and a half cycles.

No further gripper commands were sent after this was found.
