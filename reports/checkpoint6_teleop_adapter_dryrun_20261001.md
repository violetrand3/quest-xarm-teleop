# Checkpoint 6: teleop adapter and live dry run (2026-10-01)

**DRY RUN: NO ROBOT OR GRIPPER CONNECTED.** The only backend that ran is `FakeRobotBackend`. Every tool pose, workspace value and stop request below is synthetic. Nothing here shows that the xArm moves, stops or holds. No SDK client was constructed, no hardware was queried, and no project module on `linux2` was imported.

## Outcome
- The adapter is built, tested and deployed to `~/quest-xarm-teleop-dryrun/` on `linux2`.
- One live Quest dry run engaged three times, issued 1941 synthetic target requests, and recorded 3 stop requests. Limit violations: 0. Transport errors: 0.
- A single missing source record ended the third engagement. The adapter stopped requesting targets at once and did not re-engage while the button stayed held.
- The arm command path of the real backend is blocked on purpose. The blockers are in `reports/checkpoint6_commissioning_checklist_20261001.md`.

## What was built
| Concern | File | Notes |
|---|---|---|
| Input validation, engagement | `scripts/teleop_input.py` | `InputChannel` (newest input only), `Engagement` clutch, `ChannelReceiverState` (the existing v2 receiver state, unchanged, mirrored into the channel) |
| Controller-to-tool mapping | `scripts/teleop_mapping.py` | `AnchoredTranslation`, `validate_rotation`, `GripperFollower`, trigger mapping |
| Limits | `scripts/teleop_limits.py` | `Limits` (workspace, discontinuity, displacement, tracking error), `RateLimiter` |
| Scheduling, monitor, recording | `scripts/teleop_adapter.py` | `TeleopAdapter.tick`, `TeleopAdapter.monitor`, `Recorder`, strict config parser |
| Backend interface, fake | `scripts/teleop_backend.py` | `RobotBackend`, `FakeRobotBackend` (declares itself synthetic) |
| Prepared real adapters | `scripts/teleop_backend_xarm.py` | Injected objects only; no SDK or HTTP import; no entry point |
| Dry-run entry point | `scripts/teleop_dryrun.py` | Constructs only `FakeRobotBackend`; has no backend option; refuses a config marked approved for live |
| Replay and analysis | `scripts/teleop_replay.py`, `scripts/analyze_teleop_dryrun.py` | Local tools |
| Configuration | `config/teleop_dryrun_example.json` | Synthetic limits, `approved_for_live: false` |
| Tests | `tests/test_teleop_adapter.py` | 56 tests |

### Behaviour
- **Units:** metres internally; converted once in the backend (`XArmPreparedBackend.read_feedback` and `native_target`). The recording shows millimetres.
- **Orientation:** the tool orientation is captured from feedback at engagement as the backend's own tuple and passed back unchanged. The core does no Euler math, so the unverified `Rz·Ry·Rx` assumption is not used. Controller orientation never enters the target.
- **Mapping:** `p_target = p_tool_at_engagement + scale · R · (p_controller − p_controller_at_engagement)`. `R` is configuration, checked to be orthonormal with determinant +1. The shipped `R` is an arbitrary example (LOCAL −z → +x, +x → −y, +y → +z), scale 0.2, labelled unverified.
- **Engagement:** starts in `WAIT_RELEASE`. A valid release (grip ≤ 0.2) and then a later valid press (≥ 0.8) must both be seen in one unbroken identity and continuity generation. A missing or null grip is never a release. While `ACTIVE`, grip below 0.5 disengages.
- **Check order per tick:** reference notices and monitor faults → identity and continuity → availability, freshness, grip → feedback → controller discontinuity → target, displacement and workspace → rate limiter → tracking error → command.
- **Disengagement:** the request is dropped, the references are cleared, the gripper follower is released, and `backend.stop(reason)` is called. Nothing re-engages by itself.
- **Latched faults (need SIGUSR1 while disengaged, then release and press):** any reference-change notice or changed reference counter, feedback or command failure, every limit violation, scheduler overrun or stall, a failed stop. `E` is not used anywhere and no settling time is assumed.
- **Scheduler:** 20 ms period on the lab host's monotonic clock, in its own thread. Only the newest accepted input is used; ten packets between two ticks produce one request. With no new input the last request is held until the 250 ms freshness limit, never extrapolated.
- **Monitor:** a second thread. If a tick is stuck or the scheduler stops for more than 150 ms, it raises a fault and calls `backend.stop()` without waiting for the tick.
- **Gripper preview:** index trigger 0..1 → `POST /api/do/grip {"value": 0..100}` percent closed (trigger pressed = closed). Following starts only after the trigger value comes within 5 % of the reported jaw position. No execute path exists; proposals go to `FakeRobotBackend.preview_gripper`. A missing trigger or a disengagement stops following and sends nothing.

One design choice to review: the two `other_space_change_pending` events at every app start latch a fault, so each app start needs one acknowledgement before the first engagement.

## Prepared real backend (not connected)
Interface names from the inventory, as implemented in `teleop_backend_xarm.py` and tested with injected fakes:

| Operation | Calls | State |
|---|---|---|
| Feedback | `arm.get_position()`, `arm.get_err_warn_code()` (mm, degrees; `is_radian=False`) | Implemented. Any error or warning raises; nothing is cleared. |
| Target conversion | keyword arguments for `arm.set_position(x, y, z, roll, pitch, yaw)` | Implemented as `native_target`; **not called**. |
| Command | `arm.set_position(...)` | **Blocked**: raises `LiveIntegrationBlocked`. |
| Stop | `arm.emergency_stop()` then `arm.set_state(4)` (as `Robot.stop()`, `stepper.py:215-220`) | Implemented. Physical effect unverified. |
| Never called | `motion_enable`, `set_mode`, `set_state(0)`, `clean_error`, `clean_warn`, `reset`, homing, reconnect | Tests assert none of these is issued. |
| Gripper state | `GET /api/state` → `values.position`, `status.connected` | Implemented through an injected getter. |
| Gripper command | `POST /api/do/grip {"value": pct}` | **Blocked**: raises `LiveIntegrationBlocked`. |

## Tests
| Where | Result |
|---|---|
| Local, Python 3.13.5 | **151 tests OK** (95 existing + 56 new), re-run after the live run |
| `linux2`, Python 3.10.12 | **56 adapter tests OK** (3 replay tests skipped: recordings are local only) and 39 existing link tests OK |

Deployed hashes match on both hosts (`data/checkpoint6/deploy_*.log`). The previous lab files were backed up to `backup_20261001-154852/`.

Three kinds of evidence, kept apart:

| Evidence | What it covers |
|---|---|
| **Synthetic** (packets built with the real sender-side tracker, passed through the real receiver state) | All 15 requested cases, plus: source gap between release and press, app and acquisition restart, rejected packet, anchor displacement, scheduler overrun, stale feedback, tracking error, both monitor paths, refused acknowledgement while active, pure rotation requests no translation, the real entry point over a real loopback socket |
| **Replay** of saved Quest recordings | Test 3 (focus menu): no request while the grip was held from the start, through the menu and after focus returned; one engagement after the recorded release and press. Test 5 (recenter): latched, 0 requests. Checkpoint-5 run (grip held through app restart and acquisition pause): 0 engagements, 0 requests |
| **Live** | The one dry run below |

The isolation tests run the dry run in a subprocess with imports of `xarm`, `dynamixel_sdk`, `serial`, `aiohttp`, `requests`, `urllib.request`, `http.client` and `teleop_backend_xarm` forbidden and every outgoing socket connection forbidden. It exits cleanly.

## Live dry run `live_20261001-155350`
Files: `data/checkpoint6/*live_20261001-155350*` (adapter recording, link log, sender logs, cue marks, your confirmations, `analysis_*.json`). Times are seconds on the lab clock after the first tick.

| Time | Transition | Reason |
|---|---|---|
| 7.08 | WAIT_RELEASE → FAULT_LATCHED | start-up `other_space_change_pending` notices |
| 20.32 | FAULT_LATCHED → WAIT_RELEASE → ARMED | operator acknowledgement (SIGUSR1); grip already released |
| 27.82 | ARMED → ACTIVE | valid press after valid release |
| 30.14 | ACTIVE → WAIT_RELEASE → ARMED | grip released (you confirmed you let go and re-pressed) |
| 31.78 | ARMED → ACTIVE | valid press after valid release |
| 64.06 | ACTIVE → WAIT_RELEASE → ARMED | grip released on cue |
| 74.84 | ARMED → ACTIVE | valid press after valid release |
| 79.06 | ACTIVE → WAIT_RELEASE | `continuity_generation_changed`: one source record (seq 4985) never arrived |
| 83.08 | WAIT_RELEASE → ARMED | release observed after the final cue |

### Results
| Item | Result |
|---|---|
| Engagements | 3, each with engagement jump 0.000 mm (first target = tool feedback) |
| Synthetic target requests | 1941, all while ACTIVE; 0 in 2464 disengaged ticks |
| Stop requests recorded | 3 (2 × `grip_released`, 1 × `continuity_generation_changed`), all by the scheduler |
| Faults | 2, both start-up reference notices; limit violations 0 |
| Continuity generation while ACTIVE | 15 throughout |
| Preserved orientation | `(10.0, 20.0, 30.0)` (synthetic) on every request |
| Mapping error against the formula | ≤ 0.01 mm (log resolution) |
| Request minus limited target | up to 25.6 mm during the fast part of a move |
| Limited-target speed | 50.5 mm/s measured, limit 50; the log's 0.01 mm resolution adds up to about 0.9 mm/s |
| Scheduler | 4405 ticks, period median 20.0 ms, max 20.7 ms, no late ticks |
| Input age at a tick while ACTIVE | median 19.5 ms, p95 53.8 ms, max 179.4 ms (limit 250 ms) |
| Link | 2056 packets, 0 rejected, 0 errors; RTT median 236 ms, p95 308 ms (sender clock) |
| Gripper preview | "following" at 0 % closed throughout (trigger stayed at 0); 0 commands executed |

Acceleration computed from the log reads up to 297 mm/s² against a 250 limit. Positions logged at 0.01 mm allow about ±87 mm/s² of error at a 20 ms step, so the log cannot confirm or refute the bound. The exact bound is checked in the unit test.

### Cued movements (controller in LOCAL → requested synthetic displacement, peak in the 3.5 s after each cue)
| Cue | Controller moved (mm) | Requested (mm) |
|---|---|---|
| Right | x +67.7 | y −13.4 |
| Up | y +304.2 | z +60.8 |
| Forward | z −145.5, y +72.4 | x +29.1, z +14.5 |
| Left | x −94.5 | y +18.9 |

Each is 0.2 × the controller displacement through the example `R`. In this session your right, up and forward were LOCAL +x, +y and −z. That is an observation from one session, not a calibration.

- **Wrist rotation (7.3 s):** the controller rotated through 78°. The orientation output did not change. The controller's position moved up to 47 mm while you twisted, which requested up to 9.5 mm of translation.
- **Re-engagement:** between the cued release and the next press the controller moved 21.5 mm, and the first new target equalled the tool feedback (jump 0.000 mm). A large reposition was not exercised live; it is covered by the synthetic workspace test. You offered to repeat this part; I did not, since the task asked for one run.
- **Lost record:** after the disengagement at 79.06 the button read 1.0 for 199 more ticks (about 4 s). State stayed `WAIT_RELEASE`, with 0 requests.

Your account: you let go and re-pressed after the first press; you did the right, up, forward and wrist movements; you moved your hand with the controller for the reposition; your head may have moved a little, mostly during "forward". Head motion does not enter the target.

## Limitations found
- **Source gaps interrupt engagement.** One log record went missing 72 s into the app run and ended an engagement. Three more were lost in the first 1.3 s. The cause is still not established. On a real arm each one is a stop.
- **Start-up acknowledgement.** Every app start needs an operator acknowledgement because of the unknown-type space-change events.
- **Wrist twist couples into translation** through the controller's pose origin (47 mm here).
- **Delivered input rate** is about 25 Hz with gaps up to 179 ms at a tick; RTT median 236 ms. Sample age relative to the Quest is unknown.
- **The fake backend tracks perfectly and instantly.** Tracking-error and stop behaviour on a real arm are untested.
- The example mapping, scale, workspace and limits are test values.

## Final state
| Item | State |
|---|---|
| `org.questxarm.reader` | force-stopped; APK unchanged (`57f2936c…9f76`) |
| `com.rail.oculus.teleop` | unchanged (`ea988329…6341`) |
| Local sender, tunnel, logcat child | none running |
| Lab dry-run process, port 28765 | none running, not listening |
| Lab port 8765 and existing services | untouched |
| Headset serial in saved files | absent |

Stopped here. No robot or gripper connection was made.
