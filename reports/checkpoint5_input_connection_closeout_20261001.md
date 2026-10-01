# Checkpoint 5 closeout: no-motion input connection (2026-10-01)

**DIAGNOSTIC ONLY: ROBOT NOT CONNECTED.** No robot or gripper connection was attempted in this checkpoint. Both summaries of the live run record `robot_connection_attempted: false`. Robot code on `linux2` was read as text only.

## Outcome
- The new OpenXR reader (`org.questxarm.reader`, build `s4a-ff0da2692500`) is connected to the lab host through a separate, versioned path: protocol v2 `quest-pose-link`.
- One live run covered a normal window, an app stop and restart, and a real acquisition pause. Transport errors: **0**. Rejected packets: **0**. Every packet sent was received and acknowledged (1951 of 1951).
- The pause was detected **510.9 ms** after the last valid host-received sample. No output from the suspended child was accepted.
- One finding is open: the direction of `poseInPreviousSpace` (part A). Nothing uses that value.
- One thing was not obtained: your confirmation after the run of what you physically did. Only the pre-run confirmation is on file.

Deliverables:
1. This report.
2. `reports/robot_backend_source_inventory_20261001.md` (delivered earlier).
3. `docs/checkpoint5_pose_transport_v2.md` (protocol, commands, test results).

## A. Recenter interpretation
Recomputed from the saved Step 4B Test 5 recordings with `scripts/analyze_recenter_direction.py` (output: `data/checkpoint5/recenter_direction_20261001.json`). E is the recorded `poseInPreviousSpace`. "Old" and "new" are the two adjacent pose records across the coordinate switch, so residuals include one frame of physical motion.

| Event | Pose | Raw jump | Residual of `inverse(E)·T_old` | Residual of `E·T_old` |
|---|---|---|---|---|
| launch, seq 8 | head | 102.9 mm, 8.14° | 205.3 mm, 16.26° | 0.11 mm, 0.08° |
| attempt 2, seq 39063 | head | 350.6 mm, 0.98° | 701.3 mm, 1.96° | 0.04 mm, 0.02° |
| attempt 2 | right grip | 347.5 mm, 0.98° | 695.0 mm, 1.96° | 0.02 mm, 0.01° |
| attempt 3, seq 63388 | head | 35.6 mm, 1.60° | 71.2 mm, 3.20° | 0.05 mm, 0.02° |
| attempt 3 | right grip | 45.4 mm, 1.60° | 90.7 mm, 3.20° | 0.03 mm, 0.02° |

- **The expected relation does not fit this data.** `T_new_controller = inverse(E)·T_old_controller` leaves about twice the raw jump. `T_new = E·T_old` fits to within 0.11 mm in all five cases.
- **This is left as an open discrepancy.** I have not labelled it a runtime error or a spec reading error. Things checked on our side:
  - `LocalSpace`, `HeadSpace` and the action spaces are created with identity offsets.
  - The app writes `poseInPreviousSpace` to the log unmodified.
  - The decoder and the analysis use one convention throughout (column vectors, `T_LOCAL_from_X`).
- **Nothing applies E.** There is no recenter compensation anywhere. A LOCAL change is a continuity break, and that is all.
- **changeTime vs the observed switch is preserved as recorded.** Coordinates switched at the first pose after the event, 35.5 ms and 42.4 ms after `changeTime` in the two recenters (64.3 ms at launch). No fixed time correction was introduced.
- **What the reference counter counts:** `local_epoch` is the number of LOCAL `ReferenceSpaceChangePending` events the app process has received since launch. Other space types don't increment it. It is not reset per OpenXR session. It appears in every pose and event record.
- **What `sid` identifies:** the app process run (set in `AppInit`). It is not an OpenXR session id. Sessions are delimited by `session_begin`/`session_end` events, which the host counts as `session_gen`.

## B. What was built
See `docs/checkpoint5_pose_transport_v2.md` for the wire format. In short:
- **Versioned path:** `v: 2`, hello carries protocol, source format, source schema and frame `openxr_local`. v1 and v2 refuse each other.
- **Every source record is processed before the 30 Hz limiter** by `ContinuityTracker`.
- **Persistent counters:** `continuity_gen` (every break), `loss_count` (input losses), `source_gap_records` (missing source records), `decimated_since_last` (intentional skips). They are separate fields.
- **Raw input availability is separate from continuity.**
- **No motion latch**, no mapping, no smoothing, no targets.
- The v1 sender and receiver gained small hooks so v2 reuses their session and server code. Pre-change copies are in `data/checkpoint5/*.pre_v2hooks.py.bak`. The v1 tests still pass.

## C. Tests and deployment
| Where | Result |
|---|---|
| local | 95 tests OK (re-run after the live test) |
| `linux2`, Python 3.10.12 | 39 tests OK |

Deployed to `~/quest-xarm-teleop-dryrun/` (the existing directory, backed up first as `backup_20261001-135826/`). Hashes re-checked after the live run and identical on both hosts:

| File | sha256 |
|---|---|
| `scripts/quest_pose_transport.py` | `34bf6734d79c6dd215421d2c13cc40d4e4dc9e1baad58cea55cde8032926bfc7` |
| `scripts/quest_pose_stream_receiver.py` | `e129a39c717cde98ed541048c1474f536a68247ee52c1725f034034c5c748e77` |
| `scripts/quest_stream_receiver.py` | `5118ed0ef86dbd56056acc1a681b8e6db7be6ad4caa81db1269a6627acc8fde4` |
| `scripts/quest_transport.py` | `318b88e738ba89788316a97edd1cd62d9a417bd14f1fb11db1e3b8d3cf87bcaf` |
| `scripts/quest_pose_v1.py` | `6ee175f3576afd4dade6e5bcc6342054f61dce9fb74adfdc5d900c4e5f8eadbb` |
| `scripts/quest_pose_stream_sender.py` (local only) | `1c0067f0e5ad7a252b656e082478f009038b2b90585a2c8d73c192b35741f5b6` |
| `scripts/quest_stream_sender.py` (local only) | `7b0fa32701ba7be1a19537e2010b1a7cf5eb61e210523472114711b313c43368` |

Handshake run (`handshake_20261001-135927`, no app): 25 packets acknowledged, RTT median 135 ms, p95 175 ms, max 203 ms.

## D. Live run `live_20261001-151605`
Files: `data/checkpoint5/*live_20261001-151605*`. Analysis: `analysis_live_20261001-151605.json`, produced by `scripts/analyze_checkpoint5_live.py`. Times below are seconds after the first app launch on the sender's monotonic clock.

| Time | Step |
|---|---|
| 0.0 | launch app (session 1) |
| 8.6 to 41.5 | normal window |
| 47.3 | force-stop app |
| 53.4 | relaunch app (session 2) |
| 65.8 | SIGSTOP the sender's own logcat child |
| 68.3 | SIGUSR1: kill and reap the suspended child |
| 69.9 | force-stop app |
| 70.9 | SIGUSR2: new acquisition generation |
| 73.0 | launch app (session 3) |
| 84.7 | end |

### Counts
| Stage | Count |
|---|---|
| Source records read by the host | 5126 (5099 poses, 27 events, 0 parse errors, 0 overflow) |
| Source poses with input available | 5086 |
| Pose packets sent / received | 1830 / 1830 (1820 available, 10 unavailable) |
| Poses skipped intentionally by the limiter | 3268 |
| Source event packets sent / received | 27 / 27 |
| Status packets sent / received | 93 / 93 |
| All packets sent / acknowledged ok / rejected | 1951 / 1951 / 0 |

- 5099 = 1830 sent + 3268 skipped + 1 still in the slot when the sender stopped.
- The receiver was told of 3249 skips. The other 19 happened after the last pose packet, and the count only travels on the next pose packet.
- Availability decided at the source matched the flag received for all 1830 pose packets.

### Normal window (32.9 s)
| Measure | Value |
|---|---|
| Source poses | 2342, all available (71.2 Hz) |
| Pose packets sent and received | 817 and 817 (24.8 Hz) |
| Received poses marked continuous | 817 of 817, one continuity generation (25) |
| Source gaps, breaks, other packets | 0, 0, 0 |
| Host line-arrival interval (sender clock) | median 10.5 ms, p95 45.3 ms, max 48.2 ms |
| Pose send interval (sender clock) | median 33.3 ms, p95 66.7 ms, max 66.9 ms |
| Pose arrival interval (receiver clock) | median 33.8 ms, p95 68.6 ms, max 96.2 ms |

- The sent rate was 24.8 Hz, not 30. Log lines reach the host in bursts, so about one send slot in four had no new record. The limiter never sends an old pose twice.
- The right side grip read 1.0 for the whole window and the controller moved over 209 × 145 × 35 mm.

### Source gaps (kept apart from transport errors)
- **Transport errors: 0.** No sequence gaps, no rejects, no ack timeouts.
- **Source gaps: 14 gaps, 16 missing source records.** All were in the first 4.2 s after an app start: 9 in session 1, 2 in session 2, 3 in session 3. None fell in the normal window.
- Each gap raised `continuity_gen`. Twelve also cleared input (`input_lost:source_gap`) because input was available at the time.
- The cause isn't established. The records never reached the host as log lines. Whether the app, logd or adb dropped them can't be told from this data.

### Application RTT and receiver gaps
- **RTT (sender clock, all 1951 acks):** median 232.9 ms, p95 306.5 ms, max 353.3 ms.
- The handshake run, with status packets only, had a median of 135 ms. The console's last-RTT samples in this run read about 135 ms while only status packets flowed and about 170 to 185 ms under pose traffic. Those are coarse samples, 2 per second.
- I have no evidence for where the extra time under load comes from and don't attribute it to SSH.
- **Receiver packet inter-arrival (lab clock):** median 33.7 ms, p95 74.5 ms, max 268.5 ms.
- **Receiver available-pose inter-arrival:** median 33.6 ms, p95 68.3 ms. The maximum of 7965 ms is the pause and restart sequence itself.
- **Read-to-send age on the sender (same clock):** median 9.8 ms, p95 30.5 ms, max 33.3 ms. This is not sample age: the capture time on the Quest is unknown.

### Values arrived unchanged
- The receiver re-decoded the carried source record for all 1830 poses, compared the matrices to the record's LOCAL poses within 1e-9 and recomputed availability. Rejections: 0.
- Independent check: the 145 values the receiver printed while input was PRESENT (position, grip, trigger, A, B, flags) were compared with the local source log by `(acq_gen, sid, seq)`. Mismatches: 0. The console prints 3 decimals, so this check is to 0.5 mm.

### App stop and restart
| Observation | Value |
|---|---|
| Last record before the stop | `sid 152c45fd…` seq 3292, available, 50 ms after the force-stop command was issued |
| First notice to the receiver | status with `source_down:app not running` and `source_stale`, sent 505.7 ms after the last record |
| While the app was down | 24 status packets, all `source_up: false`, `stale: true`; receiver showed `SOURCE_DISCONNECTED (app not running)` with values hidden |
| First record after relaunch | new `sid fff5c05d…`, seq 1, `session_begin` |
| Breaks listed on restart | `app_run_changed`, `session_begin`, 2 × `unknown_space_change`, `session_state:IDLE/READY/SYNCHRONIZED/VISIBLE` |
| First available pose | seq 10, acknowledged `continuous: false` |
| Records of the old `sid` after relaunch | 0 |
| Input unavailable (sender clock) | 6.78 s |

Between the last pose and the first notice, the receiver got two ordinary heartbeats. During those ~500 ms its last pose aged on its own input watchdog and was not refreshed.

### Acquisition pause
Before the signal, the child was checked: pid 455586, parent = the sender (455573), command = our tag-filtered `adb logcat`. Only that process was stopped. It was never resumed.

| Observation | Value |
|---|---|
| Last record from the child | seq 829 of `sid fff5c05d…`, 25.7 ms before the SIGSTOP mark |
| Child state during the pause | `T` (stopped) |
| Watchdog detection | **510.9 ms after the last valid host-received sample** (485.2 ms after the SIGSTOP mark); input was still marked available at that moment and was cleared |
| How STALE was set | by the sender's watchdog alone; nothing set it by hand |
| Transport during the pause | alive: 38 packets sent and acknowledged between the last old record and the first new one |
| What the receiver was told | 2 heartbeats, then `source_stale`, then `source_down:acquisition aborted` + `acquisition_restart`, then `acquisition_restart` for the new generation |
| Abort | child SIGKILLed, reaped (return code −9), reader thread joined |
| Records read between SIGSTOP and the new generation | 0 |
| Records with acquisition generation 1 after the abort | 0 |
| Records of the paused `sid` after SIGSTOP | 0 |
| Old-generation lines discarded | 0 (the killed child delivered nothing) |
| New generation | `acq_gen` 3, child 457038 (ownership checked again), new `sid 11d04d33…` from seq 1 |
| First available pose in the new generation | seq 10, acknowledged `continuous: false` |
| Input unavailable (sender clock) | 7.96 s |

`acq_gen` 2 is the interval with no child between the abort and the new start.

Transport status and source/input status stayed separate throughout: the receiver showed `transport=LIVE` with `input=STALE`, then `SOURCE_DISCONNECTED`, and never `PRESENT`, until the new generation's first available pose.

### How continuity changes survived rate limiting
- The source log shows 35 generation steps on records. **8 of them fell on a pose the limiter skipped.**
- The receiver's last generation was 58 and it had 58 break reasons listed. `breaks_not_listed`: 0. Generation never went backwards. Identity changes without a generation change: 0.
- The receiver saw 29 distinct generation changes, because several breaks can ride on one packet. Each packet lists every reason.
- 13 available poses were acknowledged `continuous: false`: the first available pose of each of the 3 app runs, and the first after each of 10 gap-related breaks.
- The sender ended at generation 60. The last two breaks were its own shutdown abort and were never sent.

### Old-session rejection
- **Live:** no old-session packet was ever offered to the receiver, so the rejection path did not fire (0 rejects). The receiver logged three identities in order (`acq1/152c45fd`, `acq1/fff5c05d`, `acq3/11d04d33`).
- **Tests:** rejection of a retired identity is covered by `test_restart_no_cached_pose_and_old_identity_rejected`. Discarding old-generation lines is covered by `test_generation_restart_discards_old_lines_and_cached_pose`. The suspended-child case is covered by `test_suspended_child_is_detected_killed_and_replaced`.

### Your actions during the run
Before the run you confirmed: headset on when cued, middle-finger side grip held, hand tracking off. I did not collect a confirmation after the run. The data is consistent with the plan (grip at 1.0, input available again within about 0.2 s of each app start), but that is the recording, not your account.

## E. Robot backend
See `reports/robot_backend_source_inventory_20261001.md`. Nothing there was imported, run or connected to.

## Remaining limitations
- **Frame:** poses are in OpenXR LOCAL, unmodified. No relation to the robot base frame exists. LOCAL changes on recenter.
- **Recenter direction:** open, as in part A.
- **Unknown space type:** the app does not emit the raw `referenceSpaceType` for non-LOCAL change events, so they are treated as `unknown_space_change`. Two arrive at every session start.
- **Timing:** `t_query_ns` is a predicted display time, not a sensor capture time. There is no Quest-to-host clock relation, so absolute sample age is unknown. Arrival rate is not sample age.
- **Freshness:** the 500 ms thresholds are diagnostic. After a silent acquisition stall the last pose stays current for up to about 500 ms on the sender, and the receiver's own 500 ms input watchdog runs separately. Neither is a safety timeout.
- **Rate:** about 25 Hz delivered from a 71 Hz source, with send intervals up to 67 ms.
- **RTT:** median 233 ms under pose traffic in this run, from one run on one network path.
- **Source gaps at app start:** cause not established.
- **Pause test:** one trial of one fault (a stopped logcat child). A stalled adb server, a USB drop and a frozen app were not tested live.

## Final state
| Item | State |
|---|---|
| `org.questxarm.reader` | force-stopped, installed, APK sha256 `57f2936c…9f76` (unchanged) |
| `com.rail.oculus.teleop` | installed, APK sha256 `ea988329…6341` (unchanged) |
| Local sender, tunnel, logcat children | none running |
| Lab receiver, port 28765 | none running, not listening |
| Lab port 8765 | still held by the other program, untouched |
| Shared adb server, unrelated Android logs | untouched |
| APK builds, installs, sudo, new dependencies, firewall or SSH configuration changes | none |
| Headset serial in saved files | absent (checked in `reports`, `docs`, `scripts`, `tests`, `data/checkpoint5`) |

Stopped here. No robot targets, mapping, SDK clients or motion. The robot adapter with dry-run output has not been started.
