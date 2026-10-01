# Step 04B Test 4: right-controller tracking loss and recovery (app focused)

Date: 2026-09-30.
- Local PC and its USB-connected Quest only. No SSH, robot, gripper, network or mapping.
- No APK rebuild or install, no data clearing, no tracking-setting experiments. Hand tracking was OFF.
- The headset serial is redacted (checked by grep).

**Build:** `org.questxarm.reader` `s4a-ff0da2692500`, installed APK `57f2936ccf1756266bfca75f735eff7fbdb15fb675b7ac5dfca9fdbc61d09f76`, unchanged. `com.rail.oculus.teleop` `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341` is unchanged. Probes off.

## A. Independent tracking checks (code review + synthetic tests)
**Monitor logic:** `scripts/quest_pose_v1_record.py::right_status` and `quest_pose_v1.Pose.status`.
- `Pose.status` returns VALID_UNTRACKED whenever `!(pos_tracked && ori_tracked)`. **Either bit alone suffices**; both clearing isn't required.
- `right_status` reports available (`FRESH_TRACKED`) only for TRACKED with an active grip action, after the stream, overflow, focus and stale checks.
- The left controller is not consulted.

**Coverage gap found and filled:**
- At the monitor-status level, only "position valid, not tracked" had a test.
- Added the synthetic `test_single_tracking_flag_clear_makes_right_unavailable`. With the app focused, the pose action active, a finite matrix, grip 1.0 and the **left controller missing**, clearing **only** position-tracked, **or only** orientation-tracked, gives `VALID_UNTRACKED` with values hidden. Availability returns only with a fully tracked pose record.
- The full suite passes **64/64**.
- The availability rules were not changed. The synthetic evidence is separate from the physical trial.

## B. Acquisition
- **Recorder:** the verified pre-attach recorder. Fresh session `76e5858a05a494b1` (launch capture `test4/t4_launch_*`); one `local_space_change_pending` occurred at session start, before all trials. Probes off.
- **Abort-on-loss was not used.** Watchdog (500 ms, diagnostic only), invalidation and focus checks were unchanged.
- **Cue helper** (`scripts/step04b_test2_cues.py`, sha256 `042cc082…c7a4`, host-only), new kinds:
  - `occlusion`: long English cues.
  - `occlusion_short`: one-word English cues; the out-of-view phase was lengthened from 3 s to 5 s after you pointed out the path is toward you and then below the table edge.
  - `occlusion_short_es`: Spanish cues ("Mantén el control a la vista" / "Abajo" / "Arriba otra vez"), via the existing `spd-say -l es` (espeak-ng); you confirmed you heard it clearly. Nothing was installed.
- **Analysis:** `scripts/analyze_step04b_test4.py` (sha256 `cd27509a…0cce`). It lists **all concurrent unavailability reasons** per record, not only the displayed status.

## C. Attempts (planned cues, telemetry and your confirmation kept separate)
| Attempt | Recording | Cues | Your account | Telemetry |
|---|---|---|---|---|
| 1 | `test4/t4_occlusion_20260930-225641` | long English ("Gently move the controller down out of view, …") | "the voice was moving extremely fast… before I started, it went to the next thing" | **No degradation**: flags 15, FOCUSED, action active, grip 1.0 for the whole trial. The controller moved about 0.54 m (mostly up and to the side in LOCAL) late in the phase. **Inconclusive** (cue not followable). |
| 2 (your choice, clearer) | `test4/t4b_occlusion_retry_shortcues_20260930-230349` | "Hold it in view." / "Down." / "Back up." | You asked to try again; you started moving down while bringing it toward you. No confirmation of the specific actions. | **Position valid but untracked** for 1.72 s (details below) |
| 3 (your request, Spanish) | `test4/t4c_occlusion_attempt3_spanish_20260930-*` | "Mantén el control a la vista." / "Abajo." / "Arriba otra vez." | "Followed cues; grip held throughout; head still, no menu" | **Position valid but untracked** for 1.92 s (details below) |

The movement stayed the same gentle arm movement past the table edge in every attempt. Attempts 2 and 3 were not more disruptive; only the cue clarity changed. **Attempt 3 is the primary result**, being the only one with your confirmation of the actions. Attempt 2 is corroborating.

## D. Transition timeline (attempt 3; host monotonic time relative to the baseline cue)
| t (host) | seq | Right pose action | Flags (pv/ov/pt/ot) | Grip | Focus | Head | Monitor status (proc) | Unavailability reasons |
|---|---|---|---|---|---|---|---|---|
| 0.000 | | | | | | | **cue "Mantén el control a la vista."** | |
| 5.000 | | | | | | | **cue "Abajo."** | |
| 8.996 | 64863 | active | 15 (1/1/1/1) | 1.0 | FOCUSED | 15 | FRESH_TRACKED, **last available before** | none |
| **9.008** | 64864 | **active** | **7 (1/1/0/1)** | **1.0** | **FOCUSED** | **15** | **VALID_UNTRACKED (0.39 ms)** | **`pos_valid_untracked` only** |
| 9.008–10.923 | 110 records | active | 7 | 1.0 | FOCUSED | 15 | VALID_UNTRACKED ×110 | `pos_valid_untracked` ×110 (no other reason in any record) |
| 10.002 | | | | | | | **cue "Arriba otra vez."** | |
| **10.967** | 64974 | active | 15 | **1.0** (next 5: 1.0) | FOCUSED | 15 | FRESH_TRACKED, **first available after** (0.30 ms) | none |
| 15.003 | | | | | | | cue "Terminado" | |

- **Unavailable span:** 1.915 s (host), 1916.4 ms (XrTime query span). Each is measured on its own clock.
- **Order of flag changes:** only **position-tracked** changed, at 9.008 s. Position-valid, orientation-valid and orientation-tracked never changed in any attempt. The pose action and grip action stayed active, focus stayed FOCUSED, the head stayed TRACKED, and **no events** were recorded (no profile changes, no session or space changes).
- **Attempt 2 is the same pattern:** flags 7 from 9.384 s to 11.106 s (104 records, XrTime 1722 ms). The only reason was `pos_valid_untracked`. The first available record was at 11.111 s with grip 1.0 (next 5 at 1.0). The monitor updated 0.19 ms after the first untracked record.
- **Grip:** in attempts 2 and 3 the grip read 1.0 in every available record during the trial. In attempt 3 it dropped below 0.5 only at 18.6–19.0 s, after "Terminado", with the grip action still active. That's a real release after the trial, not one inferred from inactivity.
- **Behaviour during position-untracked** (the reported, still "valid" position):
  - Attempt 3: nearly frozen, with 18.0 mm path and 6.1 mm net over 110 samples. At re-tracking it **jumped 175.6 mm**.
  - Attempt 2: 43.5 mm path and 33.1 mm net, then a jump of about 110 mm in y at re-tracking.
  - The controller was being physically moved, so these are **not** drift, accuracy or "correction" measurements. They do show that the untracked position did not follow the physical controller, so those samples must not be used. They were never reported as tracked or available.
- **Timing caveat:** the audio cue times are not physical occlusion times. Position tracking dropped about 4.0 s (attempt 3) and 4.4 s (attempt 2) after the "down" cue. In attempt 2 the controller had already reached its low position about 2 s before tracking dropped. No sensor-to-detection or stopping latency is claimed.

## Stream
| Recording | Lines (all pose) | Events | Rate | Parse errors | Truncation | Overflow | Max record | Missing seq | Host inter-arrival med/p95/max |
|---|---|---|---|---|---|---|---|---|---|
| Attempt 1 | 2027 | 0 | 59.7/s | 0 | 0 | 0 | 1223 B | 1 | 16.3 / 45.4 / 49.1 ms |
| Attempt 2 | 1961 | 0 | 60.5/s | 0 | 0 | 0 | 1223 B | 1 | 16.1 / 44.9 / 48.2 ms |
| Attempt 3 | 1796 | 0 | 57.2/s | 0 | 0 | 0 | 1224 B | 1 | 17.3 / 43.2 / 49.9 ms |

- Pose and event records share one sequence counter. The single missing seq per recording occurred near the recorder's attach, and its type is unknown. No losses are inferred from a nominal rate.
- No telemetry pause occurred (max gap < 50 ms), so the stale watchdog was not exercised.
- Monitor processing (receipt to status update): median 0.16–0.36 ms, max ≤ 3.4 ms.

## Conclusions (separately)
1. **Was tracking degradation observed?** **Yes**, in attempts 2 and 3: right **position tracking** was lost for about 1.7–1.9 s while the controller was below the table edge. Attempt 1 showed none, and is inconclusive because the cue couldn't be followed.
2. **Was it isolated from focus loss and headset tracking loss?** **Yes.** The app stayed FOCUSED, the head stayed TRACKED, and there were no session, profile or space events throughout.
3. **Was a valid-but-untracked condition observed?** **Yes**: position VALID but not TRACKED (flags 7), with the pose action active and orientation valid and tracked.
4. **Did the monitor invalidate input correctly?** **Yes.** Every one of the 110 + 104 untracked records was VALID_UNTRACKED (unavailable, values hidden), set within 0.2–0.4 ms of receipt. `pos_valid_untracked` was the **only** concurrent reason in every one of those records, so **the position-tracking branch is physically proven independently** of focus, action-activity and grip checks.
5. **Did fresh input recover, and was the grip still pressed?** **Yes.** The first available record after re-tracking, and the following 5, had grip 1.0 in both attempts. Availability was restored only by a fully tracked pose record.
6. **Branches verified only by synthetic tests:**
   - **orientation valid but not tracked** (orientation tracking never dropped physically);
   - position or orientation **invalid** with the action active (only seen for one record during focus loss in Test 3);
   - the stale-input watchdog on a real pause;
   - overflow invalidation;
   - GRIP_UNAVAILABLE with a tracked pose (grip and pose activity always changed together physically).

Recentre, app restart, USB and real-pause tests were not run. This is not a declaration of Step 4B being complete or of robot-control readiness.

## State at stop
- The diagnostic app is force-stopped. Both apps are installed and unchanged.
- No recorder, cue or logcat processes remain. Probes are off.
