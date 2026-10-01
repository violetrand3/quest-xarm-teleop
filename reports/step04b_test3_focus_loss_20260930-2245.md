# Step 04B Test 3: focus loss/return with grip held, then release/repress

Date: 2026-09-30.
- Local PC and its USB-connected Quest only. No SSH, robot, gripper, network or mapping.
- No APK rebuild or install, no data clearing, no setting experiments. Hand tracking was OFF.
- The headset serial is redacted (checked by grep).

**Build:** `org.questxarm.reader` `s4a-ff0da2692500`, installed APK `57f2936ccf1756266bfca75f735eff7fbdb15fb675b7ac5dfca9fdbc61d09f76`, unchanged. `com.rail.oculus.teleop` `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341` is unchanged. Probes off.

## A. Recording setup
- **Recorder:** the verified pre-attach recorder. Launch capture `test3/t3_launch_*` shows a fresh session `0add96f2ec4a0595` (session_begin → … → FOCUSED), 0 probes and 0 errors.
- **Recording:** `data/step04b/test3/t3_focus_menu_20260930-223940`, with `.jsonl` (raw, every record), `.summary.json`, `.marks.txt`, `.planned.json`, `.user_preconfirm.txt`, `.console.txt` (2 Hz status) and `.test3.json` (analysis).
- **Abort-on-loss was disabled** for this run (cue kind `focus_menu`). The recording continued through the loss.
- **Unchanged:** the stale watchdog (500 ms, diagnostic only), missing/invalid handling, focus checks and raw event capture. Stream freshness (any line) and input freshness (accepted pose records only) stay separate; event and overflow records never refresh input freshness.

## B. Status-rule review and host-side fixes (documented)
I reviewed `right_status()` / `Recorder.on_line()` against the requirements and found three gaps, all fixed host-side in `scripts/quest_pose_v1_record.py` (sha256 `916bac63…42f0`; previous version backed up as `data/step04b/quest_pose_v1_record.pre_test3.py.bak`):
1. **Grip availability wasn't required.** It now returns **GRIP_UNAVAILABLE** when the right squeeze action is inactive (`null`), even if the pose is tracked.
2. **A focus-loss *event* didn't invalidate input until the next pose record.** Now a processed `session_state` event other than FOCUSED clears current input immediately (status **UNFOCUSED**). A later FOCUSED *event* alone doesn't restore it; only a FOCUSED, available *pose* record does.
3. **Pose and grip values were shown on the status line for UNFOCUSED, MISSING and similar statuses.** Now values are displayed **only for FRESH_TRACKED**. Raw values stay in the recording.

Also added to each JSONL line: `status_after` (status immediately after processing that line) and `proc_ns` (host processing time from line receipt to that status).

**Rules as tested** (4 new synthetic tests; 63/63 tests pass). Right input is **not** available when:
- the app is unfocused or focus is UNKNOWN (pose `focused=false`, a non-FOCUSED state, or a processed non-FOCUSED event);
- the right pose action is inactive;
- position or orientation is invalid or untracked;
- the grip action is inactive;
- the stream is STALE (watchdog) or DISCONNECTED.

The left controller is not required. The headset status is checked separately (eligibility uses head TRACKED in the analyses; the live status rules don't gate on the head).

**Cue helper:** `scripts/step04b_test2_cues.py` (sha256 `47dc7677…2e9c`) adds the `focus_menu` kind. The release cue is given only after 5 continuous seconds of available right input following the return cue (max 40 s). There's no abort-on-loss.

**Analysis:** `scripts/analyze_step04b_test3.py` (sha256 `c0487c0d…ba5b`).

## C. Planned cues and your confirmation
**Planned and actual cue marks**, host monotonic time relative to the baseline cue:

| Cue | Time |
|---|---|
| ready (tracked + grip ≥ 0.5 for 2 s) | −2.72 s |
| "Keep holding the grip. Baseline." | 0.00 s |
| "Open the system menu now. Keep holding the grip." | 5.00 s |
| "Close the menu and return to the app. Keep holding the grip." | 10.00 s |
| recovery interval captured (5 s available) | 19.18 s |
| "Release the grip now." | 19.18 s |
| "Press the grip again and hold." | 21.45 s |
| end | 26.57 s |

These are scheduled audio-cue starts, not physical button times.

**Your confirmation:**
- Grip held through the menu (opening, open and returning).
- Menu opened and closed with the Meta button, with no recentre.
- You did not report releasing late or long. The telemetry timing of the release is reported below as recorded.

## D. Transition timeline
Host receive time (monotonic) relative to the baseline cue. XrTime is shown separately and never subtracted from host time.

| t (host) | seq | Record | Observation | Status after processing (proc time) |
|---|---|---|---|---|
| 8.757 | 31370 | pose | **Last available right input**: FOCUSED, action active, flags 15, **grip 1.0** | FRESH_TRACKED |
| 8.764 | 31371 | pose | still FOCUSED, right action **active but flags 0** (p, q absent), grip 1.0 | **INVALID** (0.20 ms) |
| 8.777 | 31372 | pose | still FOCUSED, right action **inactive**, grip **null** (first inactive and first null) | MISSING (0.16 ms) |
| 8.792 | 31373 | event | `interaction_profile_changed` | MISSING (0.11 ms) |
| 8.798 | 31374 | event | **`session_state VISIBLE`** (first focus-loss indication), XrTime 15736239487464 | **UNFOCUSED (0.13 ms)** |
| 8.798 | 31375 | pose | first `focused=false` pose (VISIBLE), action inactive, grip null | UNFOCUSED |
| 8.8–14.07 | | poses | UNFOCUSED throughout. Head TRACKED (flags 15) throughout. | UNFOCUSED |
| 14.086 | 31742 | event | **`session_state FOCUSED`** (first focus-restored indication), XrTime 15741531637887 | UNFOCUSED: an event alone doesn't restore input (0.57 ms) |
| 14.086 | 31743 | pose | first FOCUSED pose, right action still **inactive**, grip null | MISSING |
| **14.105** | 31744 | pose | **First available right input after recovery**: action active, flags 15, **grip 1.0** (next 5 samples also 1.0) | **FRESH_TRACKED** |
| 14.124 | 31745 | event | `interaction_profile_changed` | FRESH_TRACKED |

- **Unavailable span:** from the last available record to the first available one, 5.348 s by host receive time and 5360.5 ms by XrTime query time. Each is measured on its own clock.
- **Order:** the right input became invalid and then inactive about 21–34 ms (host order) **before** the `VISIBLE` event arrived. On return, the FOCUSED event came first and the controller became available 19 ms later.
- **No telemetry pause:** the largest host gap between any two lines was 59.5 ms, during the menu-open period. The STALE watchdog was never triggered; no STALE line appears in the 2-Hz console. Focus loss was reported by explicit indications, not by silence.
- **Processing delay** (line receipt to updated diagnostic status): 0.13 ms for the VISIBLE event. Across all lines, median 0.19 ms, p95 0.51 ms, max 4.83 ms. This is host processing only, **not** sensor-to-stop or robot-stop latency.

### Release / repress
Evidence was taken only from records where focus had returned, the grip action was active with a value, and right input was otherwise available. All 701 pose records after the release cue were FRESH_TRACKED; none were unavailable.

| Time | Grip value |
|---|---|
| 19.18 s (release cue) → about 21.7 s | 1.0 throughout (still pressed) |
| **21.791 s** | **First value below 0.5 (0.40)** |
| 21.844 s | 0.0 (fully released) |
| about 22.5–23.1 s | brief values of 0.05–0.12 |
| **24.152 s** | **Back ≥ 0.5 (0.77)**, then 1.0 |

- The recorded release lasted **2.36 s**, starting about 2.6 s after the "release" cue and 0.34 s after the "press again" cue. The cue timing and the physical action differ; the telemetry values are as recorded.
- The grip action stayed active (non-null) throughout the release, so the release is a real value change, not inactivity.
- No other below-0.5 grip values occurred among available records.

### Contradictions check
**0 found** across 2654 pose records:
- `focused` agreed with `session_state` everywhere.
- Inactive pose action never came with nonzero flags.
- Pose-action and grip-action activity always agreed.
- Available status never coincided with unfocused.

One notable but not contradictory record: seq 31371 had the right pose action **active with flags 0** for one record before becoming inactive. It was classified INVALID, not available.

## Stream
| Lines | Pose | Events | Rate | Parse errors | Truncation | Overflow | Max record | Missing seq | Host inter-arrival med/p95/max |
|---|---|---|---|---|---|---|---|---|---|
| 2658 | 2654 | 4 | 60.5/s | 0 | 0 | 0 | 1209 B | 16 | 16.6 / 19.4 / 59.5 ms |

Missing sequence numbers:
- **13 of the 16** fall between 8.72 s and 9.56 s, around the menu opening and focus loss. The others are 2 at about 14.3 s (just after return) and 1 early at −12.9 s.
- Pose and event records share one counter, so the types of the missing records are unknown. The neighbouring records are poses.
- No cause is established. I haven't estimated lost records from a nominal rate.

## Conclusions (separately)
1. **Was the intended interruption observed? Yes.** An explicit `session_state VISIBLE` event and `focused=false` pose records lasted about 5.3 s. The right pose action and grip action were inactive, with `interaction_profile_changed` at both ends. The headset stayed tracked.
2. **Did input become unavailable appropriately? Yes.**
   - Status left FRESH_TRACKED at the first invalid record (INVALID, then MISSING), 34 ms *before* the focus event.
   - The VISIBLE event itself set UNFOCUSED within 0.13 ms of processing.
   - No unfocused, inactive or null record was ever classified as available, and no stale value was displayed as current.
   - The stream never paused, so the watchdog had nothing to detect here. Its behaviour is covered only by the offline tests and earlier runs.
3. **Did fresh input recover after returning? Yes.** The first available record arrived 19 ms after the FOCUSED event. A FOCUSED event alone didn't restore availability.
4. **Was the grip still pressed in the first recovery samples? Yes, per telemetry.** The first available record and the next 5 read grip 1.0, and you confirmed you held it through the menu.
5. **Was a genuine release/repress recorded? Yes**, among available, focused, grip-active records: 1.0 → 0.0 (from 21.79 s) → back to 1.0 (from 24.15 s). The release lasted about 2.4 s rather than the planned 0.5–1 s, and started after the "press again" cue began; the timing doesn't match the cues.

**Still open:**
- the cause of the missing sequence numbers clustered at the focus transition;
- the single "active but flags 0" record type at the transition;
- watchdog behaviour on a real pause (not exercised here);
- untouched-controller drift and inactivity;
- left independence;
- query-time rounding;
- Tests 4–6.

There is no motion-enable latch; "available again" is only a diagnostic status. This is not a declaration of Step 4B being complete or of robot-control readiness.

## State at stop
- The diagnostic app is force-stopped. Both apps are installed and unchanged.
- No recorder, cue or logcat processes remain. Probes are off.
