# Step 04B Test 2 follow-up: supported-hand (held) controller with moving head

Date: 2026-09-30.
- Local PC and its USB-connected Quest only. No SSH, robot, gripper or network change.
- No APK rebuild or install, no app-data clearing, no device-setting experiments. Hand tracking was OFF throughout.
- The headset serial is redacted (checked by grep).

**Build:** `org.questxarm.reader` `s4a-ff0da2692500`, installed APK `57f2936ccf1756266bfca75f735eff7fbdb15fb675b7ac5dfca9fdbc61d09f76`, unchanged. `com.rail.oculus.teleop` `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341` is unchanged. Probes off (`debug.questxarm.logprobe=0`).

**The earlier untouched-controller Test 2 stays INCONCLUSIVE** (`reports/step04b_test2_stationary_20260930-2145.md`). This follow-up does not change that result.

## A. Preparation
- **Acquisition:** the verified recorder (`--pre-attach --expect-build s4a-ff0da2692500`), unchanged.
  - Session `7b5afa3c26013e19` was used for attempt 1.
  - At your request the app was reopened for a fresh session, `a2536ec742b1d30a`, used for all later recordings.
  - No LOCAL reference-space change occurred in any recording. Each recording contains one session only.
- **Read-only source check** (commit `ff0da26`, SDK `bbed2f2`): **holding the side grip triggers no haptics and changes no app behaviour.**
  - The grip/squeeze value is read into the telemetry, the framework fields (`RightRemoteGripTrigger`, the `AllButtons` grip bit, which nothing consumes) and the on-screen text labels "sqVal" / "tf".
  - Haptics fire only on A/B/X/Y press edges or on a UI-button click. A click needs the aim ray on the button plus the **index trigger** above 0.25.
  - A, B and the index trigger were not used as keep-alive inputs.
- **Host-side changes only:**
  - `scripts/step04b_test2_cues.py` (sha256 `09020dcc…5b1c`): new kinds `held`, `held_rotate`, `held_translate` and `held_translate_guided`; opt-in `--abort-on-loss`, which ends the measurement after 0.3 s of right input that isn't fresh, focused, active and tracked, then keeps 3 s of diagnostic tail; readiness-only `--start-min-squeeze`. Previous kinds and behaviour are unchanged; a backup is in `data/step04b/`.
  - `scripts/analyze_step04b_test2.py` (sha256 `1f0c2c78…4adb`): phases are taken from the marks, the measurement ends at the first `input_lost` mark, and per-phase grip value and activity are reported. Re-running it on Test 2 Recording A reproduced every earlier phase statistic and the baseline exactly.
- **Measurement definition (unchanged):**
  - Eligible records: FOCUSED; right pose action active; head and right grip position/orientation VALID and TRACKED; finite and structurally valid.
  - Baseline: the first eligible record at least 2.0 s after the first phase's start mark (`still_1`, or `held_still`), fixed for the trial.
  - Head-relative pose: `T_HEAD_RIGHT = inverse(T_LOCAL_HEAD)·T_LOCAL_RIGHT` from the **same record**.
  - Angles: quaternion `2·acos|⟨q1,q2⟩|` / `acos((tr(R1ᵀR2)−1)/2)`.

## B. Held-controller availability
| Recording | Your confirmation | Result |
|---|---|---|
| **Attempt 1**, `held/h1_availability_*` | Supported hand, **held but grip NOT pressed** ("didn't know I had to press the middle finger button"); no buttons or jiggle | **Input lost.** See below. Measurement ended by the cue helper (`input_lost` at +2.27 s). |
| **Retry (your request)**, `held/h2_availability_retry_grip_*` | Grip pressed continuously, hand supported, no buttons or jiggle | **Available for all 15.0 s.** 901/901 eligible (100 %); longest eligible gap 20.9 ms. |

**Attempt 1 details:**
- **Grip:** the right squeeze value was **0.0** in every active record (the action was active, so this is a real 0.0). `thumbrest_touch` was `true` throughout.
- **Activity:** the right pose action had been active since before the phase, for at least 14.6 s, with the controller moving only about 1–4 mm of path per second. It went **inactive at +1.85 s for 0.79 s** (flags 0, all right inputs `null`), then became active again at +2.64 s.
- **Events:** both switches coincided with `interaction_profile_changed`.
- **Unchanged:** the head (flags 15), FOCUSED and the session.
- **Other:** 9 missing sequence numbers in about 28 s.

**Retry details:**
- **Grip:** 1.0 in all 901 records; `thumbrest_touch` true.
- **Tracking:** the right grip was flags 15 throughout, with no `interaction_profile_changed` events and the head tracked.
- **Motion:** right LOCAL change from baseline 0.23 / 0.43 / 0.49 mm and 0.15 / 0.28 / 0.31° (median/p95/max). Head 2.2 / 4.5 / 4.8 mm and 1.2 / 1.9 / 2.1°.

**Diagnostic fields available around the attempt-1 transition:**
- right `active`, `flags`, the per-bit valid/tracked booleans, and every right input (all `null` while inactive);
- head flags, `focused`, `session_state`, `t_query_ns`;
- the `interaction_profile_changed` events.

**Interaction-profile paths are NOT in the telemetry:** the v1 event carries no path. They stay missing and were not added (no rebuild). Upstream also shows profile text in the headset UI, which isn't logged. By inspection, upstream fills the "Right IP" label from `LeftHandPath`, so even that label would not show the right hand's profile.

## C. Head-motion trials (held, grip pressed, supported hand)
**Your confirmations:**
- **Trial 1 (rotation):** "nothing unusual". Hand stayed supported, grip held, head rotated gently, no vibration, no deliberate controller motion.
- **Trial 2, first attempt** (`held/h4_trial2_translate_*`): **incomplete by your report**. You hadn't finished the side-to-side and back-and-forth leaning when "hold still" came. Recorded but **not used** for conclusions; for reference, head max 128 mm / 29.8°, right LOCAL max 1.52 mm / 3.58°, coverage 100 %.
- **Trial 2 retry** (`held/h5_trial2_translate_retry_guided_*`, your request), with spoken sub-cues inside the same 10-s move phase ("Lean left / right / forward / back"; phase boundaries unchanged): "nothing unusual".

All three phases of both trials had **100 % coverage**. Grip was 1.0 in every record (no nulls), there were no exclusions and no profile changes, and the maximum gap between eligible records was ≤ 48.2 ms.

Values are median / p95 / max of the change from each trial's fixed baseline (Trial 1 baseline +2.003 s; Trial 2 retry +2.008 s).

### Trial 1: head rotation (`held/h3_trial1_rotate_*`)
| Phase | Eligible | Head Δpos mm | Head Δang ° | Right LOCAL Δpos mm | Right LOCAL Δang ° | Head-relative Δpos mm | Head-relative Δang ° |
|---|---|---|---|---|---|---|---|
| still_1 (5.0 s) | 303/303 | 0.73 / 1.03 / 1.20 | 0.13 / 0.33 / 0.37 | 0.14 / 0.33 / 0.40 | 0.07 / 0.11 / 0.13 | 1.11 / 3.57 / 4.14 | 0.16 / 0.36 / 0.39 |
| move (10.0 s) | 591/591 | 12.7 / 43.2 / 45.4 | 6.72 / 26.3 / **28.9** | 0.60 / 0.85 / **0.97** | 0.27 / 0.49 / **0.52** | 55.1 / 236.9 / 260.4 | 6.86 / 26.4 / 29.0 |
| still_2 (5.0 s) | 298/298 | 13.2 / 17.5 / 19.4 | 5.80 / 8.90 / 10.6 | 0.50 / 0.73 / 0.81 | 0.45 / 0.50 / 0.53 | 51.0 / 83.4 / 99.1 | 5.72 / 8.81 / 10.5 |

### Trial 2 retry: guided head translation/leaning (`held/h5_trial2_translate_retry_guided_*`)
| Phase | Eligible | Head Δpos mm | Head Δang ° | Right LOCAL Δpos mm | Right LOCAL Δang ° | Head-relative Δpos mm | Head-relative Δang ° |
|---|---|---|---|---|---|---|---|
| still_1 (5.0 s) | 284/284 | 0.97 / 1.51 / 1.60 | 0.18 / 0.66 / 0.76 | 0.24 / 0.53 / 0.57 | 0.08 / 0.21 / 0.31 | 1.45 / 6.46 / 7.17 | 0.18 / 0.68 / 0.77 |
| move (10.0 s) | 572/572 | 87.3 / 136.6 / **137.3** | 23.0 / 41.1 / 42.6 | 0.94 / 2.68 / **3.11** | 0.92 / 2.30 / **2.79** | 79.6 / 275.9 / 384.4 | 22.8 / 41.0 / 42.4 |
| still_2 (5.0 s) | 289/289 | 22.6 / 107.3 / 107.5 | 3.68 / 23.6 / 23.8 | 2.17 / 2.60 / 2.69 | 1.80 / 2.43 / 2.47 | 51.3 / 202.4 / 204.4 | 5.36 / 23.2 / 23.4 |

**Head translation recorded (LOCAL axes):**
- "Lean left" window: about −90 mm in x.
- "Lean right" window: up to −122 mm in x. You were still moving through the previous lean; the cues were followed with some lag.
- "Lean forward": −109 mm in z and −79 mm in y.
- "Lean back": about +101 mm in z at the start of still_2.

## D. Stream and open-issue characterization
| Recording | Lines | Pose | Events | Rate | Parse errors | Truncation | Overflow | Max record | Missing seq | Host pose inter-arrival med/p95/max ms |
|---|---|---|---|---|---|---|---|---|---|---|
| h1 attempt 1 | 1727 | 1725 | 2 | 60.3/s | 0 | 0 | 0 | 1270 B | 9 | 16.4 / 41.6 / 59.7 |
| h2 availability retry | 1946 | 1946 | 0 | 59.8/s | 0 | 0 | 0 | 1285 B | 1 | 16.7 / 17.8 / 21.8 |
| h3 Trial 1 | 2220 | 2220 | 0 | 59.2/s | 0 | 0 | 0 | 1286 B | 1 | 16.6 / 43.4 / 48.2 |
| h4 Trial 2 (incomplete) | 2159 | 2159 | 0 | 57.7/s | 0 | 0 | 0 | 1224 B | 2 | 17.1 / 43.0 / 48.0 |
| h5 Trial 2 retry | 2142 | 2142 | 0 | 57.1/s | 0 | 0 | 0 | 1223 B | 0 | 17.3 / 43.9 / 48.1 |

- **Shared counter:** pose and event records share one sequence counter. Where a seq is missing, its type is unknown, though in these files the neighbouring records were poses. Cause not established.
- **XrTime query step:** median 13.89 ms (about 72 Hz) in all recordings. The received pose rate was about 57–60/s; the difference is not explained.
- **Unchanged open items:** query-time rounding (`ToXrTime(double seconds)`) is still unmeasured and unchanged.

## Conclusions (separately)
1. **Continuous availability while held with the grip pressed: yes, for these recordings.**
   - h2, h3 and h5 each had 100 % eligible coverage, with grip 1.0 throughout and no `interaction_profile_changed` events: about 15 + 20 + 20 s, plus the waiting periods.
   - When the grip was *not* pressed (h1), the right input became inactive for 0.79 s while held. That's one occurrence; the cause is not established.
2. **Substantial headset movement recorded: yes.** Trial 1 rotation up to 28.9° (45 mm). Trial 2 retry translation up to 137 mm, with about 100 mm along each guided direction, plus up to 42.6° of accompanying rotation.
3. **LOCAL-space behaviour supports head-independent output: yes.**
   - While the head rotated up to 28.9°, the right controller's LOCAL pose changed by at most 0.97 mm / 0.52°. While the head translated up to 137 mm, it changed by at most 3.11 mm / 2.79°.
   - Over the same records, the same-record head-relative pose changed by up to 260 mm / 29.0° and 384 mm / 42.4°, following the head.
4. **Remaining confounds and unknowns:**
   - **The residual is not pure tracking drift and not a calibrated accuracy figure.** The hand was supported but neither immobilized nor independently measured, so the 0.4–3.1 mm and 0.3–2.8° residuals include real hand and wrist motion. That's especially likely while leaning (Trial 2) and in the post-movement still phases, which ended with your head away from baseline: still_2 head median 13–23 mm.
   - The hands-off untouched case remains inconclusive.
   - The cause of the inactivity and its link to `interaction_profile_changed` is unknown. Profile paths are not logged.
   - The missing sequence numbers, the gap between the ~72 Hz query step and the ~58–60/s received rate, and query-time rounding are all unexplained or unmeasured.
   - Left-controller independence is still unconfirmed (from Test 1).
   - Focus loss, occlusion, recentre and restart (Tests 3–6) have not been run.

This is not a declaration of robot-control suitability or of Step 4B being complete.

## Files
- `data/step04b/held/`: `h_launch_*`, `h_launch2_*`, and `h1_…`–`h5_…`. Each recording has `.jsonl` (raw), `.summary.json`, `.marks.txt` (including sub-cue marks), `.planned.json`, `.user_preconfirm.txt` and `.analysis.json`.

## State at stop
- The diagnostic app is force-stopped. Both apps are installed and unchanged.
- No recorder, cue or logcat client processes remain.
