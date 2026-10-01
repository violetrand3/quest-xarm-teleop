# Step 04B Test 2: stationary right controller, moving headset (STOPPED, inconclusive)

Date: 2026-09-30.
- Local PC and its USB-connected Quest only. No SSH, robot or network.
- No APK change or reinstall; no logging, rate, frame, flag or timing change.
- The headset serial is redacted (checked by grep).

**Build:** `org.questxarm.reader` `s4a-ff0da2692500`, installed APK SHA-256 `57f2936ccf1756266bfca75f735eff7fbdb15fb675b7ac5dfca9fdbc61d09f76` (unchanged). `com.rail.oculus.teleop` is unchanged (`ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341`). Probes were off (`debug.questxarm.logprobe=0`).

## Acquisition
- **Recorder:** the verified one (`quest_pose_v1_record.py --pre-attach --expect-build s4a-ff0da2692500`), unchanged.
- **Launch capture:** `data/step04b/test2/t2_launch_20260930-*` shows a fresh session `d907e8ef6f6a05e3` (session_begin → IDLE → … → FOCUSED). It had 0 probes, 0 overflows and 0 parse errors.
- **Same session for all trials.** No LOCAL reference-space change occurred inside any trial window, checked against event `changeTime` versus `t_query_ns` in the XrTime domain. The only LOCAL change was at session start, before the trials.
- **Host-only additions** (the recorder and parser are unchanged):
  - `scripts/step04b_test2_cues.py`: spoken cues via the existing `spd-say`, which you confirmed you could hear. It waits until the right grip has been FRESH/FOCUSED/active/TRACKED for 2 s, reading the recorder's JSONL only, then writes the phase marks (host monotonic, the same clock as the recorder's `host_ns`) and the planned sequence (`*.planned.json`).
  - `scripts/analyze_step04b_test2.py`: the offline analysis.
- **Planned cues:** "Hold your head still" (5 s), "Now gently rotate your head…" (10 s), "Hold your head still again" (5 s), "Stop". The marks are set when each spoken cue *starts*.

## Method (documented before computing)
- **Matrix convention:** `T_LOCAL_X = pose_matrix(p, q)` is 4×4 in the column-vector convention and maps X-frame points into LOCAL. Head-relative: `T_HEAD_RIGHT = inverse(T_LOCAL_HEAD) · T_LOCAL_RIGHT`, using head and right poses from the **same record** (the same `t_query_ns`).
- **Eligible samples:**
  - FOCUSED;
  - right pose action active;
  - head and right grip both position/orientation VALID and TRACKED;
  - finite, structurally valid values (enforced by the decoder).
- **Baseline:** the first eligible record received ≥ 2.0 s after the `still_1` cue mark and before the `move` mark. It is fixed for the whole trial and never reset.
- **Angles:**
  - quaternions: `2·acos(|⟨q1,q2⟩|)`;
  - head-relative rotation matrices: `acos((tr(R1ᵀR2) − 1)/2)`;
  - both clamped. No Euler angles.
- **Exclusions:** excluded records are counted by reason. Nothing is interpolated or zero-filled, and outliers are not removed.

## Trials
### Recording A: head rotation (`t2A_rotate_20260930-213234`)
**Your confirmation (separate from the plan):**
- You rotated your head.
- **You bumped the controller during the phases** "because otherwise you were not detecting it".
- Your head was **not really still** in the "hold still" phases.

**Status: contaminated.** It does not meet Test 2's stationary-controller condition. The numbers are reported for transparency only.

| Phase | Records | Eligible (coverage) | Longest eligible gap | Head Δpos mm (med/p95/max) | Head Δang ° | Right LOCAL Δpos mm | Right LOCAL Δang ° | Right head-relative Δpos mm | Head-relative Δang ° |
|---|---|---|---|---|---|---|---|---|---|
| still_1 (5 s) | 320 | 241 (75.3 %) | 47 ms | 60.1 / 111.0 / 111.8 | 5.6 / 11.3 / 11.3 | 0.31 / 0.73 / 0.80 | 0.01 / 0.01 / 0.02 | 79.0 / 142.0 / 146.0 | 5.6 / 11.3 / 11.3 |
| move (10 s) | 626 | 531 (84.8 %) | 667 ms | 117.7 / 189.1 / 196.4 | 21.2 / 47.5 / 51.6 | 2.05 / 3.68 / 4.47 | 0.05 / 2.01 / 2.91 | 290.7 / 485.4 / 513.9 | 20.8 / 45.8 / 49.9 |
| still_2 (5 s) | 334 | 121 (36.2 %) | 3041 ms | 155.8 / 170.8 / 171.3 | 15.0 / 19.7 / 22.6 | 2.73 / 3.09 / 3.11 | 1.98 / 2.00 / 2.01 | 261.8 / 361.1 / 363.5 | 13.6 / 19.8 / 20.8 |

- The baseline was at +2.025 s (seq 9814). The only exclusion reason was `right_action_inactive`: 79 / 95 / 213 records by phase.
- The resting pose of the right controller differed between its active segments by up to about 4.5 mm and about 2.9° (consistent with your reported bumps). Within segments during the phases, drift was 0.8–3.4 mm.

### Recording A2: head-rotation retry (`t2A2_rotate_retry_20260930-213701`)
**Your confirmation:** you agreed beforehand not to touch the controller after "Starting" even if it disappeared. Hand tracking was OFF, the controller was on the table, and the app was in front.

**Status: INCONCLUSIVE.** There were **0 eligible records** in every phase: still_1 0/342, move 0/688, still_2 0/340. The only reason was `right_action_inactive`, and no baseline could be selected.
- The right pose action became active 4.85 s before the first cue and **went inactive 0.27 s before it**.
  - That active window lasted 275 records, XrTime span **4569 ms**.
  - During that window the controller was essentially motionless: within 1.63 mm and 0.03° of its final active pose, and within 1 mm / 0.2° for the last 2.47 s.
- It then **stayed inactive for the whole 20-s trial and beyond**: an unavailable interval of 1700 records, XrTime span 24.76 s to the end of the recording.
- The headset stayed tracked (flags 15) and the app stayed FOCUSED throughout.

### Recording B: head translation
**Not performed**, by your decision after A2. It would have faced the same input-availability limit.

## Open-issue characterization (no causes assigned)
- **Right pose-action inactivity:**
  - In A, the active segments lasted 4.7–4.8 s each (e.g. 5.73→10.44 s, 11.11→15.84 s and 18.88→23.69 s relative to the first cue), separated by inactive intervals. Inactive intervals were measured from the first to the last inactive record, using `t_query_ns` (XrTime) differences: 23.5 s (before the trial), 1.8 s, 0.64 s, 3.0 s and 0.72 s.
  - In A2 there was one active window of 4.57 s, then 24.8 s+ inactive.
- **Correlation with events:** every active↔inactive switch of the right pose action coincided (within about 15 ms of host arrival) with an `interaction_profile_changed` event. These are the only events in A and A2.
- **Validity and tracking:** whenever the right pose action was active, the right grip was fully VALID and TRACKED (flags 15). There were no partial validity or tracking transitions in either trial. When inactive, the flags were 0.
- **Headset and focus:** the head was always TRACKED (flags 15) and the app always FOCUSED. Neither changed at the switches.
- **What is not claimed:** I am not attributing the inactivity to controller sleep, occlusion, the runtime or an app bug. The data shows only that a resting, untouched right controller's pose action stayed active for about 4.6–4.8 s, then became inactive until it was disturbed, together with `interaction_profile_changed`.

## Stream statistics
| Recording | Lines | Pose | Events | Rate | Parse errors | Truncation | Overflow | Max record | Missing seq | Host pose inter-arrival med/p95/max | XrTime step med/p95/max |
|---|---|---|---|---|---|---|---|---|---|---|---|
| A | 3664 | 3655 | 9 | 66.5/s | 0 | 0 | 0 | 1287 B | 1 (7593) | 12.8 / 44.7 / 48.4 ms | 13.89 / 27.78 / 27.78 ms |
| A2 | 3967 | 3965 | 2 | 67.7/s | 0 | 0 | 0 | 1287 B | 1 (25573) | 12.7 / 45.4 / 48.4 ms | 13.89 / 27.78 / 41.66 ms |

Pose and event records share one sequence counter, so the type of each missing record is unknown. Its cause is not established.

## Conclusions (separately)
1. **LOCAL versus head-relative output:** only the contaminated Recording A has data.
   - There, while the head moved up to 196 mm and 51.6°, the right controller's LOCAL pose changed by at most 4.5 mm and 2.9°, despite the reported bumps.
   - The same-record head-relative pose changed by up to 514 mm and 49.9°, tracking the head's rotation.
   - This is **qualitatively consistent with LOCAL (not head-relative) output**, but it is **not a clean Test 2 result**: the controller was disturbed and there was no clean stationary trial.
2. **Residual variation:** **not measured cleanly.** A's figures include physical bumps. A2's only stationary data, from before the cues, shows ≤ 1.6 mm and ≤ 0.03° over 4.6 s, with no head-movement phase.
3. **Input availability during the tested motions:** **insufficient.** An untouched resting controller was available for only about 4.6–4.8 s, so a 20-s stationary trial can't be measured as specified.
4. **Remaining unknowns:**
   - the cause of the ~4.6–4.8 s inactivity and its link to `interaction_profile_changed`;
   - clean residual LOCAL drift during head rotation and during translation (B was not run);
   - the cause of the single missing sequence numbers;
   - query-time rounding (`ToXrTime(double seconds)`), still unmeasured and not changed;
   - left-controller independence (from Test 1).

Test 2 has **not passed**. Tests 3–6 were not performed. No claim is made about robot-control suitability or about Step 4B being complete.

## Files
- `data/step04b/test2/`:
  - `t2_launch_*`;
  - `t2A_rotate_*` and `t2A2_rotate_retry_*`, each with `.jsonl` (raw), `.summary.json`, `.marks.txt`, `.planned.json`, `.user_preconfirm.txt` and `.analysis.json`.
- `scripts/step04b_test2_cues.py` (sha256 `d439ea3c…a4f8`) and `scripts/analyze_step04b_test2.py` (sha256 `1c0e91b4…0bc8`).

## State at stop
- The diagnostic app is force-stopped. Both apps are installed and unchanged.
- No recorder, cue or logcat processes remain.
