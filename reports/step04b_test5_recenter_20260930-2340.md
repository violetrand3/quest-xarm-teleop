# Step 04B Test 5: headset recenter and LOCAL reference-space change

Date: 2026-09-30.
- Local PC and its USB-connected Quest only. No SSH, robot, gripper, network or mapping.
- No APK rebuild or install, no data clearing, no SDK changes. Hand tracking was OFF.
- The headset serial is redacted (checked by grep).

**Build:** `org.questxarm.reader` `s4a-ff0da2692500`, installed APK `57f2936ccf1756266bfca75f735eff7fbdb15fb675b7ac5dfca9fdbc61d09f76`, unchanged. `com.rail.oculus.teleop` `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341` is unchanged. Probes off.

## A. Event telemetry actually available (inspection, before the trial)
**Source:** `main.cpp` `AppHandleEvent` at `ff0da26`.
- `XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING` with `referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL` emits `local_space_change_pending`, carrying `time_ns` = **changeTime** (integer XrTime), `pose_valid`, `pose_in_previous_space` (only if `poseValid`), and an incremented `local_epoch`.
- **Any other type** emits `other_space_change_pending` with changeTime only. **The raw `referenceSpaceType` is not emitted**, so non-LOCAL types can't be identified.

**Enum check:** `main.cpp` was compiled against OpenXR **1.1.53** (`openxr_loader_for_android` prefab headers, confirmed via `compile_commands.json`), where VIEW = 1, **LOCAL = 2**, STAGE = 3 and LOCAL_FLOOR = 1000426000. The LOCAL branch is therefore correct.

**Session association:** `sid` (per app run) plus `seq`, with sessions delimited by `session_begin`/`session_end` events. The OpenXR `XrSession` handle is **not** emitted.

**Caution:** every v1 event record carries a constant `"space":"LOCAL"` field. That's the telemetry's pose-space label, **not** the affected space.

**Decoder:** keeps `sid`, `seq`, `event`, `time_ns` (int), `pose_valid` and the app's `local_epoch`. `pose_in_previous_space` stays in the raw JSONL.

The core information for LOCAL changes (type, sid association, integer changeTime) was available. Not available: non-LOCAL raw type and the `XrSession` handle. Nothing was rebuilt.

## B. Analysis changes and tests (host-only)
- **`scripts/analyze_step04b_test5.py`** (sha256 `97962b05…a6`):
  - Classifies each pose record's LOCAL epoch as the count of same-`sid` `local_space_change_pending` events with integer `changeTime ≤` integer `t_query_ns`. Arrival order and non-LOCAL events are ignored.
  - Reports, per change: the nearest eligible before/after poses, the adjacent records, unavailable records within ±1 s, and arrival-order inversions.
  - `poseInPreviousSpace` is kept for diagnosis only. It is never applied, and is treated as unknown when `poseValid` is false.
- **`tests/test_step04b_test5_analysis.py`** (sha256 `a523bd3a…7c`, SYNTHETIC), 6 tests:
  - samples before, exactly at and after changeTime;
  - an event received after post-change samples;
  - an unrelated space type ignored;
  - `poseValid` false keeps the previous transform unknown;
  - session separation with multiple epochs;
  - adjacent large-integer XrTime values without float loss.
- **Full suite: 70/70 pass.**
- **`scripts/step04b_test2_cues.py`** (sha256 `9bd31161…53025`): sub-cues are now also supported without abort-on-loss. New kinds `recenter_es`, `recenter2_es` and `recenter3_es` (Spanish).
- The raw-input availability status is unchanged. Reference changes are reported separately, since a currently tracked pose and continuity of a control reference are different properties.

## C. Attempts and confirmations
Fresh session `73820393bd88a1a4` (launch capture `test5/t5_launch_*`). At startup the runtime emitted `other_space_change_pending` ×3 and one `local_space_change_pending` (seq 8), all before the trials.

| Attempt | Recording | Cues (Spanish) | Your account | Reference events in recording |
|---|---|---|---|---|
| 1 | `t5_recenter_20260930-232242` | Quieto / Mira un poco a la izquierda / Quieto / **Centra ahora** / Quieto | "I didn't hold Meta… I only held the middle-finger button" → **recentre not performed** | **None.** The app's epoch field stayed at 1 throughout. The head turned about +17° and back smoothly, with no frame jump > 30 mm / 5°. Right controller LOCAL unchanged (about 1 mm). |
| 2 | `t5b_recenter_retry_20260930-232742` | as 1, but **"Mantén pulsado el botón Meta con el pulgar" / "Suelta el botón Meta. Sigue con el gatillo lateral."** | "Mira a la izquierda" confused you, so you didn't turn your head. **No confirmation** of the Meta hold or recentre was given. | `other`, **LOCAL (seq 39063)**, `other` |
| 3 | `t5c_recenter_attempt3_20260930-233*` | Quieto / Mantén pulsado el botón Meta… / Suelta el botón Meta… / Quieto (no head-turn step) | **"Held Meta, recentered; grip held throughout; hand stayed still"**; between recordings you took the headset off/on, recentred or used the menu | `other`, **LOCAL (seq 63388)**, `other` |

**Attempt 3 is the primary, confirmed result. Attempt 2 is corroborating** (the telemetry shows the same pattern, but the action wasn't confirmed). Attempts 2 and 3 were made at your request after attempt 1's action wasn't performed, and the cues were clarified each time. The app's epoch counter went from **2** (end of attempt 2) to **9** (start of attempt 3): **7 LOCAL changes were received by the app between recordings and were not recorded.** Their cause isn't established, though you reported taking the headset off/on, recentring or using the menu in between.

## D. Reference-change events and LOCAL coordinate transition
### Event fields
| Attempt | seq | Event | sid | changeTime (XrTime ns) | Host receipt (rel. to first cue) | poseValid | poseInPreviousSpace (raw) |
|---|---|---|---|---|---|---|---|
| 2 | 39062 | other_space_change_pending (type unknown) | 73820393bd88a1a4 | 18625513784773 | 16.386 s | n/a | n/a |
| 2 | **39063** | **local_space_change_pending** | 73820393bd88a1a4 | **18625513805293** | 16.386 s | true | p (−0.040203, 0.045920, −0.345285), q (0, −0.0085484, 0, 0.9999635) |
| 2 | 39064 | other_space_change_pending | 73820393bd88a1a4 | 18625513809564 | 16.386 s | n/a | n/a |
| 3 | 63387 | other_space_change_pending | 73820393bd88a1a4 | 18993630853475 | 9.113 s | n/a | n/a |
| 3 | **63388** | **local_space_change_pending** | 73820393bd88a1a4 | **18993630877694** | 9.114 s | true | p (−0.024167, −0.015653, 0.020933), q (0, 0.0139812, 0, 0.9999022) |
| 3 | 63389 | other_space_change_pending | 73820393bd88a1a4 | 18993630881808 | 9.114 s | n/a | n/a |

Only seq 39063 and seq 63388 affect the LOCAL space of the published poses. The `other` events are kept separate and were not merged.

### Before/after classification by changeTime (attempt 3; attempt 2 in brackets)
| Record | seq | t_query − changeTime | Head p (LOCAL, m) | Right p (LOCAL, m) | Grip | App epoch field | Status |
|---|---|---|---|---|---|---|---|
| last before | 63384 [39059] | −6.18 ms [−13.14] | (0.0248, 0.0157, −0.0203) | (0.1556, −0.1837, −0.5325) | 1.0 [0.98] | 9 [1] | FRESH_TRACKED |
| **after by time, old coords** | 63385 [39060] | **+7.71 ms** [+0.74] | (0.0248, 0.0156, −0.0203) | (0.1556, −0.1837, −0.5325) | 1.0 | 9 [1] | FRESH_TRACKED |
| **after by time, old coords** | 63386 [39061] | **+21.60 ms** [+28.52] | (0.0247, 0.0156, −0.0203) | (0.1556, −0.1837, −0.5324) | 1.0 | 9 [1] | FRESH_TRACKED |
| (3 event records) | 63387–63389 [39062–39064] | | | | | | |
| **first new coords** | **63390** [39065] | **+35.48 ms** [+42.41] | (0.0000, −0.0001, −0.0001) | (0.1165, −0.1993, −0.5156) | 1.0 [0.98] | **10** [2] | FRESH_TRACKED |

**Observation:** in both attempts, the **two pose records queried after changeTime still carried old-frame coordinates.** The coordinate change first appears in the first pose record **after the event records**: +35.5 ms and +42.4 ms past changeTime. That's the same record where the app's own epoch field increments. With this build and runtime, then, the **published** poses change frame at about the point where the app *receives* the event, not exactly at changeTime.

A classification by changeTime alone (as specified, and as `analyze_step04b_test5.py` does) labels these two records "new epoch" although their coordinates are old. Both facts are reported; nothing was re-labelled or stitched. The data doesn't show whether the runtime's frame switch follows event delivery, `xrPollEvent` or the next `xrLocateSpace`.

### Observed coordinate change across the boundary (last old-coordinate record → first new-coordinate record)
| | Attempt 3 (63386 → 63390, 13.9 ms apart in XrTime) | Attempt 2 (39061 → 39065, 13.9 ms) |
|---|---|---|
| Head Δpos | **35.6 mm** | **350.6 mm** |
| Right Δpos | **45.4 mm** | **347.5 mm** |
| Head / right Δangle | about 0.8° (quaternion distance) | about 1.0° |
| Old←new transform from head vs from right controller | consistent (attempt 2: 0.1 mm / 0.03°) | — |

- **These are coordinate changes**, not hand velocity, tracking drift or robot displacement. The head and right controller imply the same rigid old←new transform. Physical movement while pressing Meta remains a possible small contribution.
- After each change, the head sits at about the new LOCAL origin ((0, 0, 0) ± 0.2 mm).
- **Using changeTime-only "nearest eligible" pairs** (63384 vs 63385, 39059 vs 39060) would show **0.0 mm / 0.02°**, because both of those records are old-frame. This shows that the epoch boundary by query time and the observed coordinate switch did not coincide in this data.

**`poseInPreviousSpace` (diagnostic only, not applied):** in attempt 2 the emitted pose differs from the measured old←new transform (head-derived, assuming negligible physical motion over 13.9 ms) by 701 mm. Its **inverse** matches to **0.0 mm / 0.02°**. In attempt 3 the emitted p (−0.0242, −0.0157, 0.0209) is likewise the negative of the old head position (0.0247, 0.0156, −0.0203). So the emitted value behaves as **T_new←old (previous origin in the new space)** for this runtime and build. Any future use must account for this direction. I haven't checked it against the OpenXR spec's wording.

### Input availability through the change
In attempts 2 and 3, raw right input stayed **available (FRESH_TRACKED) throughout**: FOCUSED, right action active, flags 15, head flags 15, grip action active at 0.96–1.0. There were 0 unavailable records within ±1 s of either boundary, and no focus or profile events.

This only means raw input stayed tracked and available. It does **not** mean continued robot motion would be safe: the LOCAL coordinates jumped by 35–350 mm while input stayed "available". A future control layer must invalidate its reference on such events.

## Stream
| Recording | Lines | Pose | Events | Rate | Parse errors | Truncation | Overflow | Max record | Missing seq | Host inter-arrival med/p95/max |
|---|---|---|---|---|---|---|---|---|---|---|
| Attempt 1 | 2962 | 2962 | 0 | 61.3/s | 0 | 0 | 0 | 1224 B | 1 (17155, at −12.3 s) | 14.9 / 45.5 / 48.1 ms |
| Attempt 2 | 2841 | 2838 | 3 | 58.8/s | 0 | 0 | 0 | 1225 B | 1 (37376, near recording start) | 15.8 / 45.6 / 48.3 ms |
| Attempt 3 | 2396 | 2393 | 3 | 57.9/s | 0 | 0 | 0 | 1225 B | 1 (62145, near recording start) | 17.1 / 43.9 / 48.8 ms |

- Pose and event records share one sequence counter, so each missing record's type is unknown. In each recording the app's epoch field didn't change across the gap, which rules out a **LOCAL** change being the missing record. A non-LOCAL event or a pose can't be ruled out.
- Receiving these events doesn't prove lossless event delivery: the 7 between-recording LOCAL changes were seen only through the app's counter.

## Conclusions
1. **Was a LOCAL reference change captured? Yes:** attempt 3 (confirmed by you) and attempt 2 (unconfirmed action). In each, `local_space_change_pending` came with integer changeTime, `poseValid` true and `poseInPreviousSpace`, in session `73820393bd88a1a4`, alongside two `other_space_change_pending` events of unknown type.
2. **Could old/new pose records be identified? Yes**, with an important qualification. By the specified changeTime rule: 1240 / 1153 records (attempt 3) and 1684 / 1154 (attempt 2). But the published coordinates actually switched **35–42 ms after changeTime**, at the first pose record after the event. Two records per attempt are "new" by query time and "old" by coordinates. Both the time rule and the observed switch are reported, and neither is hidden.
3. **Raw input stayed available throughout.** This is availability only, not continuity of a control reference.
4. **Limitations:**
   - non-LOCAL space types aren't identifiable;
   - the `XrSession` handle isn't emitted;
   - 7 LOCAL changes between recordings weren't captured;
   - event-delivery completeness is unproven;
   - the direction of `poseInPreviousSpace` was observed, not verified against the spec;
   - query-time rounding and end-to-end latency are not resolved.

Host elapsed times and OpenXR times were kept separate, and audio cue times are not physical event times. This is not a declaration of Step 4B being complete or of robot-control readiness.

## Files and state
- `data/step04b/test5/`: `t5_launch_*`, `t5_recenter_*`, `t5b_recenter_retry_*` and `t5c_recenter_attempt3_*`. Each has `.jsonl` (raw), `.summary.json`, `.marks.txt`, `.planned.json`, `.user_preconfirm.txt` and `.test5.json`.
- The diagnostic app is force-stopped. Both apps are installed and unchanged.
- No recorder, cue or logcat processes remain. Probes are off.
