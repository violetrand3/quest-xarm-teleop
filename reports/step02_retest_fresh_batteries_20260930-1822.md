# Step 02 retest: fresh controller batteries

Date: 2026-09-30. Code and APK were unchanged since `step02_input_monitor_20260930-1645.md`. The app was brought to the foreground with `am start` only.
The headset had to be re-authorized for USB debugging after the battery change and a headset restart.

- Invocation: `python3 scripts/quest_input_monitor.py --serial [REDACTED] --duration 90 --output data/step02/run4_fresh_batteries_20260930-181925.jsonl --summary data/step02/run4_fresh_batteries_20260930-181925.summary.json`
- Manual confirmation: you said "done" after performing the six steps. You did not give exact timings. The recorded button events run about 10 s behind the suggested schedule, so I assign pose windows to steps only where the data is clear.

## Recorded evidence

**Transport:**
- 6,447 valid messages in 90.0 s, 71.6 Hz.
- 0 parse errors.
- Inter-arrival gaps: 13.9 ms median, 15.4 ms p95, 114.4 ms max (at 35.1 s).
- LIVE for 100% of samples.

**Analog inputs:** rightTrig and rightGrip were present in 6,445 of 6,447 messages. The 2 messages without them had no buttons for either hand.
| Field | Min | Max |
|---|---|---|
| rightTrig | 0.0 | 1.0 |
| rightGrip | 0.0 | 1.0 |
| leftTrig | 0.0 | 0.0 |
| leftGrip | 0.0 | 0.215 |

**Booleans:**
| Key | Press episodes | Messages True | Messages False | First press |
|---|---|---|---|---|
| A | 3 | 40 | 6,405 | 33.2 s |
| B | 3 | 38 | 6,407 | 35.2 s |
| RTr | 3 | | | 23.0 s |
| RG | 4 | | | 26.6 s |

- A and B show both pressed and released states.
- X and Y were never pressed, which matches the script.

**Poses:**
- r and l were both present in 6,445 messages.
- Degenerate matrices appear at 60.29–60.33 s: 4 messages, for both r and l. The bottom row is fine, but max |RᵀR−I| = 0.94 and det = 0.016. These were flagged by the homogeneous check.
- The next 2 messages (60.35–60.36 s) have no poses and no buttons. This looks like a tracking or re-initialisation glitch.
- **Left/right duplication persists.** r == l exactly in these stretches: 5.3–8.4 s, 60.3–62.0 s, 70.4–73.6 s, 81.8–82.0 s and 88.8–89.3 s, plus single instants at 9.4 s and 70.2 s. Two physical controllers can't have identical poses, so during those stretches at least one label is not tracking its own controller.
- **Pose-label response (inconclusive):**
  - I used a frame-independent check: the distance and relative rotation between r and l.
  - At 55–60 s both controllers were rigidly still relative to each other (distance 0.647–0.651, relative rotation about 132°).
  - At 75–80 s r was nearly stationary (translation spread about 0.009) while l moved (spread about 0.034, rotation change about 0.14). That fits step 6.
  - At 85–90 s r moved much more than l, which doesn't fit step 6 unless it was done early.
  - Without your timings I can't confirm that each label follows the expected controller.
- Tracking-valid flag: none in the protocol, so tracking validity is still unknown.

## Status

- The "inputs stay zero" blocker is resolved: empty controller batteries were the cause.
- Superseded by the addendum below: the L/R duplication was caused by hand tracking and is gone with hand tracking off, and the pose labels are now confirmed.
  No degenerate matrices occurred in the three clean runs (5a4, 5b, 5c).
- No app replacement or parser change was made.

## Addendum: pose-label check with open-ended recordings (18:23–18:34)

I used the same monitor with an explicit `--serial [REDACTED]` and no `--duration`. Each recording was stopped with SIGINT when you said "done". I made no code or APK changes.
Each recording ended with SIGINT and left no adb client behind.

| Recording | Setup | Msgs | Rate | Errors / homog fails | L/R duplicates | Result |
|---|---|---|---|---|---|---|
| run5a_both_still | controllers picked up and put down after the start (your account) | 4264 | 71.6 Hz | 0 / 3 | 2733 (64%) | contaminated by handling |
| run5a2_both_still | on the table from the start, **hand tracking on** | 2818 | 71.2 Hz | 0 / 0 | 872 (31%) | steady until the headset was worn (~6 s), then poses moved and duplicated |
| run5a3_both_still_nohands | hand tracking **off**, Settings panel focused | 2380 | 71.7 Hz | 0 / 0 | n/a | every message empty (`&`, no poses or buttons), because the app lacked input focus |
| run5a4_both_still_nohands | hand tracking off, app focused, controllers woken | 2881 | 71.7 Hz | 0 / 0 | **0** | distance r–l constant 0.197–0.198 for 40 s |
| run5b_right_only | right moved, left on table | 4466 | 71.9 Hz | 0 / 0 | **0** | \|r\| went down to 0.33 (in hand), \|l\| stayed 0.64–0.72, r–l went 0.199 → 0.184–0.690 → 0.260 |
| run5c_left_only | left moved, right on table | 3537 | 71.8 Hz | 0 / 0 | **0** | \|l\| went down to 0.29 (in hand), \|r\| stayed 0.63–0.85, r–l went 0.259 → 0.230–0.620 → 0.243 |

Gap figures for the last three runs:
- Median: about 13.9 ms.
- p95: about 15.4 ms.
- Max: 27–29 ms.
- No gaps over 500 ms.

### Findings
- **Pose labels are correct.** `r` follows the right controller and `l` follows the left one. The evidence is the distance from the pose origin (|t|), which head rotation can't change, together with the r–l distance.
  This matches your manual confirmation of which controller you moved in each recording.
- **The poses are headset-relative.** With both controllers untouched, the two poses translated and rotated together as your head moved, while the distance between them stayed fixed. So the pose origin moves with the head. This matters for the later robot mapping and has not been converted.
- **Hand tracking caused the L/R duplication.** With hand tracking disabled in the headset and the app focused, there were 0 duplicates across 10,884 messages.
  Hand tracking was the only headset setting you changed. The monitor, parser and APK were unchanged.
- **The app must have input focus.** While a system panel had focus, the stream kept running at 72 Hz but every message was empty. The monitor correctly showed the poses as missing and did not reuse old values.
- `RThU`/`LThU` read True even with the controllers resting on the table, so they are not a reliable "held" indicator.
- There is still no tracking-valid flag, so tracking validity stays unknown.

### Remaining blockers
None for Step 2, provided hand tracking stays off and the teleop app has focus. For later steps, note the headset-relative frame, and that an empty message means the controllers are missing, not that they are at rest.
