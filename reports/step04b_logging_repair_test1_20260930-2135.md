# Step 04B: logging repair, end-to-end verification, physical Test 1 rerun

Date: 2026-09-30.
- Local PC and its USB-connected Quest only. No SSH, network transport, robot, mapping or clutch.
- No uninstall, no data clearing, no signature bypass, no tracking or boundary changes, no SDK downloads.
- The headset serial is redacted everywhere; this was verified by grep.

## Build identities
| | Old diagnostic build | Repaired diagnostic build |
|---|---|---|
| Build id | `s4a-9e6f1f24dbed` | **`s4a-ff0da2692500`** |
| Source | branch `questxarm/step04a-local-space` @ `9e6f1f2` | branch `questxarm/step04b-logging-repair` @ `ff0da26` (child of `9e6f1f2`) |
| APK | `data/step04a/org.questxarm.reader-debug-s4a-9e6f1f24dbed.apk` | `data/step04b/org.questxarm.reader-debug-s4a-ff0da2692500.apk` |
| SHA-256 | `a2a2671c2c7e494c45873e5c3e5fc7431ed5c79255bb153f8212afdda70f0841` | `57f2936ccf1756266bfca75f735eff7fbdb15fb675b7ac5dfca9fdbc61d09f76` |
| Package / versionCode | `org.questxarm.reader` / 1 | `org.questxarm.reader` / 1 (not lower) |
| Signer | `CN=Android Debug`, cert SHA-256 `ec931de599dc656f0dde3f3c50cbb79d939f2e9d6389f56d80b12bf2f25b5806` | **same certificate**: same isolated debug key; v2 signature, `apksigner` "Verifies" |
| Launchable / ABI / SDK | `com.rail.oculus.teleop.MainActivity` / arm64-v8a / min 26, target 32 | same; not testOnly |
| Meta SDK | `bbed2f20e38a5df7113630771c83cb8279e4fc26` | same (FetchContent checkout verified) |

**Original app:** `com.rail.oculus.teleop` base.apk `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341`. It was unchanged before the update, after the update and at the end, and was never replaced, stopped or modified.

**Build note:** the first, incremental rebuild reused CMake's cached configure step and embedded the **stale** id `s4a-9e6f1f24dbed` around new code. I didn't install it; it's kept as `data/step04b/DO_NOT_INSTALL_stale-buildid_incremental.apk`. I deleted the git-ignored `.cxx/` and `build/` and rebuilt clean, which embedded `s4a-ff0da2692500`.
- Logs: `data/step04b/build_repair_assembleDebug_*.log` (stale) and `build_repair_clean_assembleDebug_*.log` (used).
- The 4A APK was copied to `data/step04a/` beforehand.

## The repair (patch `patches/step04b_logging_repair_9e6f1f2..ff0da2692500.patch`; 2 files, +97 −36)
**Audit of the serialization-to-log path in `s4a-9e6f1f24dbed`:**
1. A fixed `char buf[2048]` in `qxEmitEvent` and `qxEmitPose`.
2. A bounded `vsnprintf` appender (`Out`) writing into that buffer.
3. `__android_log_print`'s internal 1024-byte format buffer. This is what cut every pose to 1023 bytes.

The disabled legacy `wE9ryARX` call isn't on this path. The 17-byte session-id buffer is fixed-length and safe.

**Changes:**
- `Out` now grows a `std::string`, so no fixed buffer remains on the path.
- All records go through one gate, `questxarm::prepare_log_message()`. A complete JSON message of at most **3000 UTF-8 bytes** (excluding the NUL) is passed unchanged. Anything larger is never emitted, even in part; instead a small, intact overflow record is emitted: `{"v":1,"type":"overflow","build","sid","seq","attempted_bytes","cap_bytes":3000}`.
- The final call is `__android_log_write(ANDROID_LOG_INFO, "QuestXarmPoseV1", msg.c_str())` for pose, event, overflow and probe records alike.
- **Size overhead:** logd stores priority (1 B) + tag (15 B + NUL) + message + NUL, i.e. at most 3018 bytes. The device reports `max payload is 4068 B`. The 3000-byte cap is an application limit, not an asserted Android maximum.
- An opt-in, finite, NON-POSE probe self-test runs once per launch, only when `debug.questxarm.logprobe=1`.
- **Unchanged:** schema (v1), all fields, numeric precision, pose frames, action bindings, tracking flags, query-time conversion, rendering and frame rate.

## Host-side changes (local)
- `quest_pose_v1.py`: decodes `OverflowRecord` and `ProbeRecord`, and provides `expected_probe()` (a byte-exact mirror of `make_probe`). Cap 3000. Truncated or invalid JSON is still rejected; nothing is repaired.
- `quest_pose_v1_record.py`:
  - Stream freshness (any line) is kept separate from input freshness (accepted pose records only).
  - An overflow record **invalidates current input** (status `OVERFLOW`) without refreshing input freshness.
  - Probes never touch input or readiness.
  - `--expect-build` drops and counts other builds' records.
  - Opt-in `--pre-attach` uses a tag-only logcat started before launch (see the probe results below).
- `quest_input_monitor.py`: the opt-in `initial_since` / `resume_from_last_seen` from the stopped 4B attempt. Defaults are unchanged.
- **Tests:** 59/59 pass (52 previous + 7 new). The new ones cover:
  - Full pose records above the old 1023-byte cutoff stay complete, trailing `in_r` fields included.
  - Probes at exactly 1023, 1024, 1500 and 3000 bytes (the cap).
  - UTF-8 byte versus character accounting.
  - Over-cap inputs produce only an intact overflow record.
  - Overflow clears input in the recorder without refreshing freshness.
  - Probes never become input.
  - Events keep the stream fresh but not the input.

  Two pre-repair assertions that expected `overflow` to be a decode error were updated to the new defined behaviour. **These host tests do not verify Android delivery.**

## Update result
`adb -s [REDACTED] install -r data/step04b/org.questxarm.reader-debug-s4a-ff0da2692500.apk` (no `-t`) printed `Success`. The installed base.apk SHA-256 is `57f2936c…9f76`, identical to the artifact. The diagnostic app was force-stopped first.

## End-to-end logging verification (on device; synthetic probes, saved separately)
**Attempt 1 (`probe_selftest_20260930-*`, pid-filtered reader):** the probes and startup events (seq 1–41) were **not received**.
- That reader only attaches after the app's pid exists, about 1 s after launch. By then logd had already discarded those entries: the main ring buffer is 256 KiB, and the app was the chattiest UID at about 80 KB/s.
- All 325 pose records that were received were intact (1118–1121 B).
- This was a host acquisition-timing problem, not truncation.

**Attempt 2 (`probe_selftest2_preattach_*`, reader attached before launch):**
| Probe | Target | Received | Byte-exact vs expected (incl. `"end":"QXEND-n"`) |
|---|---|---|---|
| 1 | 1023 | 1023 | yes |
| 2 | 1024 | 1024 | yes |
| 3 | 1500 | 1500 | yes |
| 4 | 1500 (UTF-8 `é` pad, 831 chars) | 1500 | yes |
| 5 | 3000 (= cap) | 3000 | yes |
| 6 | 3001 | overflow record only (`attempted_bytes` 3001, `cap_bytes` 3000, seq 6) | intact, no partial probe |
| 7 | 3500 | overflow record only (`attempted_bytes` 3500, seq 7) | intact, no partial probe |

- The startup events were all received (session_begin, IDLE → READY → SYNCHRONIZED → VISIBLE → FOCUSED).
- 0 parse errors. Probes were counted separately from the 348 pose records and never set input.
- 1 sequence gap (one pose record, seq 142).
- `debug.questxarm.logprobe` was set to `1` only for the probe launches and then set to `0`. The shell can't unset a property; it was empty before, and `0` disables the probes. It's a transient property that clears on reboot.

## Live smoke test (`smoke30_20260930-211738`, about 61 s, new build only)
| Gate item | Result |
|---|---|
| Pose records larger than 1023 B arriving intact | **3847 / 3847** (1117 / 1220 / 1286 B min/median/max) |
| Complete schema-valid JSON incl. trailing `in_r` | all 3847 |
| Truncation / JSON-decode errors | **0 / 0** |
| Overflow records | 0 |
| Validity / tracking / activity fields decode | yes. head: 3847 tracked (flags 15). grips: tracked, missing (inactive, flags 0), invalid and valid_untracked all observed. |
| Missing/inactive input stays unavailable | yes: 1806 records with the pose action inactive have every input `null` (never 0/false) |
| Probes | 0 (probes were off) |
| Source-sequence gaps (separate from parsing) | 6, each a single missing record |

**Gate: PASSED.** The logging return value is not used as evidence; only received payloads are.

## Physical Test 1 (new build only; `data/step04b/test1_rerun/`)
### Attempt A (`t1_buttons_…-211955`, 73 s): your own sequence
You said "not sure if I did what you wanted". You later confirmed that you moved R and L separately, squeezed the trigger and grip, and pressed A and B, and asked for a clearer scripted rerun.
- **Controllers active:** only 27–51 s (pose actions inactive and all inputs null otherwise).
- **Inputs:** trigger and grip 0.0 → 1.0. A pressed 9 times and B 6 times, each released. X/Y were also pressed.
- **Independence:** partial. "Mostly right" windows at 37–39 s and 45–47 s; "mostly left" at 42–44 s. No left/right identical records.

### Attempt B (`t1b_scripted_…-212943`, 81 s): scripted; your report: "done"
Pre-agreed script: wake both; right only; left only; right trigger ×3; right grip ×3; A ×3; B ×3.

**Logging:**
- 5110 pose records at 66.2 Hz received.
- 1117 / 1281 / 1287 B min/median/max; max record 1287 B.
- **0 truncation, 0 parse errors, 0 overflow.**
- Source-sequence gaps: 2 single records (seq 285 and 1970).
- Host pose inter-arrival: median 12.4 ms, p95 45.2 ms, max 48.4 ms.
- `t_query` step in the XrTime domain: median 13.887 ms (about 72 Hz), max 66.7 ms; strictly increasing.

| Observation | Recorded evidence |
|---|---|
| Controllers active | from about 22 s (both pose actions become active together). Inactive at 0–22 s, around 61 s and 69–77 s, each time aligned with `interaction_profile_changed` events. |
| **Right only** (26–35 s) | right path 0.06–0.71 m/s and up to 55° per second, **left 0.000 m / 0.0°** throughout. **The right pose follows the right controller.** |
| **Left only** (scripted step 3) | **not present where scripted**: the left was motionless from 26 s to 60 s. Brief left-only motion appears later at 64 s (0.021 m, 11°) and 71 s (0.099 m, 15°, right 0.002 m). That is weak evidence the left pose follows the left controller; together with attempt A's 42–44 s window it is suggestive, **not conclusive**. |
| Right trigger | 0.000–1.000. Above 0.5 at 36.5, 37.1, 39.0 and 39.9 s (4 squeezes). |
| Right grip (squeeze) | 0.000–1.000. Above 0.5 at 42.0, 42.8 and 44.1 s (3 squeezes). |
| A | presses at 48.4, 50.1 and 51.7 s, each released (3) |
| B | presses at 53.6, 55.2 and 56.9 s, each released (3) |
| Left inputs | trigger and squeeze 0.000 the whole time; X/Y/menu never pressed (as scripted) |
| Activity / tracking | head: 5110 tracked. grip_l: 2909 tracked, 2201 missing. grip_r: 2866 tracked, **43 valid-but-not-tracked**, 2201 missing. |
| Real tracking-flag observation | 62.18–62.88 s: the right grip had flags **7** (position and orientation VALID, **orientation tracked, position NOT tracked**) right after the controllers re-activated. Position and orientation tracking were reported independently. |
| Focus | FOCUSED for 5109/5110 records (the first was READY) |
| Session / reference space | one sid `26de2e3054afc7d5`. One `local_space_change_pending` at session start, before the first pose, so all poses are in the same LOCAL epoch. No mid-session LOCAL change. |
| Identical L/R poses | 0 |

**Manual confirmation:** you performed the scripted run and reported "done". For attempt A you confirmed "moved R and L separately, trigger/grip squeezes, pressed A and B". The recordings back up the right-controller, trigger, grip, A and B steps. The left-only step is not clearly visible in attempt B.

## Not yet tested (not passed)
- LOCAL-space stability with stationary controllers and a moving head (Test 2).
- Focus loss (Test 3).
- Tracking-loss occlusion (Test 4); only the incidental flag-7 interval above was seen.
- Recentre (Test 5).
- New-app stop/restart behaviour (Test 6).

## Remaining issues and open points
1. **Whole-record drops on the Android log path.** 1–6 single-record sequence gaps per minute at the current per-frame logging rate (about 66–72 records/s × ~1.2 KB). These are not truncation: no partial record was ever received after the repair. The writer-side or logd cause is not established. Per instructions the rate was not changed.
2. **Startup capture depends on attaching the reader before launch** (the opt-in `--pre-attach` mode). The pid-filtered reader loses the first ~1 s at this data rate.
3. **Pose-action inactivity** comes in blocks, together with `interaction_profile_changed` events, while the app stays FOCUSED. It could be controller idle/sleep, controllers set down, or something else; the cause is not established.
4. **Left-controller independence** is only weakly shown (see above). A short left-only check could settle it.
5. **Timing:** `t_query_ns` is the XrTime passed to `xrLocateSpace`: `ToXrTime(in.PredictedDisplayTime)`, where the framework stores the predicted display time as `double` seconds. The raw `frameState.predictedDisplayTime` is not logged, so the rounding difference is **unmeasured**. It is not a sensor-capture time and was never subtracted from host time.
6. **Logging load:** the observed arrival rate is about 63–68 lines/s against about 72 Hz frames (the XrTime step median is 13.887 ms, max 66.7 ms, which suggests occasional missed frames). No stutter was reported. Logging overhead and data age are not inferred from arrival rate.

## State at stop
- `org.questxarm.reader` (`57f2936c…9f76`) is installed and force-stopped.
- `com.rail.oculus.teleop` (`ea988329…6341`) is installed and unchanged.
- No recorder or logcat client processes are left. `debug.questxarm.logprobe` = `0`.

**Stopped after Test 1.**
