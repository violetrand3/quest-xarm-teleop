# Step 04B: STOPPED at Test 1 (pose records truncated by Android logging)

Date: 2026-09-30.
- No lab or robot involvement.
- No uninstall and no data clearing.
- The old app was not modified.
- The headset serial is redacted.

## A. Install and launch (physical; completed)
- **APK verified before install** against the Step 4A report, all matching:
  - `data/step04a/org.questxarm.reader-debug-s4a-9e6f1f24dbed.apk`, SHA-256 `a2a2671c2c7e494c45873e5c3e5fc7431ed5c79255bb153f8212afdda70f0841`, byte-identical to the build output.
  - Package `org.questxarm.reader`, label "Quest xArm Input Diagnostic".
  - Launchable `com.rail.oculus.teleop.MainActivity`.
  - Signer `CN=Android Debug`, cert SHA-256 `ec931de599dc656f0dde3f3c50cbb79d939f2e9d6389f56d80b12bf2f25b5806`.
  - Embedded build id `s4a-9e6f1f24dbed`.
  - Not `testOnly`, so no `-t` was used.
- **Before install:** `org.questxarm.reader` was absent. `com.rail.oculus.teleop` base.apk was `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341` (lastUpdateTime 2026-04-07), and the app was not running.
- **Install:** `adb -s [REDACTED] install <apk>` (no flags) printed `Success`. The installed base.apk SHA-256 is `a2a2671c…0841`, identical. The old app's APK after install is still `ea988329…6341`, **unchanged**.
- **Launch:** `am start -W -n org.questxarm.reader/com.rail.oculus.teleop.MainActivity` returned `Status: ok`, COLD, TotalTime 1030 ms. The diagnostic app was in front.
- **Telemetry identity, from intact event records:**
  - schema `v: 1`, build `s4a-9e6f1f24dbed`, session id `ef6c0d0c4d3653e2`, `space: "LOCAL"`.
  - Events: `session_begin`, `other_space_change_pending` ×2 (non-LOCAL spaces, at session start), then `session_state` IDLE → READY → SYNCHRONIZED → VISIBLE → FOCUSED.

## B. Problem: every pose record truncated at 1023 bytes
Recording: `data/step04b/t1_launch_buttons_20260930-*` (`.jsonl`, `.summary.json`, `.console.txt`, `.marks.txt`). The recorder ran 23.5 s and was stopped as soon as the problem was seen.

| Metric | Value |
|---|---|
| Lines received | 1,466 (8 events + 1,458 pose lines) |
| Pose records decoded | **0**; all 1,458 are parse errors |
| Size of every pose line | **exactly 1,023 bytes** |
| Where truncation occurs | inside `in_r`, after `"thumbstick_click":null,"`. The head, `grip_l` and `grip_r` objects are present; the right-hand `trigger_touch`, `thumbrest_touch`, `a`, `b` and the closing braces are lost. |

**Cause (identified, not yet fixed):** the app logs with `__android_log_print()`, which formats into a fixed **1024-byte** buffer (1023 characters + NUL) before sending to logd. `QuestXarmTelemetry.h` assumed a ~4 KB logcat payload and allowed records up to 2048 bytes. A full record with all three poses is longer than 1023 bytes.

The Step 4A offline tests checked the formatter's own 2048-byte cap. They did not, and on a host could not, exercise the Android logging path. This is a defect in the Step 4A build.

**Detection worked as designed:** every truncated line was rejected as malformed. None was partially accepted, and no pose or button value was inferred from it. The recorder status never showed FRESH_TRACKED.

## C–D. Physical tests 1–6: NOT PERFORMED
No button, stationary-controller, focus, occlusion, recentre or restart results exist for this build. Partial pose fields in truncated lines were **not** analysed as results.

## State left on the headset
- `org.questxarm.reader` is installed. It was still running when the recorder stopped; it has not been force-stopped or uninstalled.
- `com.rail.oculus.teleop` is unchanged and not running.

## Code changes in this step (local only)
- `scripts/quest_input_monitor.py`: opt-in `AdbLogcatSource(initial_since=..., resume_from_last_seen=...)`, so the first lines of a newly launched process aren't lost. Defaults are unchanged, and all 52 existing tests pass. The previous version is backed up as `data/step04a/quest_input_monitor.pre4b.py.bak`.
- `scripts/quest_pose_v1_record.py` (new): records every line raw, with no downsampling. It applies the right-controller status rules (DISCONNECTED / STALE / UNFOCUSED / MISSING / INVALID / VALID_UNTRACKED / FRESH_TRACKED).

## Options needing your decision (no rebuild done)
1. **Fix and rebuild as a new, separately identified build.** Log with `__android_log_write()`, which has no 1024-byte formatting buffer and is limited only by the logd payload (~4 KB). Lower the formatter cap to a value with margin below that, and keep the explicit `overflow` record. Optionally also shorten the keys. Then re-run Step 4B from Test 1 with the new build only; nothing from `s4a-9e6f1f24dbed` would be mixed in.
2. **Shrink the record below 1023 bytes** (compact keys, or fewer fields such as dropping thumbstick touch). This keeps `__android_log_print`, but leaves less margin.
3. Stop here.

Either fix changes the APK (new hash, new build id). The new APK would need to be installed over `org.questxarm.reader` with `adb install -r`, which **replaces** this diagnostic package. That is outside the current instructions and needs your approval. The old `com.rail.oculus.teleop` would not be touched.
