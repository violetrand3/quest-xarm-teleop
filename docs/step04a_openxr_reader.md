# Step 4A: OpenXR LOCAL-space diagnostic reader (`org.questxarm.reader`)

**DIAGNOSTIC ONLY.** The app has no robot-control functionality. It is a separate application that installs *alongside* `com.rail.oculus.teleop` and never replaces it. As of Step 4A it has **not been installed or run on the headset**.

## Source
| Item | Value |
|---|---|
| App source | https://github.com/jborbik/oculus_reader.git @ `9689484d319c4798e54d59509b192436647b7427` (main, 2026-05-17) |
| Local checkout | `external/oculus_reader_openxr` (the Agilex checkout `external/QuestArmTeleop` is untouched) |
| Work branch | `questxarm/step04a-local-space`: `491b3e0` (telemetry and app identity) + `9e6f1f2` (Gradle distribution checksum) |
| Patch | `patches/step04a_openxr_reader_9689484..491b3e036995.patch` (the second commit adds only `distributionSha256Sum`) |
| Meta OpenXR SDK | `meta-quest/Meta-OpenXR-SDK` tag `v85` = `bbed2f20e38a5df7113630771c83cb8279e4fc26`, fetched by CMake `FetchContent` (Oculus SDK License, accepted by the user for this build) |
| OpenXR loader | Maven `org.khronos.openxr:openxr_loader_for_android:1.1.53` |

The upstream author name matches the signer CN of the APK already installed on the headset. That is **not** evidence that this checkout is the source of that APK, and no such claim is made.

## Changes (see patch)
- `build.gradle`: `applicationId "org.questxarm.reader"`. The `namespace` and Java package stay `com.rail.oculus.teleop`, so the manifest's `com.rail.oculus.teleop.MainActivity` and `android.app.lib_name=oculusteleop` still resolve. Launch component: `org.questxarm.reader/com.rail.oculus.teleop.MainActivity`.
- `res/values/strings.xml`: the label is "Quest xArm Input Diagnostic".
- `CMakeLists.txt`: the build embeds `QUESTXARM_BUILD_ID="s4a-<git rev>[-dirty]"`.
- `src/QuestXarmTelemetry.h` (new): a bounded (2048 B) JSON formatter. It has no OpenXR dependency, so it can be host-tested.
- `src/main.cpp`:
  - Adds `qxEmitPose()` once per frame from `Update()`, plus an `AppHandleEvent()` override and session begin/end events.
  - The legacy head-relative `wE9ryARX` output is compiled out (`QUESTXARM_EMIT_LEGACY_WE9RYARX=0`). Its code and meaning are unchanged if it is re-enabled.
  - Rendering, the frame loop, UI and haptics are unchanged. The upstream app vibrates the controllers when A, B, X or Y is pressed; that behaviour is kept.
- `gradle-wrapper.properties`: pins the official SHA-256 of `gradle-8.5-bin.zip`. The checked-in `gradle-wrapper.jar` (`33ad4583…6a89`) matches Gradle's official wrapper jar for 7.1–7.3.3.

## Telemetry: logcat tag `QuestXarmPoseV1`, schema `v: 1`
One JSON object per log line. There are three record types.

**`pose`**, once per rendered frame:
- `build`, `sid`: `sid` is a random 64-bit id per app process.
- `seq`: increases across pose and event records.
- `space: "LOCAL"`.
- `t_query_ns`: the XrTime passed to every `xrLocateSpace` call in this record, taken from the frame's predicted display time. `t_query_kind` is `"predicted_display_time"`. **This is not a sensor-capture time.**
- `session_state`, `focused`.
- `local_epoch`, `local_change_time_ns`.
- `head`, `grip_l`, `grip_r`: each has `queried`, `result` (XrResult), `active` (pose-action `isActive`; not present for the head), `flags` (raw `XrSpaceLocationFlags`), `pos_valid`, `ori_valid`, `pos_tracked`, `ori_tracked`, `p`, `q`.
- `in_l`, `in_r`: `trigger`, `squeeze` (floats), `thumbstick` ([x, y]), `thumbstick_click`, `thumbstick_touch`, `trigger_touch`, `thumbrest_touch`, `a`/`b` (right) or `x`/`y`/`menu` (left). **An inactive action is `null`**, never 0 or false. Button values come from the frame's `xrSyncActions` and are not claimed to share `t_query_ns`.

**`event`**:
- `session_begin` and `session_end`.
- `session_state`, with `time_ns` from the event.
- `local_space_change_pending`, with `time_ns` = `changeTime`, `pose_valid` and `pose_in_previous_space`. `local_epoch` increments when this event is received.
- `other_space_change_pending`.
- `events_lost`: consumers must treat this as a reset.
- `interaction_profile_changed`.

**`overflow`**: emitted instead of a record that wouldn't fit in 2048 B. The decoder treats it as a dropped record.

### Pose conventions
- `p` = position `[x, y, z]` in **metres**. `q` = unit quaternion `[x, y, z, w]` (OpenXR `XrPosef`).
- The pose is **T_LOCAL←X**: it maps points in frame X (the view or the grip) into LOCAL.
  In the column-vector convention: `p_LOCAL = R(q)·p_X + p`.
  `quest_pose_v1.pose_matrix()` returns the 4×4 matrix with the translation in the last column.
- OpenXR axes are right-handed: +X right, +Y up, −Z forward. LOCAL is gravity-aligned (+Y up), with its origin and yaw set by the runtime at session start or on recentre.
- Controller poses are **grip** poses (`/user/hand/*/input/grip/pose`), not aim poses.
- Nothing is converted to head-relative, robot axes, scaled, smoothed or clutched. Head-relative values are computed only offline, for diagnosis: `inverse(T_LOCAL←head) · T_LOCAL←grip`.

### Validity
`p` is emitted only when `POSITION_VALID` is set, and `q` only when `ORIENTATION_VALID` is set. There is no identity fallback and no reuse of an older pose. Non-finite values are reported as `"nonfinite": true` and are never written as numbers. The decoder then classifies each pose:

| Status | Meaning |
|---|---|
| `missing` | Not queried, the locate failed, or the pose action is inactive. |
| `invalid` | Position or orientation is not VALID. |
| `valid_untracked` | Both are VALID but not both TRACKED. |
| `tracked` | Both are VALID and both are TRACKED. |

### Reference-space changes
Per the OpenXR spec, the LOCAL origin changes at `changeTime`. A pose record belongs to origin epoch = the number of `local_space_change_pending` events with `time_ns <= t_query_ns`, as computed by `StreamState.origin_epoch()`. Consumers must not join poses from different epochs into one trajectory, and must invalidate any control reference when the epoch changes.

## Upstream behaviour verified by inspection (SDK `bbed2f20`, `SampleXrFramework/Src/XrApp.cpp`)
- **Grip spaces:** `CreateActionSpace(GripPoseAction, hand)`, bound to `/user/hand/{left,right}/input/grip/pose`. Aim spaces use `aim/pose`.
- **Upstream's locate reference space:** `GetSpaceLocVel()` → `xrLocateSpace(space, CurrentSpace, time)`. The framework sets `CurrentSpace = StageSpace` if STAGE is supported. The app's `SessionInit()` (called afterwards) sets `CurrentSpace = LocalSpace`. The head is located with `xrLocateSpace(HeadSpace (VIEW), CurrentSpace, …)`.
- **Upstream's query time:** `ToXrTime(in.PredictedDisplayTime)`, the frame's predicted display time round-tripped through double seconds. The head and all controllers use the same value.
- **Upstream's "tracked" flag:** `in.*RemoteTracked = ActionPoseIsActive && (locationFlags & POSITION_VALID)`. Orientation-valid and both TRACKED bits are **dropped**, the head's flags are never checked, and an inactive pose becomes identity. The new telemetry therefore runs its own `xrLocateSpace` calls, explicitly against `LocalSpace`, and reports the raw flags.
- **Focus:** `Focused` is true on FOCUSED and false on VISIBLE. The session state is only logged. The new app reports both.
- **Reference-space changes:** `XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING` is only logged by the framework, with no data exposed. The new app captures it in `AppHandleEvent()`, which sees every event before the framework does.
- **Old output:** the legacy payload is `inverse(headPose) · remotePose` (head-relative).
  In `Remotes.h` the labels are crossed: `"l:"` + left matrix is printed when *RightRemoteTracked* is set, and `"r:"` + right matrix when *LeftRemoteTracked* is set. So a label's *presence* depends on the opposite controller's tracking state. This is recorded for Step 2's findings; it is not changed here.

## Build (isolated; nothing is written to `~/.gradle`, `~/.android` or `/mnt/data/Android/Sdk`)
Toolchain locations:
- SDK root: `/mnt/data/quest-xarm-build/android-sdk`. Licenses were accepted by the user via `sdkmanager --licenses`. Packages installed:
  - `ndk;27.0.12077973`
  - `cmake;3.22.1`
  - `platforms;android-32`
  - `build-tools;33.0.1`
- JDK 17: the existing `~/.gradle/jdks/eclipse_adoptium-17-amd64-linux.2` (17.0.17), read only.
- Gradle user home: `/mnt/data/quest-xarm-build/gradle-home`. Android user home (debug keystore): `/mnt/data/quest-xarm-build/android-user-home`.
- `app/local.properties` (git-ignored): `sdk.dir=/mnt/data/quest-xarm-build/android-sdk`.

```bash
cd ~/quest-xarm-teleop/external/oculus_reader_openxr/app
env -u ANDROID_SDK_ROOT \
  JAVA_HOME=$HOME/.gradle/jdks/eclipse_adoptium-17-amd64-linux.2 \
  ANDROID_HOME=/mnt/data/quest-xarm-build/android-sdk \
  ANDROID_USER_HOME=/mnt/data/quest-xarm-build/android-user-home \
  GRADLE_USER_HOME=/mnt/data/quest-xarm-build/gradle-home \
  ./gradlew --no-daemon --console=plain assembleDebug
```
Output: `app/build/outputs/apk/debug/app-debug.apk`. The debug signing key lives in the isolated Android user home. It is never copied into the project or reports.

## Local decoding and tests
- `scripts/quest_pose_v1.py`: a strict decoder, the pose math (column-vector convention), `StreamState` (sessions, sequencing, LOCAL epochs), and a live monitor (`--serial --duration --output`). The monitor reuses the Step 2 `AdbLogcatSource`, filtered to package `org.questxarm.reader` and tag `QuestXarmPoseV1`. It is not used until the app is installed.
- `tests/test_quest_pose_v1.py`: synthetic tests, plus a host-compiled round trip through the real `QuestXarmTelemetry.h` (`tests/fixtures/qx_format_harness.cpp`).
- The old `quest_input_monitor.py` / `wE9ryARX` path and the Step 3 transport are unchanged. Their tests still pass.

Run the tests with:
```bash
cd ~/quest-xarm-teleop && python3 -m unittest tests/test_quest_pose_v1.py tests/test_quest_input_monitor.py tests/test_quest_transport.py
```

## Update: Step 4B logging repair (build `s4a-ff0da2692500`)
Build `s4a-9e6f1f24dbed` logged through `__android_log_print()`, which silently truncated every pose record to 1023 bytes. That build is superseded.

Since `ff0da26` (branch `questxarm/step04b-logging-repair`, patch `patches/step04b_logging_repair_9e6f1f2..ff0da2692500.patch`):
- Records are serialized into `std::string`.
- One gate, `prepare_log_message()`, enforces a **3000-byte** application cap (UTF-8, excluding NUL). Over-cap records are replaced by a complete `overflow` record with `attempted_bytes` and `cap_bytes`.
- Logging uses `__android_log_write()`.
- An opt-in NON-POSE probe self-test runs when `debug.questxarm.logprobe=1`.

For full startup capture, record with `quest_pose_v1_record.py --pre-attach --expect-build <id>`.

Details are in `reports/step04b_logging_repair_test1_20260930-2135.md`.
