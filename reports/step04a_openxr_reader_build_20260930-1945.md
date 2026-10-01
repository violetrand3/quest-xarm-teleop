# Step 04A: LOCAL-space OpenXR diagnostic reader (build only)

Date: 2026-09-30.
- There was no SSH or change on linux2, and no robot, gripper, ROS or SDK use.
- No APK was installed, uninstalled or launched.
- There was no app-data clearing, no wireless ADB and no new network service.
- The headset serial is redacted, and no signing key material is included.

Full details are in `docs/step04a_openxr_reader.md`.

## 1. Source behaviour verified by inspection
**App:** `jborbik/oculus_reader` @ `9689484d319c4798e54d59509b192436647b7427`, checked out at `external/oculus_reader_openxr`. The `external/QuestArmTeleop` checkout was left untouched. Large APKs were not fetched: the repository's APK is a 133-byte LFS pointer.

**Meta SDK:** `Meta-OpenXR-SDK` `v85` = `bbed2f20e38a5df7113630771c83cb8279e4fc26`. This was confirmed both by `git ls-remote` and in the `FetchContent` checkout used by the build.

| Claim | Verified |
|---|---|
| `SessionInit` sets `CurrentSpace = LocalSpace` | Yes (`main.cpp:786` upstream). The framework first sets `StageSpace` if supported, and `SessionInit` runs afterwards and overrides it. |
| The output multiplies by `inverse(headPoseMatrix)` | Yes (`main.cpp:1087-1096`): `Multiply(head.Inverted(), remotePose)`, so the output is head-relative. |
| The Meta SDK is pinned | `CMakeLists.txt`: `GIT_TAG v85`, which resolves to `bbed2f20…`. |
| Reference space of the controller poses | `xrLocateSpace(gripSpace, CurrentSpace (=LOCAL), t)`. The head uses `xrLocateSpace(VIEW, CurrentSpace, t)`. |
| Grip or aim | `in.*RemotePose` is **grip** (`/user/hand/*/input/grip/pose`). `*RemotePointPose` is aim. |
| Query time | `ToXrTime(in.PredictedDisplayTime)`: the frame's predicted display time, via double seconds. The head and all controllers use the same value. |
| locationFlags actually exposed | Only `*RemoteTracked = poseActionActive && POSITION_VALID`. Orientation-valid and both TRACKED bits are dropped, the head's flags are ignored, and an inactive pose is replaced with identity. |
| Action activity | `ActionPoseIsActive` for poses. Button and analog `isActive` is available through `GetActionState*` but unused upstream. |
| Session focus and state | `Focused` is true on FOCUSED and false on VISIBLE. The state goes to a virtual `SessionStateChanged(state)` without its time. |
| Reference-space change | `REFERENCE_SPACE_CHANGE_PENDING` is **only logged** by the framework. It is reachable through the virtual `AppHandleEvent()`, which sees every event first. |
| Extra finding | In upstream `Remotes.h` the labels are crossed: `"l:"` + left matrix is printed when `RightRemoteTracked` is set, and vice versa. |

The upstream author's name matches the signer CN of the APK installed on the headset. **This is not evidence that the checkout is the source of that APK, and I make no such claim.**

## Changes
- **Branch:** `questxarm/step04a-local-space`, with commits `491b3e0` and `9e6f1f2`.
- **Patches:** `patches/step04a_openxr_reader_full_9689484..9e6f1f24dbed.patch` (sha256 `a3db2224…58cc0`); the first commit alone is in `patches/step04a_openxr_reader_9689484..491b3e036995.patch`.
- **Files:** 6 files, +394 −2.
  - `applicationId org.questxarm.reader`, label "Quest xArm Input Diagnostic". The namespace and Java class stay the same, so the manifest and library references resolve.
  - A new `QuestXarmTelemetry.h`.
  - In `main.cpp`:
    - `qxEmitPose()` runs its own `xrLocateSpace` calls against **LocalSpace**, at **one** query time, for the head and both **grips**, with the raw flags.
    - The pose-action `isActive` and per-button `isActive` are recorded; an inactive action is written as `null`.
    - Session state/focus, `local_space_change_pending` (with `changeTime`, `poseValid` and `poseInPreviousSpace`), `events_lost`, and session begin/end are all recorded.
    - The legacy `wE9ryARX` output is compiled out, and its meaning is unchanged.
  - The build embeds its source revision.
  - The Gradle 8.5 distribution's SHA-256 is pinned.
- **Unchanged:** the rendering camera and frame loop. Controller haptics on A/B/X/Y are kept from upstream. The manifest still requests `HAND_TRACKING` as upstream did; hand tracking itself must remain OFF on the headset.
- **What the published data does not do:** there is no `inverse(head)`, no left-controller compensation, no robot axis mapping, no scaling, no smoothing and no clutch.

## 2. Offline tests (synthetic; they do not establish Quest 3S or physical behaviour)
`python3 -m unittest tests/test_quest_pose_v1.py tests/test_quest_input_monitor.py tests/test_quest_transport.py` ran **52 tests, all OK**.
- **13 new tests** (`tests/test_quest_pose_v1.py`):
  - Matrix layout and transform direction.
  - A stationary controller with a moving head: the LOCAL pose stays fixed while the head-relative pose changes.
  - Independent left and right controllers.
  - Trigger 0.75, squeeze 0.3, A=true and B=false are preserved.
  - Missing poses stay missing, and absent input fields are rejected.
  - invalid, valid-untracked and tracked are distinguished.
  - Focus loss.
  - Flag and field consistency.
  - Malformed, NaN, `1e999`, truncated, over-size, wrong-space, non-unit-quaternion and overflow records are rejected.
  - Session change, duplicate or out-of-order `seq`, LOCAL-change epochs by `changeTime`, and `events_lost`.
  - A **round trip through the real `QuestXarmTelemetry.h`**, compiled for the host with `g++ -Wall -Wextra -Werror`.
- **39 regression tests** (Step 2 monitor + Step 3 transport) all pass. `quest_input_monitor.py` is unchanged since Step 3 (`8ebd9d7e…`).

## 3. APK build: SUCCESS
| Item | Value |
|---|---|
| Built from | branch HEAD `9e6f1f24dbed` (clean). The embedded build id is `s4a-9e6f1f24dbed`. |
| Build log | `data/step04a/build_assembleDebug_20260930-193649.log`: `BUILD SUCCESSFUL in 3m 24s` |
| APK | `external/oculus_reader_openxr/app/build/outputs/apk/debug/OculusTeleop-debug.apk`, copied to `data/step04a/org.questxarm.reader-debug-s4a-9e6f1f24dbed.apk` |
| SHA-256 | `a2a2671c2c7e494c45873e5c3e5fc7431ed5c79255bb153f8212afdda70f0841` (10,719,152 bytes) |
| Package | `org.questxarm.reader`, versionCode 1, label "Quest xArm Input Diagnostic", debuggable |
| Launchable component | `org.questxarm.reader/com.rail.oculus.teleop.MainActivity` |
| SDK | minSdk 26, targetSdk 32, compileSdk 32 |
| ABI | `arm64-v8a` only. Libraries: liboculusteleop, libopenxr_loader, libktx, libc++_shared. |
| Tags in the native library | `QuestXarmPoseV1` present, `wE9ryARX` **absent** |
| Signature | `apksigner verify`: **Verifies**, v2 scheme, 1 signer, `CN=Android Debug`. Certificate SHA-256 `ec931de599dc656f0dde3f3c50cbb79d939f2e9d6389f56d80b12bf2f25b5806`. The debug key is in the isolated `/mnt/data/quest-xarm-build/android-user-home` and is not shared. |

**Toolchain (isolated):**
- JDK 17.0.17 (existing `~/.gradle/jdks` Adoptium, read only).
- Gradle 8.5 (checksum-pinned) and AGP 8.1.4.
- NDK 27.0.12077973, SDK CMake 3.22.1, platform android-32 and build-tools 33.0.1, all in `/mnt/data/quest-xarm-build/android-sdk`. **You accepted the Android licenses interactively**, and **you accepted the Oculus SDK License** for this build.
- `GRADLE_USER_HOME=/mnt/data/quest-xarm-build/gradle-home`.
- `openxr_loader_for_android` 1.1.53.
- Not touched: `~/.gradle`, `~/.android`, `/mnt/data/Android/Sdk` and adb. Nothing was written to the root disk (97% full).
- The checked-in `gradle-wrapper.jar` matches Gradle's official 7.1–7.3.3 wrapper jar.

**Headset (read-only check):** `org.questxarm.reader` is **not installed**. The existing `com.rail.oculus.teleop` base.apk is unchanged (sha256 `ea988329…6341`, the same as in Step 2).

## 4. Physical headset behaviour: NOT YET TESTED
None of the following has been observed on the Quest 3S:
- that the app launches;
- that the OpenXR runtime accepts the bindings (touch_plus, which is expected for 3S controllers);
- the actual flag values;
- LOCAL-space stability;
- recentre (reference-space change) events;
- focus behaviour;
- the record rate or logcat truncation in practice.

## Remaining points for the next step
- Installing this APK needs your explicit approval. It installs as a separate package and doesn't touch `com.rail.oculus.teleop`.
- The query time passes through the framework's `double` seconds (`ToXrTime(in.PredictedDisplayTime)`). All three locates use the identical value, but it may differ by rounding from the raw `frameState.predictedDisplayTime`.
- A record is emitted per rendered frame, roughly the display rate. This is higher than the old app's rate, and logcat load will need checking on the device.

**Stopped after Step 4A.**
