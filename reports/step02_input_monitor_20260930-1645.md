# Step 02: Controller app and local input monitor

Date: 2026-09-30. Everything was done locally, over USB ADB only. There was no SSH, robot code, network listener, port forwarding or wireless ADB.
The headset serial is redacted everywhere below. It does not appear in any saved file (checked with grep).

## A. Upstream checkout

- Repo: https://github.com/agilexrobotics/QuestArmTeleop.git, branch `ros2`.
- Checkout: shallow (`--depth 1`), blobless and sparse, at `external/QuestArmTeleop`.
- Sparse set: `/README.md`, `/src/oculus_reader/scripts/`, `/src/oculus_reader/APK/teleop-debug.apk`.
  The first sparse pass also fetched `alvr_client_android.apk`, config and launch files. I narrowed the set afterwards, so those files are no longer checked out.
- Commit: **145b80360cf2c5f2e63179086d0aab2cb563826c** (2026-08-18, "Revise connection instructions and renumber sections").
- I did not run any upstream install script, conda/pip step, colcon build or ROS step.

### Upstream APK: `src/oculus_reader/APK/teleop-debug.apk`

- It is a real APK and not an LFS pointer: 4,865,715 bytes, starts with `PK\x03\x04`, `zipfile.testzip()` reports no errors, and `file` reports "Android package (APK), with APK Signing Block".
- SHA-256: `97b49f94682a732e14d131d50bcc7885e183d73b977e19a1752c28502519cd4b`
- Metadata from aapt2 and apksigner:
  - package `com.rail.oculus.teleop`, versionCode 1, versionName 1.0, targetSdk 25.
  - The app is debuggable.
  - Its launchable activity is `com.rail.oculus.teleop.MainActivity`. This matches the reader's `am start` component.
  - Native code is arm64-v8a only.
  - It is V2-signed with `CN=Android Debug`. The certificate SHA-256 is `fac61745…033b9c`.
- The log tag `wE9ryARX` is inside `lib/arm64-v8a/libvrinputstandard.so`, not the dex. It matches the tag in the reader (`self.tag`).

## B. Installation and launch

- **The package was already installed** before this step. It was first installed on 2026-03-11 and last updated on 2026-04-07. It is **not the upstream build**:
  - The installed base.apk SHA-256 is `ea9883294ae6465f09d581d6c50f2939fff00890e4b3ead8316caae83df26341`.
  - Its signer is `CN=ovrbuild, CN=jedrzej, OU=Jedrzejs-MacBook-Air.local`, with certificate SHA-256 `3714f848…d713b683`.
  - It has the same package, versionCode 1, MainActivity and log tag. Its `classes.dex`, `libvrapi.so`, `libvrinputstandard.so`, manifest and `assets/handProfile_meter.json` differ from upstream.
- Because the signer is different, installing the upstream APK would have required uninstalling the existing app first. **You chose to keep the existing build**, so I installed, replaced, uninstalled and cleared nothing.
- Launch: `adb -s [REDACTED] shell am start -W -n com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity` returned `Status: ok` (cold start).
  During the tests I also used `am force-stop` and `am start` on this app to test exit and restart.

## C. Monitor

- Files: `scripts/quest_input_monitor.py` and `tests/test_quest_input_monitor.py`. Both use the standard library only.
- Source: one `adb -s SERIAL logcat --pid=<app pid> -T <device epoch at start> -v epoch -v usec -v threadtime -s wE9ryARX:V` subprocess.
  - I checked all of these options against `logcat --help` on the headset.
  - Lines older than the session start are also dropped, so earlier buffered messages don't count.
- Verified protocol (from source and live output): `l:<16 floats> |r:<16 floats> &<buttons>`.
  - Each matrix is row-major 4x4, with the translation in column 4 and a bottom row of `0 0 0 1`.
  - The buttons use `L`/`R` markers, boolean keys and analog items (`rightTrig 0.000000`).
- Validation:
  - A message is rejected and counted as a parse error if it has anything other than exactly one `&`, an unknown or duplicate controller key, a value count other than 16, a non-numeric or non-finite value, or a bad analog value.
  - A missing controller is recorded as `null`, never as zeros or an identity pose.
  - The homogeneous check is flagged but does not reject the message. It fails if bottom-row error > 1e-4, max |RᵀR−I| > 1e-3, or |det R − 1| > 1e-3. These tolerances are set by the app's roughly 6-significant-digit printing.
- Freshness:
  - Counters and receive time are updated only in `record_message`, using `time.monotonic_ns()` when the line is read.
  - Status is computed by the print loop from its own clock every 0.5 s, so STALE shows up even while the reader is blocked.
  - DISCONNECTED means the app process is gone or the device is unavailable. A supervisor thread checks `pidof` every 1 s and restarts logcat when the app pid changes.
  - The 500 ms threshold is diagnostic only (`--stale-ms`) and is not a safety timeout.
- The device logcat time is saved as `device_log_time`, the device-clock write time. It is never subtracted from host time.
- **Tracking validity:** the protocol has no tracking-valid flag, so it is reported as `null`/"unknown" everywhere.
- Cleanup: only the monitor's own adb client subprocesses are terminated, never the adb server. After Ctrl+C, app exit and USB loss there were no leftover `adb logcat` processes, and the adb server kept pid 430514 throughout.

## D. Automated tests (no device)

`python3 -m unittest tests/test_quest_input_monitor.py`: **15 tests, all passed.** They cover:
- a valid message and a real line format
- one controller only, and no controllers
- malformed and non-finite matrices
- analog and boolean button parsing and bad analog values
- a malformed line being counted without crashing
- STALE with no new messages, and polling not refreshing freshness
- DISCONNECTED taking priority over other states
- identical poses at about 72 Hz staying LIVE with tracking unknown
- gap statistics and JSONL fields

## E. Live runs (data in `data/step02/`)

Every run used this invocation, with a different output name each time:

```
python3 scripts/quest_input_monitor.py --duration <N> --output data/step02/<run>.jsonl --summary data/step02/<run>.summary.json
```

`--serial` was omitted: the monitor picks the device automatically only when exactly one is authorized, otherwise it requires `--serial [REDACTED]`.

| Run | Purpose | Valid msgs | Rate | Gap median / p95 / max | Parse errors | Status |
|---|---|---|---|---|---|---|
| run1 (60 s) | ambient: steps **not** performed (your answer) | 4303 | 71.7 Hz | 13.9 / 15.7 / 29.5 ms | 0 | LIVE 100% |
| run1b (60 s) | normal guided run | 4299 | 71.6 Hz | **13.9 / 15.9 / 30.8 ms** | 0 | LIVE 100% |
| run1c (20 s) | button-only check (partly performed) | 1434 | 71.7 Hz | 13.9 / 15.4 / 28.6 ms | 0 | LIVE 100% |

- Homogeneous-check failures: 0 in every run.
- Gaps over 500 ms during the normal runs: 0.

### Observed fields

- Poses: `r` and `l` were present in 100% of messages in every run. Neither was ever reported missing, including while the left controller was supposed to be set down.
- Left pose equalled right pose exactly in 34% of run1 messages and 10% of run1b messages, in stretches of up to about 5.6 s.
- Button keys seen: A, B, RThU, RJ, RG, RTr, X, Y, LThU, LJ, LG, LTr, rightJS, rightTrig, rightGrip, leftJS, leftTrig, leftGrip.
- **All button and analog values were 0/False in every run.** You said you pressed the trigger and grip in run1b but not A/B, and that you did the run1c check only partly. So "no input reported" is **not a confirmed app defect**. It is still unresolved.
- Right-controller motion roughly followed the run1b script. The smallest spread was about 0.01 source units in the 40–45 s "rest" window, compared with 0.2–0.8 while moving. No two successive right poses were exactly identical.

### Interruption tests

- **run2 (90 s): app backgrounded** (Meta button, so Library/Home was in front; the app process stayed alive under the same pid).
  - The stream stopped at 27.2 s.
  - Status went LIVE → STALE within 1 s. The age kept rising to 62.9 s while the read was blocked. It was correctly **not** DISCONNECTED, because the process and logcat were still alive.
  - The USB unplug did not happen in this run: there were no transport events.
- **run3 (75 s): app exit and USB loss.**
  - The `am force-stop` at 10.0 s produced DISCONNECTED ("app not running") at 10.3 s.
  - The USB was unplugged at about 17 s, earlier than planned. The status stayed DISCONNECTED with the reason "device unavailable: device not found", which is what should happen.
  - My relaunch at 20 s failed because the device was absent, so the stream did not resume.
  - At 27.5 s the device briefly reported the "check for a confirmation dialog" state, which means it was unauthorized while reconnecting. It was back as `device` by 35.5 s.
  - There was no hang: the run exited 0 on time and left no adb clients behind.
- **run3b (25 s): app restart recovery.**
  - Force-stop at 5 s gave DISCONNECTED at 5.2 s.
  - The relaunch at 10 s gave a new pid, and logcat reattached at 10.4 s. The status went STALE → LIVE by 11.0 s.
  - Recorded gap: 5897 ms. Trailing silence at the end: 8 ms.

## Results that need your confirmation (not automated)

- Physical controller exercise: **not confirmed as passed.** Run1 was ambient, and in run1b and run1c the buttons were only partly pressed. I have no confirmation that the poses correspond to the physical controllers rather than the head or hands.
- The USB unplug and re-plug was done by hand. The monitor's behaviour during it is automated and logged above.

## Remaining blockers and open questions

1. **Controller input not yet shown in the stream.** Every button and analog value was 0/False. This needs a fully performed button test, with the trigger held, the grip held and A/B pressed. If the values still stay at 0, the installed non-upstream build is the likely cause. Replacing it with the upstream APK would require your approval to uninstall the existing app.
2. **Left pose looks suspicious.** It sometimes exactly equals the right pose and is never reported missing. The app may be emitting a fallback or hand pose. The upstream README says to turn off hand tracking and gesture recognition, and we haven't checked that on the headset.
3. **No tracking-valid flag.** The protocol has none, so tracking validity stays unknown. Fresh messages show only that the app is logging.
4. **Pose and button values stop when the app is backgrounded.** The logcat stream stops while the process stays alive, which gives STALE rather than DISCONNECTED. That behaviour is correct; the app just needs to stay in the foreground.
