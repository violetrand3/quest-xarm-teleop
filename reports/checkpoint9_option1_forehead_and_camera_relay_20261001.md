# Checkpoint 9: option 1 (headset on the forehead, cameras on the laptop), 2026-10-01

Fake robot only. Nothing in the lab moved and no arm or gripper command was sent.

## Result
Option 1 works in a dry run: with the headset on the forehead and the low-rate two-camera page open, the clutch stayed engaged for 26.5 s, the controller was tracked in 1751 of 1751 samples during 20 s of movement, and the control link behaved as it does with no video.

| Measure (run `fh_20261001-185214`) | Value |
|---|---|
| Controller tracking during the 20 s of movement | 1751 / 1751 samples tracked and focused |
| Clutch | engaged 26.5 s, ended by the cued release; no drops |
| Input age at a scheduler tick while engaged | median 20 ms, p99 63 ms, max 108 ms (limit 250 ms) |
| Round trip, sender clock | median 231 ms, p95 317 ms, max 579 ms |
| Link | 0 rejected packets, 0 errors |
| Hand travel | about 470 x 420 x 250 mm |

## What was set up
1. **Wear-sensor override on the headset:** `adb shell am broadcast -a com.oculus.vrpowermanager.prox_close`. The headset then reports "mounted" while off the face. Undo: `adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable`, or reboot the headset. While it is active the headset does not sleep by itself when taken off.
2. **Low-rate camera relay:** `scripts/lab_cam_relay.py`, running on `linux2` on loopback port 28090 as a viewer of the existing camera wall. Two views (wrist `iphone`, fixed `zed`), 8 frames per second maximum, 480 px wide, JPEG quality 55, newest frame only. About 200 kB/s in total. It opens no camera device and has no robot code. Viewed locally through `ssh -N -L 127.0.0.1:18091:127.0.0.1:28090 linux2` at `http://127.0.0.1:18091/`.
3. **Session scripts** now always cold-start the Quest app.

## Why the control link was slow earlier
| Situation | Round trip median |
|---|---|
| No camera viewer (gripper run 2) | 222 ms |
| Five full-rate camera streams open in a browser (gripper run 1; A/B run `ab_20261001-184313`) | 355-630 ms, with arrival gaps over 250 ms |
| Two-view relay page open (this run) | 231 ms |

- During the A/B run a browser tab on `localhost:8090` (a separate tunnel, not one of mine) was streaming all five cameras the whole time, so that run has no clean "no video" phase. It still shows the link at 350-630 ms while five streams were flowing.
- Each full-rate stream was held to about 165 kB/s there, which suggests the path carries roughly 1.5 MB/s in total. Five full-rate cameras want several times that.
- Not a controlled experiment: the comparison is across runs at different times.

## Findings
- **The Quest app does not recover from a warm relaunch.** Starting it while it is already running ends its session (STOPPING, IDLE, READY) and no poses follow. It needs a force-stop first. The first forehead attempt failed for this reason, not because of tracking.
- Video delay was not measured. The relay stamps each frame with the lab clock, but the two computers' clocks are not known to agree.
- The wrist view depends on DroidCam staying in the foreground on the phone.
- A second local tunnel forwards the stepper control page (port 8093). It is not mine and I did not use it.

## State left running
- Camera relay on `linux2` (exits by itself 90 minutes after start) and its tunnel on local port 18091, so the page keeps working.
- Wear-sensor override still active on the headset.
- Quest app stopped; no sender, no control tunnel, no lab dry-run process.
