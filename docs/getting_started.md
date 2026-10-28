# Getting started

How to set up a new operator PC for teleoperating the lab's xArm6 with a Meta Quest 3S. Daily use is in
`teleop_usage.md`.

## How it fits together

```
Quest 3S --USB--> your PC: scripts/teleop --SSH--> lab host (linux2, user dev): ~/quest-xarm-teleop-dryrun
                  (reads the controller,                         live_teleop.py talks to the arm, the
                   shows the camera page)                        gripper service and the cameras
```

Everything runs from your PC. `scripts/teleop` starts and stops the program on the lab host over SSH, and
opens the tunnels it needs.

## What you need

- **A Linux PC.** The launcher uses `ss`, `setsid` and GNU tools, so macOS does not work as is.
- **Tools on the PATH:** `python3` (3.10 or newer, standard library only), `ssh`, `rsync` and `adb` (Android
  platform-tools). `scripts/teleop` names any of them that is missing.
- **SSH to the lab host as `dev` without a password prompt** (key login). Check it with `ssh linux2 true`.
  If your SSH config uses another name for that machine, set `LAB_HOST` in `config/teleop_session.conf`.
- **The Quest headset** with developer mode on, and the teleop app `org.questxarm.reader` installed. The
  lab's headset already has it. The app is our fork of oculus_reader with the changes in `patches/`. Its APK
  is not in this repository, so ask for it before using another headset.

## First time

```
git clone <the repository> && cd quest-xarm-teleop
scripts/teleop status
```

`status` is read-only. It shows the arm, the cameras, the disk, whether the headset and its app are
there, and whether the lab host runs the same programs as your checkout.

The lab host has **one copy of the programs, shared by everyone** who uses the `dev` account
(`~/quest-xarm-teleop-dryrun`). `record` and `try` refuse to start when it differs from your checkout,
so a session never runs code you don't have. When yours is the newer version, install it with:

```
scripts/teleop sync      # refused while a session is running; records which version was installed and by whom
```

If the other person's version is newer, update your checkout (`git pull`) instead of syncing over theirs.

## Then

Read `teleop_usage.md` (before a session, the controls, and troubleshooting), then run:

```
scripts/teleop record
```

## Rules of the rig

- The arm is shared and safety-sensitive. Someone must be at the rig with the physical stop in reach,
  nobody near the arm, and no other program commanding it (for example a policy started from the lab's
  stepper page).
- The software never clears controller errors and never changes the controller's safety settings (fence,
  reduced mode). Those are lab-side decisions.
- Only one session at a time: there is one arm. `status` says when a session is already running.
- Ctrl-C and `scripts/teleop stop` are software stops, not the physical stop.
