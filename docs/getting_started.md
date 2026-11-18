# Getting started

How to set up a new operator PC for teleoperating the lab's robots with a Meta Quest 3S: the xArm6, and the two
YAM arms. The setup is the same for both. Daily use: `teleop_usage.md` (xArm), `yam_teleop.md` (YAM).

## How it fits together

```
Quest 3S --USB--> your PC: scripts/teleop --SSH--> lab host (linux2, user dev): ~/quest-xarm-teleop-dryrun
                  (reads the controllers,                        xArm: live_teleop.py talks to the arm, the
                   shows the camera page)                              gripper service and the cameras
                                                                 YAM:  yam_live.py drives both arms (CAN)
```

Everything runs from your PC. `scripts/teleop` starts and stops the program on the lab host over SSH, and
opens the tunnels it needs.

## What you need

- **A Linux PC, or Windows with WSL2** (see below). The launcher uses `ss`, `setsid` and GNU tools, so
  macOS does not work as is.
- **Tools on the PATH:** `python3` (3.10 or newer, standard library only), `ssh`, `rsync` and `adb` (Android
  platform-tools). `scripts/teleop` names any of them that is missing.
- **SSH to the lab host as `dev` without a password prompt** (key login). Check it with `ssh linux2 true`.
  If your SSH config uses another name for that machine, set `LAB_HOST` in `config/teleop_session.conf`.
- **A Meta Quest 3S** with developer mode on and the teleop app installed (below).

## The headset app

The app is `org.questxarm.reader`, build `s4a-ff0da2692500`: our fork of oculus_reader with the changes in
`patches/`. The APK is attached to the repository's GitHub release `quest-app-s4a-ff0da2692500`, not
committed. Its SHA-256 starts with `57f2936ccf175626`.

1. Turn on developer mode for the headset in the Meta Horizon phone app (Devices, then Headset settings,
   then Developer mode). This needs a Meta developer account, which is free.
2. Connect the headset by USB. In the headset, allow USB debugging ("Always allow from this computer").
   `adb devices` must then list it as `device`.
3. Install the app: `adb install org.questxarm.reader-debug-s4a-ff0da2692500.apk`.

Nothing else is needed on the headset. `scripts/teleop` starts the app itself at the beginning of each
session.

## On Windows: WSL2

`scripts/teleop` is a bash program, so on Windows it runs inside WSL2 (Ubuntu). This setup has not been
tried yet. If a step does not work as described here, please report it.

1. Install WSL2 with Ubuntu (`wsl --install` in an administrator PowerShell). In Ubuntu, run
   `sudo apt install python3 rsync openssh-client iproute2 git`.
2. Use the **Windows** adb for the headset, because WSL cannot see USB devices by itself. Install Android
   platform-tools on Windows, then in Ubuntu make `adb` call it:
   ```
   mkdir -p ~/.local/bin
   printf '#!/bin/sh\nexec "/mnt/c/<path to>/platform-tools/adb.exe" "$@"\n' > ~/.local/bin/adb
   chmod +x ~/.local/bin/adb      # a new terminal has ~/.local/bin on the PATH; check with: adb devices
   ```
   adb.exe ends its lines with CRLF (`\r\n`). The launcher allows for that, so the wrapper does not need to
   change them (with a launcher older than 2026-11-02 it counted 0 headsets).
3. SSH from **inside Ubuntu**: put your key in Ubuntu's `~/.ssh` and check `ssh linux2 true`. If the name
   does not resolve there (Tailscale names often don't inside WSL), add a `Host linux2` entry with the
   address you use on Windows to `~/.ssh/config`.
4. Clone the repository inside Ubuntu (for example in `~`), not on `C:`.
5. Open the camera page in your Windows browser at the address `scripts/teleop` prints
   (`http://127.0.0.1:18091/`). WSL2 passes such local addresses through to Windows.

## First time

```
git clone git@github.com:shezitt/quest-xarm-teleop.git && cd quest-xarm-teleop
scripts/teleop status
```

`status` is read-only. It shows the arm, the cameras, the disk, whether the headset and its app are
there, and whether the lab host runs the same programs as your checkout.

The lab host has **one copy of the programs, shared by everyone** who uses the `dev` account
(`~/quest-xarm-teleop-dryrun`). `record` and `try` refuse to start when the programs that run there differ
from your checkout, so a session never runs code you don't have. Files that run only on your PC (such as
the headset reader) are not compared.

To install a new version there, commit it and push it to GitHub first, then:

```
scripts/teleop sync
```

`sync` installs only a version that is committed, pushed, and not older than what the lab host has, and
never while a session is running. So the others can always match it with `git pull`. When it refuses, it
says why:

| Message | What to do |
|---|---|
| changes that are not committed | commit and `git push`, then sync |
| commits that are not on GitHub | `git push`, then sync |
| GitHub has newer commits, or the lab host runs a newer version | `git pull` (then there is usually nothing to sync) |
| the lab host's copy is not a version on GitHub | someone installed work in progress with an older launcher. Ask them to push it, or replace it with `scripts/teleop sync --overwrite` |

## Working with others

Several people can use the rig from their own PCs, one session at a time. They share the lab host's copy, so:

- **`git pull` before every session.** When the lab host runs a newer version than your checkout, the session
  commands refuse to start and say so; `git pull` fixes it. Never `sync --overwrite` someone else's version
  unless they agree.
- **Change the programs only in your clone**, then commit, push and `scripts/teleop sync`. The lab host's copy is
  not a git checkout (the lab host would need a GitHub login for this private repository): `sync` installs it from
  your checkout. Its `README.txt` and `SYNCED_FROM.txt` say which commit it is and who installed it, and `sync`
  moves files that the repository no longer has into a `backup_*_removed` folder there.
- **One session at a time** (one xArm, one pair of YAM arms, one camera page). `scripts/teleop status` says when
  one is running.

To give someone access: invite them to the GitHub repository (it is private), and have their SSH public key added
to the lab account on linux2 by whoever manages it. They then follow this page from "What you need".

## Then

Read `teleop_usage.md` (xArm) or `yam_teleop.md` (YAM): before a session, the controls, and troubleshooting. Then:

```
scripts/teleop record          # xArm
scripts/teleop yam-sim         # YAM, simulated: nothing moves; then scripts/teleop yam-record
```

## Rules of the rig

- The arms are shared and safety-sensitive. Someone must be at the rig with the physical stop (xArm) or the
  e-stop (YAM) in reach, nobody near the arms, and no other program commanding them (for example a policy
  started from the lab's stepper page, or another agent on the lab host).
- The software never clears controller errors and never changes the controller's safety settings (fence,
  reduced mode). Those are lab-side decisions.
- Only one session at a time. `status` says when a session is already running.
- The YAM arms go limp and drop when their program or the power stops; a YAM session lowers them to rest first
  (`yam_teleop.md`).
- Ctrl-C and `scripts/teleop stop` are software stops, not the physical stop.
