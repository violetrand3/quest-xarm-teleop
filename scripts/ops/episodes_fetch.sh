#!/usr/bin/env bash
# Copies recorded episodes from the lab host to this PC. Nothing is deleted or changed on the lab host.
# Usage: episodes_fetch.sh <session name> [--list]     -> ~/quest-xarm-teleop/data/episodes/<session name>/
set -u
N=$1; DEST=~/quest-xarm-teleop/data/episodes
SSHO="ssh -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10"
if [ "${2:-}" = "--list" ]; then $SSHO linux2 "cd ~/quest-xarm-teleop-dryrun/episodes/$N && du -sh . && ls"; exit $?; fi
mkdir -p "$DEST" && rsync -a --partial --info=progress2 -e "$SSHO" "linux2:quest-xarm-teleop-dryrun/episodes/$N" "$DEST/" && du -sh "$DEST/$N"
