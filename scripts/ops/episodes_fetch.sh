#!/usr/bin/env bash
# Copies recorded episodes from the lab host to this PC. Nothing is deleted or changed on the lab host.
# Usage: episodes_fetch.sh <session name> [--list]     -> data/episodes/<session name>/ in this checkout
set -u
N=$1; P=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd); . "$P/config/teleop_session.conf" || exit 2; DEST=$P/data/episodes
SSHO="ssh -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10"
if [ "${2:-}" = "--list" ]; then $SSHO "$LAB_HOST" "cd ~/$LAB_DIR/episodes/$N && du -sh . && ls"; exit $?; fi
mkdir -p "$DEST" && rsync -a --partial --info=progress2 -e "$SSHO" "$LAB_HOST:$LAB_DIR/episodes/$N" "$DEST/" && du -sh "$DEST/$N"
