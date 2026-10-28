#!/usr/bin/env bash
# Makes the camera recordings of a session watchable in both places:
#   1. copies the session from the lab host to this PC (nothing is deleted or changed there),
#   2. makes view_<camera>.mp4 (H.264) next to each episode's recordings on this PC,
#   3. copies only those view files back into the same episode folders on the lab host.
# Usage: episodes_videos.sh <session name>
set -u
N=$1; P=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd); . "$P/config/teleop_session.conf" || exit 2; DEST=$P/data/episodes
SSHO="ssh -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10"
bash "$P/scripts/ops/episodes_fetch.sh" "$N" > /dev/null || { echo "fetch failed"; exit 2; }
for E in "$DEST/$N"/episode_[0-9][0-9][0-9][0-9]; do [ -d "$E" ] && bash "$P/scripts/ops/episode_to_mp4.sh" "$E"; done
rsync -a -m --include='*/' --include='view_*.mp4' --exclude='*' -e "$SSHO" "$DEST/$N/" "$LAB_HOST:$LAB_DIR/episodes/$N/" \
  && echo "view files are also on the lab host: ~/$LAB_DIR/episodes/$N/episode_*/view_*.mp4"
