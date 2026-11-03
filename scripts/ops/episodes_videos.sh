#!/usr/bin/env bash
# Makes the camera recordings of a session watchable, and copies what is needed for watching to this PC:
#   1. on the lab host, makes view_<camera>.mp4 (H.264) next to each episode's camera recordings, all cameras and episodes
#      at once (at most 4 at a time, lowest priority, so other work on the lab host is not slowed); a view file that is
#      already there is kept,
#   2. copies the session to this PC WITHOUT the raw camera recordings (cam_*.mjpeg, about 0.7 MB/s per camera): the
#      view files, steps, frame times and events. The raw recordings stay on the lab host (nothing there is deleted);
#      scripts/ops/episodes_fetch.sh <session> copies everything, raw recordings included.
# Usage: episodes_videos.sh <session name>
set -u
N=$1; P=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd); . "$P/config/teleop_session.conf" || exit 2; DEST=$P/data/episodes
SSHO="ssh -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10"
t0=$SECONDS; rc=0
# shellcheck disable=SC2016
$SSHO "$LAB_HOST" bash -s -- "$LAB_DIR" "$N" <<'LAB' || { echo "some videos were not made on the lab host (above)"; rc=1; }
set -u
cd ~/"$1/episodes/$2" || exit 2
one() {   # <episode dir>/cam_<camera>.mjpeg -> <episode dir>/view_<camera>.mp4 (written aside, then renamed)
  f=$1; e=$(dirname "$f"); c=$(basename "$f" .mjpeg); c=${c#cam_}; out=$e/view_$c.mp4; tmp=$e/.view_$c.mp4.part
  [ -s "$out" ] && [ "$out" -nt "$f" ] && { echo "$out: already made"; return 0; }
  nice -n 19 ffmpeg -nostdin -loglevel error -y -f mjpeg -framerate 30 -i "$f" -c:v libx264 -crf 20 -pix_fmt yuv420p -threads 4 -f mp4 "$tmp" \
    && mv -f "$tmp" "$out" \
    && echo "$out: $(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames,width,height -of csv=p=0 "$out") (width,height,frames), $(du -h "$out" | cut -f1)" \
    || { rm -f "$tmp"; echo "$out: FAILED"; return 1; }
}
export -f one
ls episode_[0-9][0-9][0-9][0-9]/cam_*.mjpeg > /dev/null 2>&1 || { echo "no camera recordings in $PWD"; exit 0; }
printf '%s\0' episode_[0-9][0-9][0-9][0-9]/cam_*.mjpeg | xargs -0 -n 1 -P 4 bash -c 'one "$1"' _
LAB
echo "videos made on the lab host in $((SECONDS - t0)) s"; t1=$SECONDS
mkdir -p "$DEST" && rsync -a -z --partial --exclude='cam_*.mjpeg' --exclude='.view_*.part' -e "$SSHO" "$LAB_HOST:$LAB_DIR/episodes/$N" "$DEST/" \
  || { echo "copying the session failed"; exit 2; }
echo "copied to this PC in $((SECONDS - t1)) s (without the raw camera recordings): $(du -sh "$DEST/$N" | cut -f1)"
echo "raw camera recordings stay on the lab host: ~/$LAB_DIR/episodes/$N/episode_*/cam_*.mjpeg (scripts/ops/episodes_fetch.sh $N copies them)"
exit $rc
