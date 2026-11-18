#!/usr/bin/env bash
# Makes ordinary video files (H.264 .mp4) from an episode's camera recordings, for viewing. Local files only.
# Usage: episode_to_mp4.sh <episode directory>     -> <episode>/view_<camera>.mp4 (the .mjpeg and its frame times stay the data of record)
set -u
E=${1%/}
for f in "$E"/cam_*.mjpeg; do
  [ -f "$f" ] || { echo "no camera recording in $E"; exit 2; }
  c=$(basename "$f" .mjpeg); c=${c#cam_}
  ffmpeg -loglevel error -y -f mjpeg -framerate 30 -i "$f" -c:v libx264 -crf 20 -pix_fmt yuv420p "$E/view_$c.mp4" \
    && echo "$E/view_$c.mp4: $(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames,width,height -of csv=p=0 "$E/view_$c.mp4") (width,height,frames), $(du -h "$E/view_$c.mp4" | cut -f1)"
done
