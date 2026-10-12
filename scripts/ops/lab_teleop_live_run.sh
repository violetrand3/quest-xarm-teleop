#!/usr/bin/env bash
# Runs ON THE LAB HOST, detached. One live teleop session (REAL ARM).
# Usage: [REC=1] lab_teleop_live_run.sh <name> <duration s> [extra live_teleop.py flags] ; needs logs/confirm_<name>.txt
# REC=1: episodes are recorded into episodes/<name>/ (thumb button A start/stop+save, B discard); the episode videos
#        are then the camera record. Without it, low-rate camera frames are saved for the cross-check as before.
cd ~/quest-xarm-teleop-dryrun || exit 1
N=$1; DUR=$2; shift 2
REC=${REC:-0}; CAP=""; RECFLAGS=()
if [ "$REC" = 1 ]; then
  RECFLAGS=(--record-dir "episodes/$N")
else
  python3 scripts/lab_frame_capture.py logs/cam_$N $((DUR + 30)) 5 zed iphone > logs/cam_$N.log 2>&1 &
  CAP=$!
fi
python3 -u scripts/live_teleop.py --address-from-file /home/dev/robotdash/stepper.py --moves-real-arm \
  --human-confirmation "$(tr '\n"' '  ' < logs/confirm_$N.txt)" --duration "$DUR" \
  --log logs/teleop_$N.jsonl --link-log logs/teleop_link_$N.jsonl --summary logs/teleop_$N.summary.json "${RECFLAGS[@]}" "$@" \
  > logs/teleop_$N.console.txt 2>&1 &
echo $! > logs/pid_$N; wait $!
echo "live_teleop exit code: $?"
if [ -n "$CAP" ]; then
  sleep 1; kill $CAP 2>/dev/null; wait $CAP 2>/dev/null
  for k in zed iphone; do python3 scripts/frames_refdiff.py logs/cam_$N/$k > logs/refdiff_${N}_$k.json 2>/dev/null; echo "frames $k: $(ls logs/cam_$N/$k 2>/dev/null | wc -l)"; done
else
  python3 scripts/episode_inspect.py "episodes/$N" > logs/episodes_$N.check.txt 2>&1; echo "episode check exit code: $?"; cat logs/episodes_$N.check.txt
  echo "episodes on disk: $(du -sh "episodes/$N" 2>/dev/null | cut -f1); free: $(df -h . | awk 'NR==2{print $4}')"
fi
echo "arm connections by pid: $(ss -tnp 2>/dev/null | grep ':502 \|:30002 ' | grep -v 127.0.0.1 | grep -oE 'pid=[0-9]+' | sort -u | tr '\n' ' ')"
touch logs/done_$N
