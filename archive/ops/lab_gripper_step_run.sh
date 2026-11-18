#!/usr/bin/env bash
# Runs ON THE LAB HOST, detached. One gripper step check (REAL jaws move) with wrist-camera frames.
# Usage: lab_gripper_step_run.sh <name> <delta pct> ; needs logs/confirm_<name>.txt
cd ~/quest-xarm-teleop-dryrun || exit 1
N=$1; DELTA=$2; TARGETS=${3:-}
python3 scripts/lab_frame_capture.py logs/cam_$N 45 15 iphone > logs/cam_$N.log 2>&1 &
CAP=$!
sleep 3
( cd scripts && python3 -u gripper_step_check.py --moves-real-gripper --human-confirmation "$(tr '\n"' '  ' < ../logs/confirm_$N.txt)" \
    --delta-pct "$DELTA" --targets "$TARGETS" --dwell 2.2 --out ../logs/gripper_step_$N.json ) 2>&1
echo "check exit code: $?"
sleep 1.5; kill $CAP 2>/dev/null; wait $CAP 2>/dev/null
python3 scripts/wrist_frames_measure.py logs/cam_$N/iphone > logs/frames_$N.json 2>/dev/null; echo "wrist frames: $(ls logs/cam_$N/iphone 2>/dev/null | wc -l)"
touch logs/done_$N
