#!/usr/bin/env bash
# Runs ON THE LAB HOST, detached. One supervised arm_first_move_test.py run with camera frames.
# Usage: lab_first_move_run.sh <name> <mode 0|7> <axis> [extra flag] ; needs logs/confirm_<name>.txt
cd ~/quest-xarm-teleop-dryrun || exit 1
N=$1; MODE=$2; AXIS=$3; shift 3; EXTRA="$*"
python3 scripts/lab_frame_capture.py logs/cam_$N 60 10 zed iphone > logs/cam_$N.log 2>&1 &
CAP=$!
sleep 2.5
python3 -u scripts/arm_first_move_test.py --address-from-file /home/dev/robotdash/stepper.py --moves-real-arm \
  --human-confirmation "$(tr '\n"' '  ' < logs/confirm_$N.txt)" --axis "$AXIS" --distance 10 --speed 10 --acc 100 --mode "$MODE" $EXTRA \
  --out logs/first_move_$N.json > logs/first_move_$N.console.txt 2>&1 &
echo $! > logs/pid_$N; wait $!
echo "test exit code: $?"
grep -v '^\[SDK\]\|^ROBOT_IP\|^SDK_VERSION\|^change protocol\|xArm is \(not \)\?ready' logs/first_move_$N.console.txt | sed -E 's/([0-9]{1,3}\.){3}[0-9]{1,3}/<ip>/g'
sleep 1.5; kill $CAP 2>/dev/null; wait $CAP 2>/dev/null
for k in zed iphone; do python3 scripts/frames_refdiff.py logs/cam_$N/$k > logs/refdiff_${N}_$k.json 2>/dev/null; echo "frames $k: $(ls logs/cam_$N/$k 2>/dev/null | wc -l)"; done
echo "arm connections by pid: $(ss -tnp 2>/dev/null | grep ':502 \|:30002 ' | grep -v 127.0.0.1 | grep -oE 'pid=[0-9]+' | sort -u | tr '\n' ' ')"
touch logs/done_$N
