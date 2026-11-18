#!/usr/bin/env bash
# Local wrapper: run a lab-side script detached from this SSH connection and wait for its done marker.
# Usage: lab_run_detached.sh <name> <confirmation-file> <max seconds> <lab script> [args after name...]
set -u
N=$1; CONF=$2; MAXS=$3; SCRIPT=$4; shift 4
SSHO=(-o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
scp -q "${SSHO[@]}" "$CONF" "linux2:quest-xarm-teleop-dryrun/logs/confirm_$N.txt" || exit 2
ssh "${SSHO[@]}" linux2 "cd ~/quest-xarm-teleop-dryrun && rm -f logs/done_$N && (nohup setsid bash scripts/ops/$SCRIPT $N $* > logs/run_$N.log 2>&1 < /dev/null &) ; \
  for i in \$(seq 1 $((MAXS * 2))); do [ -f logs/done_$N ] && break; sleep 0.5; done; \
  [ -f logs/done_$N ] || { echo 'NOT FINISHED in time: sending SIGINT to the test (it parks the arm)'; kill -INT \$(cat logs/pid_$N 2>/dev/null) 2>/dev/null; sleep 5; }; cat logs/run_$N.log"
