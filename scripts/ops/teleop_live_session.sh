#!/usr/bin/env bash
# One LIVE teleop session: Quest right controller -> REAL arm on linux2. Normally started by scripts/teleop.
# Usage: [REC=1] teleop_live_session.sh <name> <confirmation-file> <teleop seconds> [extra live_teleop.py flags]
# REC=1: episodes are recorded ON THE LAB HOST (episodes/<name>/): thumb button A starts / stops and saves, B discards.
# The lab program runs detached and parks the arm by itself (on SIGINT, on its own duration limit, or on a fault).
# The console shows only what the operator needs; everything else goes to <data dir>/detail_<name>.txt.
set -u
P=~/quest-xarm-teleop; D=${TELEOP_DATA_DIR:-$P/data/checkpoint10}; R=$P/run; N=$1; CONF=$2; TEL=$3; shift 3
mkdir -p "$D" "$R"
L=quest-xarm-teleop-dryrun/logs; REC=${REC:-0}; DET=$D/detail_$N.txt
SSHO=(-n -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
S=$(adb devices | awk 'NR>1 && $2=="device"{print $1}'); [ "$(echo "$S" | grep -c .)" = 1 ] || { echo "need exactly one headset on adb"; exit 2; }
msg() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$1"; }                                                         # for the operator
mark() { python3 -c 'import time,sys;print(time.monotonic_ns(), sys.argv[1])' "$1" >> "$D/marks_$N.txt"; }  # timeline, log only
last() { tail -n 1 "$D/teleop_$N.console.txt" 2>/dev/null; }
st() { last | awk -F' [|] ' '{split($2,a," "); print a[1]}'; }
why() { last | sed -n 's/^[^|]*| [A-Z_]* \[\([^]]*\)\].*/\1/p'; }
epf() { last | sed -n 's/.* | ep=\(.*\)$/\1/p'; }   # e.g. "rec#2:14s saved=1 disc=0 ref=0"
num() { echo "$1" | sed -n "s/.*$2=\([0-9]*\).*/\1/p"; }
wait_state() { local end=$((SECONDS + $2)); while [ $SECONDS -lt $end ]; do s=$(st); for w in $1; do [ "$s" = "$w" ] && { echo "$s"; return 0; }; done; sleep 0.2; done; echo "$(st)"; return 1; }
wait_avail() { python3 - "$D/src_$N.jsonl" "$1" "$2" <<'PY'
import json,sys,time
path,need,maxs=sys.argv[1],float(sys.argv[2]),float(sys.argv[3]); t0=time.monotonic(); since=None
def ok():
    try:
        with open(path,'rb') as f:
            f.seek(0,2); f.seek(max(0,f.tell()-6000)); lines=f.read().decode('utf-8','replace').splitlines()[1:]
        r=json.loads(lines[-1]); return r.get('available') is True and (time.monotonic_ns()-r['host_ns'])<300e6
    except Exception: return False
while time.monotonic()-t0<maxs:
    if ok():
        since=since or time.monotonic()
        if time.monotonic()-since>=need: sys.exit(0)
    else: since=None
    time.sleep(0.05)
sys.exit(1)
PY
}
ack() { ssh "${SSHO[@]}" linux2 "kill -USR1 \$(cat ~/$L/pid_$N)" >> "$DET" 2>&1; }
finish() {
  T=$(cat "$R/tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null; for i in $(seq 1 60); do kill -0 "$T" 2>/dev/null || break; sleep 0.25; done
  sed -i "s/$S/[REDACTED]/g" "$D/tx_$N.console.txt" 2>/dev/null
  T=$(cat "$R/tunnel_link.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:18765:127.0.0.1:28765" && kill "$T"
  ssh "${SSHO[@]}" linux2 "kill -INT \$(cat ~/$L/pid_$N) 2>/dev/null; for i in \$(seq 1 60); do [ -f ~/$L/done_$N ] && break; sleep 0.5; done; [ -f ~/$L/done_$N ] && echo 'lab program finished and parked' || echo 'LAB PROGRAM NOT FINISHED'; cat ~/$L/run_$N.log" >> "$DET" 2>&1
  if grep -q 'lab program finished and parked' "$DET"; then msg "The arm program has ended and parked the arm."
  else msg "WARNING: the arm program did not confirm that it ended. Look at the arm; use the physical stop if it is moving."; fi
  T=$(cat "$R/tailc.pid" 2>/dev/null); kill "$T" 2>/dev/null
  if [ "$REC" = 1 ]; then scp -q "${SSHO[@]:1}" "linux2:$L/teleop_$N.*" "linux2:$L/teleop_link_$N.jsonl" "linux2:$L/episodes_$N.check.txt" "$D/" >> "$DET" 2>&1
  else scp -q "${SSHO[@]:1}" "linux2:$L/teleop_$N.*" "linux2:$L/teleop_link_$N.jsonl" "linux2:$L/refdiff_${N}_*.json" "$D/" >> "$DET" 2>&1; fi
  adb -s "$S" shell am force-stop org.questxarm.reader >> "$DET" 2>&1; echo "DONE $N ($1)" >> "$DET"
}
videos() {
  if [ "$REC" = 1 ] && [ "${MAKE_VIDEOS:-1}" = 1 ]; then
    msg "Copying the demos here and making the videos ..."
    if bash "$P/scripts/ops/episodes_videos.sh" "$N" >> "$DET" 2>&1; then msg "Videos are ready."; else msg "The videos were not made; see $DET"; fi
  fi
  return 0
}

scp -q "${SSHO[@]:1}" "$CONF" "linux2:$L/confirm_$N.txt" || { echo "cannot reach linux2"; exit 2; }
msg "Starting the arm program on linux2 ..."
ssh "${SSHO[@]}" linux2 "cd ~/quest-xarm-teleop-dryrun && rm -f logs/done_$N && (REC=$REC nohup setsid bash scripts/ops/lab_teleop_live_run.sh $N $((TEL + 90)) $* > logs/run_$N.log 2>&1 < /dev/null &)"
: > "$D/teleop_$N.console.txt"
ssh "${SSHO[@]}" linux2 "tail -n +1 -F ~/$L/teleop_$N.console.txt 2>/dev/null" > "$D/teleop_$N.console.txt" 2>/dev/null &
echo $! > "$R/tailc.pid"
for i in $(seq 1 100); do grep -q "Armed in mode\|ABORTED\|Traceback\|^error" "$D/teleop_$N.console.txt" 2>/dev/null && break; sleep 0.25; done
if ! grep -q "Armed in mode" "$D/teleop_$N.console.txt"; then
  msg "NOT STARTED. The arm program says: $(grep -m1 'ABORTED\|^error\|Error' "$D/teleop_$N.console.txt" | sed -E 's/([0-9]{1,3}\.){3}[0-9]{1,3}/<ip>/g' | cut -c1-400)"
  finish abort; exit 3
fi
msg "Arm armed. $(grep -m1 '^workspace' "$D/teleop_$N.console.txt" | cut -c1-200)"
# Ctrl-C (or a kill) from here on is a clean software stop: sender and tunnel closed, the lab program told to park the arm.
trap 'trap "" INT TERM; echo; msg "INTERRUPTED: ending the session."; mark "interrupted by the operator"; finish interrupted; videos; exit 130' INT TERM
ssh "${SSHO[@]:1}" -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -L 127.0.0.1:18765:127.0.0.1:28765 linux2 > "$R/tunnel_link.txt" 2>&1 < /dev/null &
echo $! > "$R/tunnel_link.pid"; for i in $(seq 1 40); do ss -ltn | grep -q '127.0.0.1:18765 ' && break; sleep 0.25; done
( cd "$P/scripts" && exec python3 -u quest_pose_stream_sender.py --serial "$S" --port 18765 --max-rate 30 --log "$D/tx_$N.jsonl" --source-log "$D/src_$N.jsonl" --summary "$D/tx_$N.summary.json" > "$D/tx_$N.console.txt" 2>&1 ) &
echo $! > "$R/tx.pid"; sleep 2
msg "Starting the headset app and looking for the right controller ..."
adb -s "$S" shell am broadcast -a com.oculus.vrpowermanager.prox_close > /dev/null 2>&1   # keeps the headset awake while it is not worn
adb -s "$S" shell am force-stop org.questxarm.reader; sleep 1
mark "launch app"; adb -s "$S" shell am start -W -n org.questxarm.reader/com.rail.oculus.teleop.MainActivity >> "$DET" 2>&1
wait_avail 2 90 || { msg "No controller input after 90 s. Is the headset awake and the right controller on and in its view? Ending."; mark "abort: no input"; finish abort; exit 3; }
msg "Controller found."
sleep 4; mark "acknowledge start-up notices"; ack; sleep 1.5; mark "state before cues = $(st)"
[ "$(st)" = "ARMED" ] || msg "Release the side grip (middle finger)."
s=$(wait_state "ARMED" 12); mark "after release cue = $s"
[ "$s" = "ARMED" ] || { ack; sleep 1.5; s=$(wait_state "ARMED" 8); mark "after second acknowledgement = $s"; }
[ "$s" = "ARMED" ] || { msg "Could not get ready (state $s: $(why)). Ending."; mark "abort: not armed ($s)"; finish abort; exit 4; }
mark "teleop window start"
msg "READY for $((TEL / 60)) min. Hold the side grip (middle finger) to move the arm, release to pause. Index trigger closes the jaws."
[ "$REC" = 1 ] && msg "Demos: thumb button A starts one and, pressed again, saves it. B discards the one in progress."
prev="ARMED"; end=$((SECONDS + TEL)); fault=0; pw=""; psv=0; pds=0; prf=0; warned=0
while [ $SECONDS -lt $end ]; do
  grep -q '^{"result"' "$D/teleop_$N.console.txt" 2>/dev/null && { mark "the lab program ended before the window did"; msg "The arm program ended."; break; }   # e.g. "teleop stop" from another terminal
  s=$(st)
  if [ -n "$s" ] && [ "$s" != "$prev" ]; then
    mark "state $prev -> $s"
    case "$s" in
      ACTIVE) msg "  arm following";;
      FAULT_LATCHED) fault=1; break;;
      *) [ "$prev" = "ACTIVE" ] && msg "  paused";;
    esac
    prev=$s
  fi
  if [ "$REC" = 1 ]; then
    e=$(epf)
    if [ -n "$e" ]; then
      w=${e%% *}; w=${w%%#*}; sv=$(num "$e" saved); ds=$(num "$e" disc); rf=$(num "$e" ref)
      [ "${sv:-0}" -gt "$psv" ] && { mark "episode saved: $e"; msg "DEMO SAVED (saved so far: $sv)"; }
      [ "${ds:-0}" -gt "$pds" ] && { mark "episode discarded: $e"; msg "DEMO DISCARDED"; }
      [ "${rf:-0}" -gt "$prf" ] && { mark "episode refused: $e"; msg "DEMO NOT STARTED: a camera is not delivering or the disk is nearly full"; }
      [ "$w" = rec ] && [ "$pw" != rec ] && { mark "episode recording: $e"; msg "DEMO RECORDING ..."; }
      [ "$w" = error ] && [ "$pw" != error ] && { mark "episode recorder error"; msg "RECORDING ERROR: demos are no longer being recorded"; }
      psv=${sv:-0}; pds=${ds:-0}; prf=${rf:-0}; pw=$w
    fi
  fi
  [ $warned = 0 ] && [ $((end - SECONDS)) -le 20 ] && { warned=1; msg "20 seconds left$([ "$pw" = rec ] && echo ': press A now to save the demo in progress')"; }
  sleep 0.25
done
if [ $fault = 1 ]; then mark "fault latched; ending"; msg "FAULT: $(why). The arm is stopped; the session ends."; else mark "cue end"; msg "Time is up. Release the side grip."; fi
sleep 1.5; mark "end"
finish ok
videos
exit 0
