#!/usr/bin/env bash
# One live session: Quest index trigger -> REAL gripper on linux2 (arm is FAKE, no arm connection).
# Usage: gripper_live_session.sh <name> <confirmation-file>
# Touches only: this session's own processes, org.questxarm.reader, and the gripper via live_gripper_test.py.
set -u
P=~/quest-xarm-teleop; D=$P/data/checkpoint8; R=$P/run; N=$1; CONF=$2
SSHO=(-o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
serial() { adb devices | awk 'NR>1 && $2=="device"{print $1}'; }
say() { spd-say -w -l es "$1" >/dev/null 2>&1; }
mark() { python3 -c 'import time,sys;print(time.monotonic_ns(), sys.argv[1])' "$1" >> "$D/marks_$N.txt"; echo "MARK $1"; }
st() { tail -n 1 "$D/gripper_$N.console.txt" 2>/dev/null | awk -F' [|] ' '{split($2,a," "); print a[1]}'; }
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
        if time.monotonic()-since>=need: print("available"); sys.exit(0)
    else: since=None
    time.sleep(0.05)
print("NOT available within limit"); sys.exit(1)
PY
}
ack() { ssh "${SSHO[@]}" linux2 "pkill -USR1 -f '[l]ive_gripper_test.py --config' && echo 'operator acknowledgement sent'"; }
finish() {
  T=$(cat "$R/tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null; for i in $(seq 1 60); do kill -0 "$T" 2>/dev/null || break; sleep 0.25; done; echo "sender stopped"
  S=$(serial); [ -n "$S" ] && sed -i "s/$S/[REDACTED]/g" "$D/tx_$N.console.txt" 2>/dev/null
  T=$(cat "$R/tunnel_link.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:18765:127.0.0.1:28765" && kill "$T" && echo "link tunnel closed"
  ssh "${SSHO[@]}" linux2 "pkill -INT -f '[l]ive_gripper_test.py --config' && echo 'gripper test SIGINT sent (restores jaws, then exits)'"
  G=$(cat "$R/grip.pid" 2>/dev/null); for i in $(seq 1 60); do kill -0 "$G" 2>/dev/null || break; sleep 0.5; done; kill -0 "$G" 2>/dev/null && echo "remote STILL running" || echo "remote exited"
  ssh "${SSHO[@]}" linux2 "pkill -f 'python3 - .*logs/cam_$N' ; true"; C=$(cat "$R/camcap.pid" 2>/dev/null); kill "$C" 2>/dev/null
  scp -q "${SSHO[@]}" "linux2:quest-xarm-teleop-dryrun/logs/gripper_$N.*" "linux2:quest-xarm-teleop-dryrun/logs/gripper_link_$N.*" "$D/" && echo fetched
  scp -q -r "${SSHO[@]}" "linux2:quest-xarm-teleop-dryrun/logs/cam_$N" "$D/" && echo "wrist frames fetched: $(ls "$D/cam_$N" | wc -l)"
  [ -n "$S" ] && adb -s "$S" shell am force-stop org.questxarm.reader && echo "app force-stopped"
  echo "DONE $N ($1)"
}
engage() {
  for attempt in 1 2; do
    say "Suelta el botón del dedo medio."
    s=$(wait_state "ARMED" 12) || { [ "$s" = "FAULT_LATCHED" ] && { mark "$1: latched before release; acknowledgement"; ack; sleep 1; continue; }; }
    mark "$1: after release cue = $s"
    say "Aprieta y mantén el dedo medio."
    s=$(wait_state "ACTIVE FAULT_LATCHED" 12); mark "$1: after press cue = $s"
    [ "$s" = "ACTIVE" ] && return 0
    [ "$s" = "FAULT_LATCHED" ] && { mark "$1: acknowledgement"; ack; sleep 1; }
  done
  return 1
}
S=$(serial); [ "$(echo "$S" | grep -c .)" = 1 ] || { echo "need exactly one headset on adb"; exit 2; }
scp -q "${SSHO[@]}" "$CONF" "linux2:quest-xarm-teleop-dryrun/logs/confirm_$N.txt" || exit 2
# wrist-camera frames, captured ON linux2 from its own loopback camera wall (view-only GET), 2 per second,
# so that no video crosses the network during the run; fetched afterwards
ssh "${SSHO[@]}" linux2 "mkdir -p ~/quest-xarm-teleop-dryrun/logs/cam_$N && exec python3 - ~/quest-xarm-teleop-dryrun/logs/cam_$N" > /dev/null 2>&1 <<'PY' &
import sys,time,urllib.request
out=sys.argv[1]; last=0; end=time.monotonic()+420
try:
    r=urllib.request.urlopen("http://127.0.0.1:8090/cam/iphone",timeout=10)
    while time.monotonic()<end:
        line=r.readline()
        if not line: break
        if not line.startswith(b"--frame"): continue
        n=None
        while True:
            h=r.readline().strip()
            if not h: break
            if h.lower().startswith(b"content-length"): n=int(h.split(b":")[1])
        d=r.read(n)
        if time.monotonic()-last>=0.5 and len(d)>12000:
            last=time.monotonic(); open(f"{out}/{time.monotonic_ns()}.jpg","wb").write(d)
except Exception: pass
PY
echo $! > "$R/camcap.pid"
ssh "${SSHO[@]}" linux2 "cd ~/quest-xarm-teleop-dryrun/scripts && exec python3 -u live_gripper_test.py --config ../config/teleop_dryrun_example.json --moves-real-gripper --human-confirmation \"\$(tr '\n\"' '  ' < ../logs/confirm_$N.txt)\" --port 28765 --duration 420 --log ../logs/gripper_$N.jsonl --link-log ../logs/gripper_link_$N.jsonl --summary ../logs/gripper_$N.summary.json" > "$D/gripper_$N.console.txt" 2>&1 &
echo $! > "$R/grip.pid"
for i in $(seq 1 60); do grep -q "GRIPPER LIVE" "$D/gripper_$N.console.txt" 2>/dev/null && break; kill -0 "$(cat "$R/grip.pid")" 2>/dev/null || break; sleep 0.25; done
head -c 330 "$D/gripper_$N.console.txt"; echo
grep -q "GRIPPER LIVE" "$D/gripper_$N.console.txt" || { echo "remote did not start"; finish abort; exit 3; }
ssh "${SSHO[@]}" -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -L 127.0.0.1:18765:127.0.0.1:28765 linux2 > "$R/tunnel_link.txt" 2>&1 &
echo $! > "$R/tunnel_link.pid"; for i in $(seq 1 40); do ss -ltn | grep -q '127.0.0.1:18765 ' && break; sleep 0.25; done
ss -ltn | grep -q '127.0.0.1:18765 ' || { echo "link tunnel FAILED"; finish abort; exit 3; }
( cd "$P/scripts" && exec python3 -u quest_pose_stream_sender.py --serial "$S" --port 18765 --max-rate 30 --log "$D/tx_$N.jsonl" --source-log "$D/src_$N.jsonl" --summary "$D/tx_$N.summary.json" > "$D/tx_$N.console.txt" 2>&1 ) &
echo $! > "$R/tx.pid"; sleep 2
adb -s "$S" shell am force-stop org.questxarm.reader; sleep 1  # always a cold start: a warm relaunch leaves the app without a session
mark "launch app"; adb -s "$S" shell am start -W -n org.questxarm.reader/com.rail.oculus.teleop.MainActivity 2>&1 | grep -E "Status|LaunchState"
say "Ponte el visor. Control derecho en la mano. No aprietes nada."
wait_avail 2 90 || { say "No se detecta el control. Paramos."; mark "abort: no input"; finish abort; exit 3; }
sleep 5; mark "acknowledge start-up notices"; ack; sleep 1.5; mark "state before cues = $(st)"
engage "engagement" || { say "No se pudo activar. Paramos."; mark "abort: engagement failed"; finish abort; exit 4; }
sleep 1
mark "cue squeeze 1";  say "Aprieta el gatillo del índice muy despacio, hasta el fondo."; sleep 6
mark "cue release 1";  say "Suelta el gatillo muy despacio."; sleep 6
mark "cue squeeze 2";  say "Otra vez. Aprieta despacio."; sleep 5
mark "cue release 2";  say "Suelta despacio."; sleep 5
mark "cue half";       say "Ahora a la mitad, y mantén."; sleep 4
mark "state before clutch release = $(st)"
mark "cue clutch release"; say "Suelta el botón del dedo medio. Mantén el gatillo."
s=$(wait_state "WAIT_RELEASE ARMED FAULT_LATCHED" 10); mark "state after clutch release = $s"; sleep 1
mark "cue squeeze while disengaged"; say "Ahora aprieta el gatillo a fondo."; sleep 3.5
mark "cue end"; say "Suelta todo. Terminado."; sleep 2
mark "end"
finish ok
