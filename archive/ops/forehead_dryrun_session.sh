#!/usr/bin/env bash
# Option-1 check: headset on the forehead, operator watches the low-rate camera page on the laptop.
# FAKE ROBOT ONLY (teleop_dryrun on linux2). Usage: forehead_dryrun_session.sh <name>
set -u
P=~/quest-xarm-teleop; D=$P/data/checkpoint9; R=$P/run; N=$1
SSHO=(-o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
S=$(adb devices | awk 'NR>1 && $2=="device"{print $1}'); [ "$(echo "$S" | grep -c .)" = 1 ] || { echo "need exactly one headset on adb"; exit 2; }
say() { spd-say -w -l es "$1" >/dev/null 2>&1; }
mark() { python3 -c 'import time,sys;print(time.monotonic_ns(), sys.argv[1])' "$1" >> "$D/marks_$N.txt"; echo "MARK $1"; }
st() { tail -n 1 "$D/dry_$N.console.txt" 2>/dev/null | awk -F' [|] ' '{split($2,a," "); print a[1]}'; }
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
ack() { ssh "${SSHO[@]}" linux2 "pkill -USR1 -f '[t]eleop_dryrun.py --config' && echo 'operator acknowledgement sent'"; }
finish() {
  T=$(cat "$R/tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null; for i in $(seq 1 60); do kill -0 "$T" 2>/dev/null || break; sleep 0.25; done; echo "sender stopped"
  sed -i "s/$S/[REDACTED]/g" "$D/tx_$N.console.txt" 2>/dev/null
  T=$(cat "$R/tunnel_link.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:18765:127.0.0.1:28765" && kill "$T" && echo "link tunnel closed"
  ssh "${SSHO[@]}" linux2 "pkill -INT -f '[t]eleop_dryrun.py --config'"; G=$(cat "$R/dry.pid" 2>/dev/null); for i in $(seq 1 40); do kill -0 "$G" 2>/dev/null || break; sleep 0.5; done
  scp -q "${SSHO[@]}" "linux2:quest-xarm-teleop-dryrun/logs/fh_$N.*" "linux2:quest-xarm-teleop-dryrun/logs/fh_link_$N.*" "$D/" && echo fetched
  adb -s "$S" shell am force-stop org.questxarm.reader && echo "app force-stopped"; echo "DONE $N ($1)"
}
ssh "${SSHO[@]}" linux2 "cd ~/quest-xarm-teleop-dryrun/scripts && exec python3 -u teleop_dryrun.py --config ../config/teleop_dryrun_example.json --port 28765 --duration 300 --log ../logs/fh_$N.jsonl --link-log ../logs/fh_link_$N.jsonl --summary ../logs/fh_$N.summary.json" > "$D/dry_$N.console.txt" 2>&1 &
echo $! > "$R/dry.pid"
for i in $(seq 1 60); do grep -q "DRY RUN" "$D/dry_$N.console.txt" 2>/dev/null && break; sleep 0.25; done
ssh "${SSHO[@]}" -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -L 127.0.0.1:18765:127.0.0.1:28765 linux2 > "$R/tunnel_link.txt" 2>&1 &
echo $! > "$R/tunnel_link.pid"; for i in $(seq 1 40); do ss -ltn | grep -q '127.0.0.1:18765 ' && break; sleep 0.25; done
( cd "$P/scripts" && exec python3 -u quest_pose_stream_sender.py --serial "$S" --port 18765 --max-rate 30 --log "$D/tx_$N.jsonl" --source-log "$D/src_$N.jsonl" --summary "$D/tx_$N.summary.json" > "$D/tx_$N.console.txt" 2>&1 ) &
echo $! > "$R/tx.pid"; sleep 2
echo "relay viewers at start: $(curl -s -m 3 http://127.0.0.1:18091/stats)"
adb -s "$S" shell am force-stop org.questxarm.reader; sleep 1  # always a cold start: a warm relaunch leaves the app without a session
mark "launch app"; adb -s "$S" shell am start -W -n org.questxarm.reader/com.rail.oculus.teleop.MainActivity 2>&1 | grep -E "Status|LaunchState"
say "Visor en la frente, como una gorra. Control derecho delante de ti."
wait_avail 2 90 || { say "No se detecta el control. Paramos."; mark "abort: no input"; finish abort; exit 3; }
sleep 4; mark "acknowledge start-up notices"; ack; sleep 1.5; mark "state before cues = $(st)"
ok=0
for attempt in 1 2 3; do
  say "Suelta el botón del dedo medio."; s=$(wait_state "ARMED" 10) || { [ "$s" = "FAULT_LATCHED" ] && { ack; sleep 1; continue; }; }
  mark "attempt $attempt: after release cue = $s"
  say "Aprieta y mantén el dedo medio."; s=$(wait_state "ACTIVE FAULT_LATCHED" 10); mark "attempt $attempt: after press cue = $s"
  [ "$s" = "ACTIVE" ] && { ok=1; break; }; [ "$s" = "FAULT_LATCHED" ] && { ack; sleep 1; }
done
[ $ok = 1 ] || { say "No se pudo activar. Paramos."; mark "abort: engagement failed"; finish abort; exit 4; }
mark "cue move"; say "Mueve el control despacio durante veinte segundos, mirando la pantalla."
sleep 20
mark "state before release cue = $(st)"; mark "cue release"; say "Suelta el botón. Terminado."
s=$(wait_state "WAIT_RELEASE ARMED FAULT_LATCHED" 8); mark "final state = $s"; sleep 1
echo "relay stats at end: $(curl -s -m 3 http://127.0.0.1:18091/stats)"
mark "end"; finish ok
