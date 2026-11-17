#!/usr/bin/env bash
# One bimanual YAM SIMULATION session: both Quest controllers -> two SIMULATED YAM arms on the lab host.
# NO HARDWARE: the lab program (scripts/yam_sim_live.py) opens no CAN link. Normally started by scripts/teleop yam-sim.
# Usage: yam_sim_session.sh <name>
# Ctrl-C ends it. Enter acknowledges a latched hand problem (e.g. after the headset's view was re-centred).
set -u
P=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd); . "$P/config/teleop_session.conf" || exit 2
H=$LAB_HOST; D=${TELEOP_DATA_DIR:-$P/data/sessions}; R=$P/run; N=$1; L=$LAB_DIR/logs
LAB_PY=${YAM_PYTHON:-/home/dev/yam/i2rt/.venv/bin/python}   # I2RT's environment on the lab host
LINK_PORT=18766; VIEW_PORT=18767                               # this PC; the lab program listens on 28766 / 28767
mkdir -p "$D" "$R"
SSHO=(-n -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
S=$(adb devices | tr -d '\r' | awk 'NR>1 && $2=="device"{print $1}'); [ "$(echo "$S" | grep -c .)" = 1 ] || { echo "need exactly one headset on adb"; exit 2; }
msg() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$1"; }
for p in $LINK_PORT $VIEW_PORT; do ss -ltn | grep -q "127.0.0.1:$p " && { msg "Port $p on this PC is in use (another session?). Ending."; exit 2; }; done

finish() {
  T=$(cat "$R/yam_tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null; for i in $(seq 1 40); do kill -0 "$T" 2>/dev/null || break; sleep 0.25; done
  sed -i "s/$S/[REDACTED]/g" "$D/yam_tx_$N.console.txt" 2>/dev/null
  T=$(cat "$R/yam_tunnel.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:$LINK_PORT:" && kill "$T"
  ssh "${SSHO[@]}" "$H" "kill -INT \$(cat ~/$L/yam_pid_$N) 2>/dev/null; for i in \$(seq 1 20); do grep -q '^{\"result\"' ~/$L/yam_$N.console.txt && break; sleep 0.5; done; tail -n 1 ~/$L/yam_$N.console.txt" > "$D/yam_end_$N.txt" 2>&1
  grep -q '"result"' "$D/yam_end_$N.txt" && msg "The simulation has ended." || msg "WARNING: the simulation program did not confirm that it ended (it opens no hardware)."
  T=$(cat "$R/yam_tail.pid" 2>/dev/null); kill "$T" 2>/dev/null
  adb -s "$S" shell am force-stop org.questxarm.reader > /dev/null 2>&1
}

msg "Starting the YAM simulation on $H (no hardware) ..."
ssh "${SSHO[@]}" "$H" "test -x $LAB_PY" || { msg "I2RT's Python ($LAB_PY) is not on $H. Ending."; exit 2; }
ssh "${SSHO[@]}" "$H" "cd ~/$LAB_DIR && mkdir -p logs && (nohup setsid $LAB_PY -u scripts/yam_sim_live.py --operator-yaw ${OPERATOR_YAW:-0} --log logs/yam_$N.jsonl > logs/yam_$N.console.txt 2>&1 < /dev/null & echo \$! > logs/yam_pid_$N)"
: > "$D/yam_$N.console.txt"
ssh "${SSHO[@]}" "$H" "tail -n +1 -F ~/$L/yam_$N.console.txt 2>/dev/null" > "$D/yam_$N.console.txt" 2>/dev/null &
echo $! > "$R/yam_tail.pid"
for i in $(seq 1 120); do grep -q "Listening on\|Traceback\|^error" "$D/yam_$N.console.txt" 2>/dev/null && break; sleep 0.25; done
if ! grep -q "Listening on" "$D/yam_$N.console.txt"; then
  msg "NOT STARTED. The simulation program says: $(grep -v 'INFO' "$D/yam_$N.console.txt" | grep -m1 'Error\|error' | cut -c1-300)"
  finish; exit 3
fi
trap 'trap "" INT TERM; echo; msg "Ending the session."; finish; exit 130' INT TERM
ssh "${SSHO[@]:1}" -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:$LINK_PORT:127.0.0.1:28766 -L 127.0.0.1:$VIEW_PORT:127.0.0.1:28767 "$H" > "$R/yam_tunnel.txt" 2>&1 < /dev/null &
echo $! > "$R/yam_tunnel.pid"; for i in $(seq 1 40); do ss -ltn | grep -q "127.0.0.1:$LINK_PORT " && break; sleep 0.25; done
( cd "$P/scripts" && exec python3 -u quest_pose_stream_sender.py --serial "$S" --port $LINK_PORT --max-rate 30 --source-log "$D/yam_src_$N.jsonl" \
    --summary "$D/yam_tx_$N.summary.json" > "$D/yam_tx_$N.console.txt" 2>&1 ) &
echo $! > "$R/yam_tx.pid"; sleep 2
msg "Starting the headset app ..."
adb -s "$S" shell am broadcast -a com.oculus.vrpowermanager.prox_close > /dev/null 2>&1   # keeps the headset awake while it is not worn
adb -s "$S" shell am force-stop org.questxarm.reader; sleep 1
adb -s "$S" shell am start -W -n org.questxarm.reader/com.rail.oculus.teleop.MainActivity > /dev/null 2>&1
python3 -u "$P/scripts/controller_wait.py" "$D/yam_src_$N.jsonl" --need 2 --serial "$S" || { msg "No controller. Ending."; finish; exit 3; }
msg "READY (simulation). Watch the arms at http://127.0.0.1:$VIEW_PORT/"
msg "Each hand: hold the side grip (middle finger) to move its arm, release to stop; the index trigger closes its gripper."
msg "Left controller -> left arm, right controller -> right arm (switch the left controller on too). Ctrl-C ends; Enter acknowledges a latched problem."
prev=""
while true; do
  if grep -q '^ENDING:\|^{"result"' "$D/yam_$N.console.txt" 2>/dev/null; then
    msg "The simulation ended by itself: $(sed -n 's/^ENDING: //p' "$D/yam_$N.console.txt" | head -1)"; finish; exit 0
  fi
  IFS= read -r -t 0.5 _ < /dev/tty 2> /dev/null; rc=$?
  if [ $rc = 0 ]; then ssh "${SSHO[@]}" "$H" "kill -USR1 \$(cat ~/$L/yam_pid_$N)" && msg "Acknowledged."
  elif [ $rc -lt 128 ]; then sleep 0.5; fi   # no terminal to read from
  line=$(grep 'YAM SIMULATION' "$D/yam_$N.console.txt" | tail -n 1 | sed 's/^ *[0-9.]*s YAM SIMULATION (no hardware) | //')
  now=$(echo "$line" | sed 's/ \[[^]]*\]//g')   # a line is shown when a hand's state changes, with its notes
  [ -n "$now" ] && [ "$now" != "$prev" ] && { msg "  $line"; prev=$now; }
done
