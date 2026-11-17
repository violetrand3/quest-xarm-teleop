#!/usr/bin/env bash
# One bimanual YAM session: both Quest controllers -> two YAM arms on the lab host (scripts/yam_live.py).
#   yam_session.sh <name>                      SIMULATED arms, no hardware (scripts/teleop yam-sim)
#   yam_session.sh <name> <confirmation file>  the REAL arms: THEY MOVE (scripts/teleop yam-try, after its questions)
# Ctrl-C ends it: the lab program lowers both arms to rest first. Enter acknowledges a latched hand problem.
set -u
P=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd); . "$P/config/teleop_session.conf" || exit 2
H=$LAB_HOST; D=${TELEOP_DATA_DIR:-$P/data/sessions}; R=$P/run; N=$1; CONF=${2:-}; L=$LAB_DIR/logs
if [ -n "$CONF" ]; then KIND="real arms"; ARMS="--arms real --moves-real-arms"; else KIND="simulation, no hardware"; ARMS="--arms sim"; fi
LAB_PY=${YAM_PYTHON:-/home/dev/yam/i2rt/.venv/bin/python}   # I2RT's environment on the lab host
LINK_PORT=18766; VIEW_PORT=18767                               # this PC; the lab program listens on 28766 / 28767
mkdir -p "$D" "$R"
SSHO=(-n -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
S=$(adb devices | tr -d '\r' | awk 'NR>1 && $2=="device"{print $1}'); [ "$(echo "$S" | grep -c .)" = 1 ] || { echo "need exactly one headset on adb"; exit 2; }
msg() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$1"; }
for p in $LINK_PORT $VIEW_PORT; do ss -ltn | grep -q "127.0.0.1:$p " && { msg "Port $p on this PC is in use (another session?). Ending."; exit 2; }; done

finish() {   # $1 = notstarted: the lab program exited at start-up, so there is nothing to lower
  if [ "${1:-}" = notstarted ]; then
    T=$(cat "$R/yam_tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null
    T=$(cat "$R/yam_tunnel.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:$LINK_PORT:" && kill "$T"
    ssh "${SSHO[@]}" "$H" "ps -o args= -p \$(cat ~/$L/yam_pid_$N 2>/dev/null) 2>/dev/null | grep -q yam_live.py" \
      && msg "WARNING: the YAM program is still running on $H; look at the arms" || msg "The YAM program is not running; nothing was commanded."
    T=$(cat "$R/yam_tail.pid" 2>/dev/null); kill "$T" 2>/dev/null; adb -s "$S" shell am force-stop org.questxarm.reader > /dev/null 2>&1
    return
  fi
  msg "Lowering both arms to rest and ending ..."
  # One stop signal, and none if the program is already ending by itself: a second one, while the arms are held
  # because lowering could not finish, lets them go (they drop). That stays the operator's decision (scripts/teleop stop).
  # Sent before the sender stops, so the lab program ends on the operator's stop, not on a lost connection.
  ssh "${SSHO[@]}" "$H" "grep -q '^ENDING:' ~/$L/yam_$N.console.txt || kill -INT \$(cat ~/$L/yam_pid_$N) 2>/dev/null"
  T=$(cat "$R/yam_tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null; for i in $(seq 1 40); do kill -0 "$T" 2>/dev/null || break; sleep 0.25; done
  sed -i "s/$S/[REDACTED]/g" "$D/yam_tx_$N.console.txt" 2>/dev/null
  T=$(cat "$R/yam_tunnel.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:$LINK_PORT:" && kill "$T"
  ssh "${SSHO[@]}" "$H" "for i in \$(seq 1 120); do grep -q '^{\"result\"\|^COULD NOT LOWER' ~/$L/yam_$N.console.txt && break; sleep 0.5; done; grep '^Both arms are at rest\|^COULD NOT LOWER\|^{\"result\"' ~/$L/yam_$N.console.txt" > "$D/yam_end_$N.txt" 2>&1
  if grep -q '"at_rest": true' "$D/yam_end_$N.txt"; then msg "Both arms are at rest and the program has ended."
  elif grep -q '^COULD NOT LOWER' "$D/yam_end_$N.txt"; then
    msg "WARNING: $(grep -m1 '^COULD NOT LOWER' "$D/yam_end_$N.txt" | cut -c1-300)"
    msg "The lab program is still running and holding the arms. Look at the arms; scripts/teleop stop lets them go (they drop), or use the e-stop."
  else msg "WARNING: the lab program did not confirm that the arms are at rest. Look at the arms; use the e-stop if they move."; fi
  T=$(cat "$R/yam_tail.pid" 2>/dev/null); kill "$T" 2>/dev/null
  adb -s "$S" shell am force-stop org.questxarm.reader > /dev/null 2>&1
}

msg "Starting the YAM program on $H ($KIND) ..."
ssh "${SSHO[@]}" "$H" "test -x $LAB_PY" || { msg "I2RT's Python ($LAB_PY) is not on $H. Ending."; exit 2; }
if [ -n "$CONF" ]; then scp -q "${SSHO[@]:1}" "$CONF" "$H:$L/confirm_$N.txt" || { msg "cannot copy the confirmation to $H. Ending."; exit 2; }
  ARMS="$ARMS --human-confirmation \"\$(tr '\n\"' '  ' < logs/confirm_$N.txt)\""; fi
ssh "${SSHO[@]}" "$H" "cd ~/$LAB_DIR && mkdir -p logs && (nohup setsid $LAB_PY -u scripts/yam_live.py $ARMS --operator-yaw ${OPERATOR_YAW:-0} --log logs/yam_$N.jsonl > logs/yam_$N.console.txt 2>&1 < /dev/null & echo \$! > logs/yam_pid_$N)"
: > "$D/yam_$N.console.txt"
ssh "${SSHO[@]}" "$H" "tail -n +1 -F ~/$L/yam_$N.console.txt 2>/dev/null" > "$D/yam_$N.console.txt" 2>/dev/null &
echo $! > "$R/yam_tail.pid"
for i in $(seq 1 120); do grep -q "Listening on\|Traceback\|^error" "$D/yam_$N.console.txt" 2>/dev/null && break; sleep 0.25; done
if ! grep -q "Listening on" "$D/yam_$N.console.txt"; then
  msg "NOT STARTED. The YAM program says: $(grep -v 'INFO' "$D/yam_$N.console.txt" | grep '^[A-Za-z]*Error\|^error' | tail -1 | cut -c1-300)"
  finish notstarted; exit 3
fi
trap 'trap "" INT TERM HUP PIPE; echo; msg "Ending the session." 2> /dev/null; finish 2> /dev/null; exit 130' INT TERM HUP PIPE
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
sleep 4; ssh "${SSHO[@]}" "$H" "kill -USR1 \$(cat ~/$L/yam_pid_$N)"   # the app's start-up reference notices, once (as on the xArm)
msg "READY ($KIND). Cameras: http://127.0.0.1:${RELAY_LOCAL_PORT:-18091}/   3D view of what the arms are commanded to do: http://127.0.0.1:$VIEW_PORT/"
msg "Each hand: hold the side grip (middle finger) to move its arm, release to stop; while the grip is held, the index trigger closes its gripper (as on the xArm)."
msg "Left controller -> left arm, right controller -> right arm (switch the left controller on too). Ctrl-C ends; Enter acknowledges a latched problem."
prev=""; TTY=0; { : < /dev/tty; } 2> /dev/null && TTY=1   # Enter is read only when there is a terminal
while true; do
  if grep -q '^ENDING:\|^{"result"' "$D/yam_$N.console.txt" 2>/dev/null; then
    msg "The YAM program is ending by itself: $(sed -n 's/^ENDING: //p' "$D/yam_$N.console.txt" | head -1)"; finish; exit 0
  fi
  if [ "$TTY" = 1 ]; then
    IFS= read -r -t 0.5 _ < /dev/tty && { ssh "${SSHO[@]}" "$H" "kill -USR1 \$(cat ~/$L/yam_pid_$N)" && msg "Acknowledged."; }
  else sleep 0.5; fi
  line=$(grep ' YAM \(SIMULATION\|REAL ARMS\)' "$D/yam_$N.console.txt" | tail -n 1 | sed 's/^ *[0-9.]*s YAM [^|]*| //')
  now=$(echo "$line" | sed 's/ \[[^]]*\]//g')   # a line is shown when a hand's state changes, with its notes
  [ -n "$now" ] && [ "$now" != "$prev" ] && { msg "  $line"; prev=$now; }
done
