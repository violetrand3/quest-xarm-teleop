#!/usr/bin/env bash
# One bimanual YAM session: both Quest controllers -> two YAM arms on the lab host (scripts/yam_live.py).
#   yam_session.sh <name>                      SIMULATED arms, no hardware (scripts/teleop yam-sim)
#   yam_session.sh <name> <confirmation file>  the REAL arms: THEY MOVE (scripts/teleop yam-try, after its questions)
#   yam_session.sh <name> <confirmation file> rec    the same, recording demos (scripts/teleop yam-record)
# Ctrl-C ends it: the lab program lowers both arms to rest first. Enter acknowledges a latched hand problem.
set -u
P=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd); . "$P/config/teleop_session.conf" || exit 2
H=$LAB_HOST; D=${TELEOP_DATA_DIR:-$P/data/sessions}; R=$P/run; N=$1; CONF=${2:-}; REC=$([ "${3:-}" = rec ] && echo 1 || echo 0); L=$LAB_DIR/logs
YAM_RECORD_CAMS=${YAM_RECORD_CAMS:-rs_color}   # camera-wall cameras saved with every demo (the RealSense sees both arms)
if [ -n "$CONF" ]; then KIND="real arms"; ARMS="--arms real --moves-real-arms"; else KIND="simulation, no hardware"; ARMS="--arms sim"; fi
LAB_PY=${YAM_PYTHON:-/home/dev/yam/i2rt/.venv/bin/python}   # I2RT's environment on the lab host
LINK_PORT=18766; VIEW_PORT=18767                               # this PC; the lab program listens on 28766 / 28767
# YAM_HEADSET=web: the cameras are shown IN the headset. Its browser opens a VR page served by this PC over the USB
# cable, and that page reads the controllers in place of the headset app (scripts/quest_web_source.py). app: as before.
case "${YAM_HEADSET:-app}" in app|web) ;; *) echo "YAM_HEADSET must be app or web (config/teleop_session.conf)"; exit 2;; esac
WEB=$([ "${YAM_HEADSET:-app}" = web ] && echo 1 || echo 0); WEB_PORT=${YAM_WEB_PORT:-8791}
mkdir -p "$D" "$R"
SSHO=(-n -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
S=$(adb devices | tr -d '\r' | awk 'NR>1 && $2=="device"{print $1}'); [ "$(echo "$S" | grep -c .)" = 1 ] || { echo "need exactly one headset on adb"; exit 2; }
msg() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$1"; }
for p in $LINK_PORT $VIEW_PORT $([ "$WEB" = 1 ] && echo $WEB_PORT); do ss -ltn | grep -q "127.0.0.1:$p " && { msg "Port $p on this PC is in use (another session?). Ending."; exit 2; }; done

finish() {   # $1 = notstarted: the lab program exited at start-up, so there is nothing to lower
  # Exactly once, and only in this script's main process (the traps below make sure of it): a second stop signal
  # while the arms are held (lowering could not finish) would let them go (they drop).
  [ -e "$R/yam_finishing_$N" ] && return
  : > "$R/yam_finishing_$N"
  if [ "${1:-}" = notstarted ]; then
    T=$(cat "$R/yam_tx.pid" 2>/dev/null); kill -INT "$T" 2>/dev/null
    T=$(cat "$R/yam_tunnel.pid" 2>/dev/null); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:$LINK_PORT:" && kill "$T"
    ssh "${SSHO[@]}" "$H" "ps -o args= -p \$(cat ~/$L/yam_pid_$N 2>/dev/null) 2>/dev/null | grep -q yam_live.py" \
      && msg "WARNING: the YAM program is still running on $H; look at the arms" || msg "The YAM program is not running; nothing was commanded."
    T=$(cat "$R/yam_tail.pid" 2>/dev/null); kill "$T" 2>/dev/null; adb -s "$S" shell am force-stop org.questxarm.reader > /dev/null 2>&1
    [ "$WEB" = 1 ] && adb -s "$S" reverse --remove tcp:$WEB_PORT > /dev/null 2>&1
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
  ssh "${SSHO[@]}" "$H" "for i in \$(seq 1 120); do grep -q '^{\"result\"\|^COULD NOT LOWER' ~/$L/yam_$N.console.txt && break; sleep 0.5; done; grep '^Both arms are at rest\|^COULD NOT LOWER\|^demos:\|^{\"result\"' ~/$L/yam_$N.console.txt" > "$D/yam_end_$N.txt" 2>&1
  if grep -q '"at_rest": true' "$D/yam_end_$N.txt"; then msg "Both arms are at rest and the program has ended."
  elif grep -q '^COULD NOT LOWER' "$D/yam_end_$N.txt"; then
    msg "WARNING: $(grep -m1 '^COULD NOT LOWER' "$D/yam_end_$N.txt" | cut -c1-300)"
    msg "The lab program is still running and holding the arms. Look at the arms; scripts/teleop stop lets them go (they drop), or use the e-stop."
  else msg "WARNING: the lab program did not confirm that the arms are at rest. Look at the arms; use the e-stop if they move."; fi
  T=$(cat "$R/yam_tail.pid" 2>/dev/null); kill "$T" 2>/dev/null
  adb -s "$S" shell am force-stop org.questxarm.reader > /dev/null 2>&1
  [ "$WEB" = 1 ] && adb -s "$S" reverse --remove tcp:$WEB_PORT > /dev/null 2>&1
  if [ "$REC" = 1 ]; then
    msg "$(grep -m1 '^demos:' "$D/yam_end_$N.txt" 2>/dev/null || ssh "${SSHO[@]}" "$H" "grep -m1 '^demos:' ~/$L/yam_$N.console.txt")"
    if [ "${MAKE_VIDEOS:-1}" = 1 ]; then
      msg "Making the videos on $H and copying the demos here ..."
      if bash "$P/scripts/ops/episodes_videos.sh" "$N" >> "$D/yam_videos_$N.txt" 2>&1; then msg "Demos and videos: $P/data/episodes/$N/"; else msg "The videos were not made; see $D/yam_videos_$N.txt"; fi
    fi
  fi
}

msg "Starting the YAM program on $H ($KIND) ..."
ssh "${SSHO[@]}" "$H" "test -x $LAB_PY" || { msg "I2RT's Python ($LAB_PY) is not on $H. Ending."; exit 2; }
if [ -n "$CONF" ]; then scp -q "${SSHO[@]:1}" "$CONF" "$H:$L/confirm_$N.txt" || { msg "cannot copy the confirmation to $H. Ending."; exit 2; }
  ARMS="$ARMS --human-confirmation \"\$(tr '\n\"' '  ' < logs/confirm_$N.txt)\""; fi
if [ "$REC" = 1 ]; then
  ARMS="$ARMS --record-dir episodes/$N --record-max-s $((${DEMO_MAX_MIN:-5} * 60)) --record-min-free-gb ${MIN_FREE_GB:-6}"
  for c in ${YAM_RECORD_CAMS//,/ }; do ARMS="$ARMS --record-cam $c=${LAB_CAMERA_STATUS%/api/status}/cam/$c"; done
fi
ssh "${SSHO[@]}" "$H" "cd ~/$LAB_DIR && mkdir -p logs && (nohup setsid $LAB_PY -u scripts/yam_live.py $ARMS --speed ${YAM_SPEED:-7.5} --scale ${YAM_SCALE:-0.667} $([ "${YAM_VELOCITY_FF:-1}" = 0 ] && echo --no-velocity-ff) --operator-yaw ${OPERATOR_YAW:-0} --log logs/yam_$N.jsonl > logs/yam_$N.console.txt 2>&1 < /dev/null & echo \$! > logs/yam_pid_$N)"
: > "$D/yam_$N.console.txt"
ssh "${SSHO[@]}" "$H" "tail -n +1 -F ~/$L/yam_$N.console.txt 2>/dev/null" > "$D/yam_$N.console.txt" 2>/dev/null &
echo $! > "$R/yam_tail.pid"
for i in $(seq 1 120); do grep -q "Listening on\|Traceback\|^error" "$D/yam_$N.console.txt" 2>/dev/null && break; sleep 0.25; done
if ! grep -q "Listening on" "$D/yam_$N.console.txt"; then
  msg "NOT STARTED. The YAM program says: $(grep -v 'INFO' "$D/yam_$N.console.txt" | grep '^[A-Za-z]*Error\|^error' | tail -1 | cut -c1-300)"
  finish notstarted; exit 3
fi
# Ctrl-C reaches every process of the session; only the main one ends it (a $(...) subshell just exits). During the
# loop below the trap only notes the stop and the loop runs the ending: a trap that runs while `read -t` waits is cut
# short by read's own timer, wherever it is (2026-11-18, twice: the ending stopped half way, the loop went on, and the
# tunnel and the console tail were left running; the lab program had lowered the arms to rest).
STOP=0; IN_LOOP=0
end_now() { trap "" INT TERM HUP PIPE; echo; msg "Ending the session." 2> /dev/null; finish 2> /dev/null; exit 130; }
on_stop() { [ "$BASHPID" = "$$" ] || exit 130; if [ "$IN_LOOP" = 1 ]; then STOP=1; else end_now; fi; }
trap on_stop INT TERM HUP PIPE
ssh "${SSHO[@]:1}" -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:$LINK_PORT:127.0.0.1:28766 -L 127.0.0.1:$VIEW_PORT:127.0.0.1:28767 "$H" > "$R/yam_tunnel.txt" 2>&1 < /dev/null &
echo $! > "$R/yam_tunnel.pid"; for i in $(seq 1 40); do ss -ltn | grep -q "127.0.0.1:$LINK_PORT " && break; sleep 0.25; done
SRC=(--serial "$S"); STATUS="$R/yam_status_$N.txt"; echo "starting" > "$STATUS"
[ "$WEB" = 1 ] && SRC=(--source web --web-port "$WEB_PORT" --web-cams "http://127.0.0.1:${RELAY_LOCAL_PORT:-18091}" --web-views "${YAM_WEB_VIEWS:-}" --web-big "${YAM_WEB_BIG:-}" --web-stereo "${YAM_WEB_STEREO:-}" --web-status "$STATUS")
( cd "$P/scripts" && exec python3 -u quest_pose_stream_sender.py "${SRC[@]}" --port $LINK_PORT --max-rate 30 --ack-timeout 8 --source-log "$D/yam_src_$N.jsonl" \
    --summary "$D/yam_tx_$N.summary.json" > "$D/yam_tx_$N.console.txt" 2>&1 ) &
echo $! > "$R/yam_tx.pid"; sleep 2
adb -s "$S" shell am force-stop org.questxarm.reader; sleep 1
if [ "$WEB" = 1 ]; then
  msg "Opening the headset's browser ..."
  adb -s "$S" reverse tcp:$WEB_PORT tcp:$WEB_PORT > /dev/null || { msg "The headset cannot reach this PC over USB (adb reverse). Ending."; finish; exit 3; }
  adb -s "$S" shell am start -a android.intent.action.VIEW -d "http://localhost:$WEB_PORT/" com.oculus.browser > /dev/null 2>&1 \
    || adb -s "$S" shell am start -n com.oculus.vrshell/.MainActivity -d apk://com.oculus.browser -e uri "http://localhost:$WEB_PORT/" > /dev/null 2>&1
  msg "PUT THE HEADSET ON. Its browser shows \"Teleop headset view\" (if not: open the Browser and go to localhost:$WEB_PORT)."
  msg "Point a controller at \"Start (see the room)\" and press its trigger. The cameras then float in front of you."
else
  msg "Starting the headset app ..."
  adb -s "$S" shell am broadcast -a com.oculus.vrpowermanager.prox_close > /dev/null 2>&1   # keeps the headset awake while it is not worn
  adb -s "$S" shell am start -W -n org.questxarm.reader/com.rail.oculus.teleop.MainActivity > /dev/null 2>&1
fi
python3 -u "$P/scripts/controller_wait.py" "$D/yam_src_$N.jsonl" --need 2 --serial "$S" $([ "$WEB" = 1 ] && echo --web) || { msg "No controller. Ending."; finish; exit 3; }
sleep 4; ssh "${SSHO[@]}" "$H" "kill -USR1 \$(cat ~/$L/yam_pid_$N)"   # the app's start-up reference notices, once (as on the xArm)
msg "READY ($KIND). Cameras: http://127.0.0.1:${RELAY_LOCAL_PORT:-18091}/   3D view of what the arms are commanded to do: http://127.0.0.1:$VIEW_PORT/"
msg "Each hand: hold the side grip (middle finger) to move its arm, release to stop; while the grip is held, the index trigger closes its gripper (as on the xArm)."
msg "Left controller -> left arm, right controller -> right arm (switch the left controller on too). Ctrl-C ends; Enter acknowledges a latched problem."
[ "$REC" = 1 ] && msg "Demos: right thumb A starts one and, pressed again, saves it; B discards the one in progress."
prev=""; seen_ep=0; stopped=0; TTY=0; { : < /dev/tty; } 2> /dev/null && TTY=1   # Enter is read only when there is a terminal
IN_LOOP=1
while true; do
  [ "$STOP" = 1 ] && end_now
  if grep -q '^ENDING:\|^{"result"' "$D/yam_$N.console.txt" 2>/dev/null; then
    msg "The YAM program is ending by itself: $(sed -n 's/^ENDING: //p' "$D/yam_$N.console.txt" | head -1)"; finish; exit 0
  fi
  if [ "$TTY" = 1 ]; then
    IFS= read -r -t 0.5 _ < /dev/tty && [ "$STOP" = 0 ] && { ssh "${SSHO[@]}" "$H" "kill -USR1 \$(cat ~/$L/yam_pid_$N)" && msg "Acknowledged."; }
    [ "$STOP" = 1 ] && end_now
  else sleep 0.5; fi
  line=$(grep ' YAM \(SIMULATION\|REAL ARMS\)' "$D/yam_$N.console.txt" | tail -n 1 | sed 's/^ *[0-9.]*s YAM [^|]*| //')
  now=$(echo "$line" | sed 's/ \[[^]]*\]//g; s/ | ep=.*//')   # a line is shown when a hand's state changes, with its notes
  if [ "$REC" = 1 ]; then   # the recorder's own messages: started, SAVED, DISCARDED, NOT started
    eps=$(grep '^episode \|^EPISODE' "$D/yam_$N.console.txt"); k=$(printf '%s' "$eps" | grep -c .)
    [ "$k" -gt "$seen_ep" ] && { printf '%s\n' "$eps" | tail -n $((k - seen_ep)) | while IFS= read -r e; do msg "DEMO: $e"; done; seen_ep=$k; }
  fi
  [ -n "$now" ] && [ "$now" != "$prev" ] && { msg "  $line"; prev=$now; }
  [ "$WEB" = 1 ] && [ -n "$line" ] && printf '%s\n' "$line" > "$STATUS.new" && mv "$STATUS.new" "$STATUS"   # shown in the headset
  # A latched stop: the hands' state says "fault" until Enter. Said once, plainly (the status line is long; on
  # 2026-11-19 the operator did not see "Enter clears it" at its end and kept pressing the grip).
  if printf '%s' "$now" | grep -q ' fault'; then
    [ "$stopped" = 0 ] && { msg "STOPPED: both arms are holding where they are (the reason is in [...] above). The grip does not restart them."
                            msg "         Look at the arms; when it is safe, press Enter here to continue."; stopped=1; }
  else stopped=0; fi
done
