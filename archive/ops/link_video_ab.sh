#!/usr/bin/env bash
# A/B measurement: does camera video over the same path disturb the Quest control link?
# Fake robot only (teleop_dryrun on linux2). Cameras: view-only GET of the existing camera wall.
# Usage: link_video_ab.sh <name> <seconds-per-phase>
set -u
P=~/quest-xarm-teleop; D=$P/data/checkpoint9; R=$P/run; N=$1; SEC=${2:-20}
SSHO=(-o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=10)
S=$(adb devices | awk 'NR>1 && $2=="device"{print $1}')
mark() { python3 -c 'import time,sys;print(time.monotonic_ns(), sys.argv[1])' "$1" >> "$D/marks_$N.txt"; echo "MARK $1"; }
consume() { # $1 = label, rest = camera keys; reads the streams for $SEC seconds and reports bytes/s
python3 - "$SEC" "$D/video_$N.jsonl" "$@" <<'PY'
import sys,time,threading,urllib.request,json
sec=float(sys.argv[1]); out=sys.argv[2]; label=sys.argv[3]; keys=sys.argv[4:]; res={}
def pull(k):
    n=b=0; t0=time.monotonic()
    try:
        r=urllib.request.urlopen(f"http://127.0.0.1:18090/cam/{k}",timeout=10)
        while time.monotonic()-t0<sec:
            d=r.read(65536)
            if not d: break
            b+=len(d); n+=d.count(b"--frame")
        r.close()
    except Exception as e: res[k]={"error":str(e)}; return
    dt=time.monotonic()-t0; res[k]={"kB_s":round(b/dt/1000),"fps":round(n/dt,1)}
th=[threading.Thread(target=pull,args=(k,)) for k in keys]
[t.start() for t in th]
if not keys: time.sleep(sec)
[t.join() for t in th]
rec={"phase":label,"feeds":res,"total_kB_s":sum(v.get("kB_s",0) for v in res.values())}
open(out,"a").write(json.dumps(rec)+"\n"); print(json.dumps(rec))
PY
}
ssh "${SSHO[@]}" linux2 "cd ~/quest-xarm-teleop-dryrun/scripts && exec python3 -u teleop_dryrun.py --config ../config/teleop_dryrun_example.json --port 28765 --duration 300 --link-log ../logs/ab_link_$N.jsonl --summary ../logs/ab_$N.summary.json" > "$D/ab_$N.console.txt" 2>&1 &
echo $! > "$R/ab.pid"
for i in $(seq 1 60); do grep -q "DRY RUN" "$D/ab_$N.console.txt" 2>/dev/null && break; sleep 0.25; done
ssh "${SSHO[@]}" -N -o ExitOnForwardFailure=yes -L 127.0.0.1:18765:127.0.0.1:28765 linux2 > "$R/tunnel_link.txt" 2>&1 & echo $! > "$R/tunnel_link.pid"
ssh "${SSHO[@]}" -N -o ExitOnForwardFailure=yes -L 127.0.0.1:18090:127.0.0.1:8090 linux2 > "$R/tunnel_cam.txt" 2>&1 & echo $! > "$R/tunnel_cam.pid"
for i in $(seq 1 40); do ss -ltn | grep -q '127.0.0.1:18765 ' && ss -ltn | grep -q '127.0.0.1:18090 ' && break; sleep 0.25; done
( cd "$P/scripts" && exec python3 -u quest_pose_stream_sender.py --serial "$S" --port 18765 --max-rate 30 --log "$D/tx_$N.jsonl" --summary "$D/tx_$N.summary.json" > "$D/tx_$N.console.txt" 2>&1 ) & echo $! > "$R/tx.pid"
sleep 6
mark "A none";            consume "A none"
mark "B wrist+zed";       consume "B wrist+zed" iphone zed
mark "C none";            consume "C none"
mark "D all five";        consume "D all five" iphone zed c920 rs_color rs_depth
mark "E none";            consume "E none"
mark "end"
T=$(cat "$R/tx.pid"); kill -INT "$T" 2>/dev/null; for i in $(seq 1 40); do kill -0 "$T" 2>/dev/null || break; sleep 0.25; done
[ -n "$S" ] && sed -i "s/$S/[REDACTED]/g" "$D/tx_$N.console.txt"
for f in tunnel_link tunnel_cam; do T=$(cat "$R/$f.pid"); ps -o args= -p "$T" 2>/dev/null | grep -q -- "-L 127.0.0.1:18" && kill "$T"; done
ssh "${SSHO[@]}" linux2 "pkill -INT -f '[t]eleop_dryrun.py --config'"; G=$(cat "$R/ab.pid"); for i in $(seq 1 40); do kill -0 "$G" 2>/dev/null || break; sleep 0.5; done
scp -q "${SSHO[@]}" "linux2:quest-xarm-teleop-dryrun/logs/ab_link_$N.jsonl" "linux2:quest-xarm-teleop-dryrun/logs/ab_$N.summary.json" "$D/" && echo fetched
