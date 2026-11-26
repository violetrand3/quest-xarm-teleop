# Sourced by scripts/teleop and the scripts it starts, after the settings. LOCAL=1 when this machine IS the lab host
# (LAB_HOST is its own name, or TELEOP_LOCAL=1): the operator sits at the lab host with the headset plugged into it.
# Then nothing goes through SSH: `ssh` and `scp` below run the same commands here, in the lab account's home, and the
# tunnels are not made (the programs' own ports are used: local_ports). Run it as the account that owns ~/$LAB_DIR
# (dev), so the programs, their files and the arms' lock belong to the same account as in every other session.
LOCAL=0
if [ "${TELEOP_LOCAL:-}" = 1 ] || [ "$LAB_HOST" = "$(hostname -s 2> /dev/null)" ] || [ "$LAB_HOST" = "$(hostname 2> /dev/null)" ]; then LOCAL=1; fi
if [ "$LOCAL" = 1 ]; then
  [ -d "$HOME/$LAB_DIR/scripts" ] || { echo "STOP: this is the lab host, but $(id -un) has no ~/$LAB_DIR. Run it as the lab account (sudo -u dev -i), from ~/$LAB_DIR." >&2; exit 2; }
  ssh() {   # ssh [options] <host> [command ...]: the command, here. A call that only makes a tunnel (-N, or -f with -L) does nothing.
    local n=0 tunnel=0
    while [ $# -gt 0 ]; do
      case "$1" in
        -n) n=1; shift;;
        -N) tunnel=1; shift;;
        -L) tunnel=1; shift 2;;
        -o|-p|-i|-l) shift 2;;
        -*) shift;;
        *) break;;
      esac
    done
    shift   # the host
    [ "$tunnel" = 1 ] && return 0
    [ $# -gt 0 ] || { echo "local mode: an interactive ssh is not supported" >&2; return 2; }
    if [ "$n" = 1 ]; then (cd "$HOME" && bash -c "$*" < /dev/null); else (cd "$HOME" && bash -c "$*"); fi
  }
  scp() {   # scp [options] <file> <host>:<path relative to the home>
    while [ $# -gt 2 ]; do case "$1" in -o|-P|-i) shift 2;; *) shift;; esac; done
    cp "$1" "$HOME/${2#*:}"
  }
fi
local_ports() {   # the tunnels' ends on this PC are the programs' own ports when there are no tunnels
  [ "$LOCAL" = 1 ] || return 0
  RELAY_LOCAL_PORT=$RELAY_PORT; LINK_PORT=28766; VIEW_PORT=28767
}
