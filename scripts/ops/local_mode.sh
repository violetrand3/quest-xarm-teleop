# Sourced by scripts/teleop and the scripts it starts, after the settings. LOCAL=1 when this machine IS the lab host
# (LAB_HOST is its own name, or TELEOP_LOCAL=1): the operator sits at the lab host with the headset plugged into it.
# Then nothing goes through SSH: `ssh` and `scp` below run the same commands here, as the lab account (LAB_ACCOUNT,
# dev) in its home, and the tunnels are not made (the programs' own ports are used: local_ports).
#   - run by the lab account itself: directly;
#   - run by another account of the machine (its own checkout, its own adb): through `sudo -n -u <lab account>`, which
#     must work without a password (one line in /etc/sudoers.d; docs/getting_started.md). So the lab programs, their
#     files and the arms' lock belong to the lab account, as in every session started from an operator's PC.
LOCAL=0; export LAB_ACCOUNT=${LAB_ACCOUNT:-dev}
if [ "${TELEOP_LOCAL:-}" = 1 ] || [ "$LAB_HOST" = "$(hostname -s 2> /dev/null)" ] || [ "$LAB_HOST" = "$(hostname 2> /dev/null)" ]; then LOCAL=1; fi
if [ "$LOCAL" = 1 ]; then
  LAB_HOME=$(getent passwd "$LAB_ACCOUNT" | cut -d: -f6)
  as_lab() {   # a command, as the lab account with its HOME
    if [ "$(id -un)" = "$LAB_ACCOUNT" ] && [ "${TELEOP_LOCAL_SUDO:-}" != 1 ]; then "$@"; else sudo -n -u "$LAB_ACCOUNT" -H -- "$@"; fi
  }
  if [ "$(id -un)" != "$LAB_ACCOUNT" ] || [ "${TELEOP_LOCAL_SUDO:-}" = 1 ]; then
    sudo -n -u "$LAB_ACCOUNT" -H -- true 2> /dev/null || { echo "STOP: this is the lab host and you are $(id -un): the lab programs run as $LAB_ACCOUNT, through sudo without a password.
Set that up once (it asks for your sudo password):
  echo '$(id -un) ALL=($LAB_ACCOUNT) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/teleop-$(id -un)-as-$LAB_ACCOUNT > /dev/null && sudo chmod 440 /etc/sudoers.d/teleop-$(id -un)-as-$LAB_ACCOUNT && sudo visudo -c" >&2; exit 2; }
  fi
  as_lab test -d "$LAB_HOME/$LAB_DIR/scripts" || { echo "STOP: this is the lab host, but $LAB_ACCOUNT has no ~/$LAB_DIR (the installed copy of the programs)." >&2; exit 2; }
  ssh() {   # ssh [options] <host> [command ...]: the command, here. A call that only makes a tunnel (-N or -L) does nothing.
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
    if [ "$n" = 1 ]; then as_lab bash -c "cd; $*" < /dev/null; else as_lab bash -c "cd; $*"; fi
  }
  scp() {   # scp [options] <file> <host>:<path relative to the lab account's home>
    while [ $# -gt 2 ]; do case "$1" in -o|-P|-i) shift 2;; *) shift;; esac; done
    as_lab bash -c 'cd; cat > "$1"' _ "${2#*:}" < "$1"
  }
fi
local_ports() {   # the tunnels' ends on this PC are the programs' own ports when there are no tunnels
  [ "$LOCAL" = 1 ] || return 0
  RELAY_LOCAL_PORT=$RELAY_PORT; LINK_PORT=28766; VIEW_PORT=28767
}
