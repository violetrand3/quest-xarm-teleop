#!/usr/bin/env bash
# rsync's "remote shell" at the lab host itself (scripts/ops/local_mode.sh): rsync calls it as `<this> <host> <command ...>`;
# the command is run here, as the lab account in its home.
shift
if [ "$(id -un)" = "${LAB_ACCOUNT:-dev}" ] && [ "${TELEOP_LOCAL_SUDO:-}" != 1 ]; then cd && exec "$@"; fi
exec sudo -n -u "${LAB_ACCOUNT:-dev}" -H -- bash -c 'cd; exec "$@"' _ "$@"
