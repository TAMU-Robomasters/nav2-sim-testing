#!/usr/bin/env bash
#
# autoboot_disable.sh — stop everything started by autoboot.sh: the waypoint
# patrol and the full Nav2 launch (and any lingering nav2/lidar/uart nodes).
#
# Usage:
#     ./autoboot_disable.sh
#
set -uo pipefail

RUN_DIR="/tmp/sentry_autoboot"
LAUNCH_PIDFILE="${RUN_DIR}/launch.pgid"
PATROL_PIDFILE="${RUN_DIR}/patrol.pgid"

log() { echo "[disable $(date '+%H:%M:%S')] $*"; }

# Send a signal to a whole process group identified by the PGID in $1's file.
# Escalates INT -> TERM -> KILL, waiting for the group to actually die.
stop_group() {
  local name="$1" pidfile="$2"
  if [[ ! -f "${pidfile}" ]]; then
    log "${name}: no pidfile, nothing to stop"
    return
  fi
  local pgid
  pgid="$(cat "${pidfile}")"
  if [[ -z "${pgid}" ]] || ! kill -0 "${pgid}" 2>/dev/null; then
    log "${name}: process group ${pgid:-?} not running"
    rm -f "${pidfile}"
    return
  fi

  log "${name}: stopping process group ${pgid}"
  for sig in INT TERM KILL; do
    kill -"${sig}" -"${pgid}" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "${pgid}" 2>/dev/null || break 2
      sleep 0.5
    done
    log "${name}: still alive after SIG${sig}, escalating"
  done

  if kill -0 "${pgid}" 2>/dev/null; then
    log "${name}: WARNING — process group ${pgid} survived SIGKILL"
  else
    log "${name}: stopped"
  fi
  rm -f "${pidfile}"
}

# Stop patrol first (it's the client), then the stack it talks to.
stop_group "patrol" "${PATROL_PIDFILE}"
stop_group "launch" "${LAUNCH_PIDFILE}"

# Fallback sweep: kill any nav2 / lidar / uart / patrol processes that outlived
# their launch (e.g. if autoboot was killed uncleanly and the pidfiles are gone).
log "sweeping for any leftover stack processes..."
PATTERNS=(
  "square_patrol.py"
  "real_lidar_navigate.launch.py"
  "nav2_"
  "ldlidar"
  "uart_odom"
  "laserscan_toolbox"
)
for pat in "${PATTERNS[@]}"; do
  pkill -INT -f "${pat}" 2>/dev/null || true
done
sleep 2
for pat in "${PATTERNS[@]}"; do
  pkill -KILL -f "${pat}" 2>/dev/null || true
done

log "done — stack and patrol stopped"
