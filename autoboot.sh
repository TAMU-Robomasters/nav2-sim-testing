#!/usr/bin/env bash
#
# autoboot.sh — bring up the full Nav2 stack on the real robot, wait until it is
# ACTIVE, then start the waypoint patrol (square_patrol.py) that drives the
# sentry between points.
#
# This is fire-and-forget: it launches everything DETACHED (survives this
# terminal / suitable for rc.local / systemd), records PIDs, and exits once the
# patrol is running. Stop everything later with:
#
#     ./autoboot_disable.sh
#
# Usage:
#     ./autoboot.sh                    # run with the commands defined below
#     USE_RVIZ=true ./autoboot.sh      # env-override a default (see EDIT ME block)
#
set -euo pipefail

# ============================================================================
#  EDIT ME  ——  the ROS commands autoboot runs.
#
#  This is the AutoX-style "just open the script and change the command" block.
#  To add/remove a launch argument, edit LAUNCH_CMD below: add a line like
#      use_keepout:=false
#  or uncomment one of the examples. One arg per line, no trailing backslashes.
#
#  The ${VAR:-default} bits mean you can EITHER edit the value here directly, OR
#  override it for a single run from the environment, e.g.:
#      USE_RVIZ=true ./autoboot.sh
# ============================================================================
LAUNCH_CMD=(
  ros2 launch sam_bot_bringup real_lidar_navigate.launch.py
    use_rviz:="${USE_RVIZ:-false}"
    use_uart_odom:="${USE_UART_ODOM:-true}"
    enable_mcu_bridge:="${ENABLE_MCU_BRIDGE:-true}"   # ON by default: sentry co-runs AutoX (MCU=ROS) sharing this UART
    use_keepout:="${USE_KEEPOUT:-false}"   # keepout-zone costmap filter; default off
  # --- optional extras: uncomment / edit as needed ---------------------------
  # map:=/home/orin/repos/nav2-sim-testing/src/sam_bot_bringup/maps/3v3_field.yaml
)

# Client that drives the robot once the stack is ACTIVE. Edit args here too.
PATROL_CMD=(
  ros2 run sam_bot_bringup square_patrol.py
)

# Set PATROL to "false" (or run with RUN_PATROL=false) to bring up the stack but
# NOT start driving the robot — handy for testing the launch without motion.
RUN_PATROL="${RUN_PATROL:-true}"
# ============================================================================
#  end EDIT ME  ——  machinery below usually needs no changes.
# ============================================================================

# ---------- Paths ----------
WS_ROOT="/home/orin/repos/nav2-sim-testing"
ROS_SETUP="/opt/ros/humble/setup.bash"
OVERLAY_SETUP="${WS_ROOT}/install/setup.bash"

RUN_DIR="/tmp/sentry_autoboot"
LAUNCH_PIDFILE="${RUN_DIR}/launch.pgid"
PATROL_PIDFILE="${RUN_DIR}/patrol.pgid"
LAUNCH_LOG="${RUN_DIR}/launch.log"
PATROL_LOG="${RUN_DIR}/patrol.log"

ACTIVE_TIMEOUT="${ACTIVE_TIMEOUT:-120}"   # seconds to wait for the stack to go ACTIVE

# SERVICE_MODE=true is used by the systemd unit (install_service.sh): run the
# stack + patrol in the FOREGROUND as children of this script, then block. That
# keeps this process alive for as long as the stack runs, so systemd can track
# it (systemctl status), capture its output in the journal, and restart it on
# crash — and systemd's cgroup teardown kills the children on `systemctl stop`.
# Default false = the original detached fire-and-forget behavior.
SERVICE_MODE="${SERVICE_MODE:-false}"

# Lifecycle nodes that must reach "active" before we start patrolling.
REQUIRED_NODES=(
  /map_server
  /amcl
  /controller_server
  /planner_server
  /behavior_server
  /bt_navigator
)

log() { echo "[autoboot $(date '+%H:%M:%S')] $*"; }
err() { echo "[autoboot $(date '+%H:%M:%S')] ERROR: $*" >&2; }

# SERVICE_MODE teardown. Fires on EXIT/TERM/INT so Ctrl-C or `systemctl stop`
# (SIGTERM to us) tears the stack down. Kills our child process groups first,
# then sweeps the known stack processes because `ros2 launch` reparents some
# nodes out of our group. Under systemd this is belt-and-suspenders — its cgroup
# kill already reaps everything — but it keeps a manual run from orphaning nodes.
cleanup_service() {
  trap - EXIT TERM INT   # avoid re-entry
  log "SERVICE_MODE: stopping stack"
  # Fast path: signal the child process groups we own.
  [[ -n "${PATROL_PGID:-}" ]] && kill -TERM -"${PATROL_PGID}" 2>/dev/null || true
  [[ -n "${LAUNCH_PGID:-}" ]] && kill -TERM -"${LAUNCH_PGID}" 2>/dev/null || true
  # `ros2 launch` reparents some node children into their own sessions, so the
  # group-kill above can leave them behind. Under systemd this is moot (cgroup
  # kill reaps everything), but for a manual SERVICE_MODE run + Ctrl-C we sweep
  # the known stack processes so nothing is orphaned.
  local pat
  for pat in square_patrol.py real_lidar_navigate.launch.py nav2_ ldlidar uart_odom laserscan_toolbox; do
    pkill -TERM -f "${pat}" 2>/dev/null || true
  done
  sleep 2
  for pat in square_patrol.py real_lidar_navigate.launch.py nav2_ ldlidar uart_odom laserscan_toolbox; do
    pkill -KILL -f "${pat}" 2>/dev/null || true
  done
}

# Run an argv array DETACHED in its own process group, redirecting to a log.
# Echoes the new PGID on stdout. Re-sources ROS inside the detached shell so the
# child is self-contained; "$@" is passed through verbatim so args with spaces
# (or the ${VAR:-default} substitutions above) survive intact.
spawn_detached() {
  local logfile="$1"; shift
  setsid bash -c '
    source "'"${ROS_SETUP}"'"
    source "'"${OVERLAY_SETUP}"'"
    exec "$@"
  ' _ "$@" >"${logfile}" 2>&1 </dev/null &
  echo "$!"
}

# ---------- Sanity checks ----------
if [[ ! -f "${OVERLAY_SETUP}" ]]; then
  err "workspace overlay not found: ${OVERLAY_SETUP} — build first with 'colcon build --symlink-install'"
  exit 1
fi

mkdir -p "${RUN_DIR}"

# Refuse to double-start.
if [[ -f "${LAUNCH_PIDFILE}" ]] && kill -0 "$(cat "${LAUNCH_PIDFILE}")" 2>/dev/null; then
  err "a launch is already running (PGID $(cat "${LAUNCH_PIDFILE}")). Run ./autoboot_disable.sh first."
  exit 1
fi

# ---------- Source ROS ----------
# ROS/ament setup.bash files reference unbound vars (AMENT_TRACE_SETUP_FILES,
# etc.), so relax `nounset` while sourcing them, then restore it.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${OVERLAY_SETUP}"
set -u

# ---------- 1. Launch the Nav2 stack ----------
log "starting: ${LAUNCH_CMD[*]}"
if [[ "${SERVICE_MODE}" == "true" ]]; then
  # Foreground-managed: run the launch in its own process group (setsid) so the
  # trap can kill the whole group, but keep it a direct child so we can `wait` on
  # it. Output inherits our stdout → the systemd journal.
  trap cleanup_service EXIT TERM INT
  setsid "${LAUNCH_CMD[@]}" &
  LAUNCH_PGID=$!
  log "launch running as PGID ${LAUNCH_PGID} (SERVICE_MODE; output → journal)"
else
  LAUNCH_PGID="$(spawn_detached "${LAUNCH_LOG}" "${LAUNCH_CMD[@]}")"
  echo "${LAUNCH_PGID}" >"${LAUNCH_PIDFILE}"
  log "launch running as PGID ${LAUNCH_PGID} (log: ${LAUNCH_LOG})"
fi

# ---------- 2. Wait until the stack is ACTIVE ----------
log "waiting up to ${ACTIVE_TIMEOUT}s for the nav stack to become ACTIVE..."
deadline=$(( $(date +%s) + ACTIVE_TIMEOUT ))
while true; do
  # Bail out if the launch died on us.
  if ! kill -0 "${LAUNCH_PGID}" 2>/dev/null; then
    err "the launch process exited early — see ${LAUNCH_LOG}"
    exit 1
  fi

  all_active=true
  for node in "${REQUIRED_NODES[@]}"; do
    state="$(ros2 lifecycle get "${node}" 2>/dev/null || true)"
    if [[ "${state}" != active* ]]; then
      all_active=false
      break
    fi
  done

  if ${all_active}; then
    log "nav stack is ACTIVE (all required nodes up)"
    break
  fi

  if (( $(date +%s) >= deadline )); then
    err "timed out waiting for the stack to go ACTIVE. Last checked node '${node}' = '${state:-<not found>}'."
    err "leaving the launch running so you can inspect it; tail ${LAUNCH_LOG}"
    exit 1
  fi
  sleep 2
done

# ---------- 3. Start the waypoint patrol ----------
if [[ "${RUN_PATROL}" != "true" ]]; then
  log "RUN_PATROL=${RUN_PATROL} — stack is up but NOT starting the patrol. Robot will not move."
  if [[ "${SERVICE_MODE}" == "true" ]]; then
    log "SERVICE_MODE: blocking on the launch (systemd tracks this process)."
    wait "${LAUNCH_PGID}"
    err "the launch exited (rc=$?) — letting systemd restart the service."
    exit 1
  fi
  log "done — stop everything with ./autoboot_disable.sh"
  exit 0
fi

log "starting: ${PATROL_CMD[*]}"
if [[ "${SERVICE_MODE}" == "true" ]]; then
  setsid "${PATROL_CMD[@]}" &
  PATROL_PGID=$!
  log "patrol running as PGID ${PATROL_PGID} (SERVICE_MODE; output → journal)"
  log "sentry is patrolling — blocking on the launch (systemd tracks this process)."
  # Stay in the foreground for as long as the stack lives. If the launch dies,
  # exit non-zero so systemd restarts the whole service (cgroup teardown reaps
  # the orphaned patrol first).
  wait "${LAUNCH_PGID}"
  err "the launch exited (rc=$?) — letting systemd restart the service."
  exit 1
fi

PATROL_PGID="$(spawn_detached "${PATROL_LOG}" "${PATROL_CMD[@]}")"
echo "${PATROL_PGID}" >"${PATROL_PIDFILE}"
log "patrol running as PGID ${PATROL_PGID} (log: ${PATROL_LOG})"

log "done — sentry is patrolling. Stop everything with ./autoboot_disable.sh"
