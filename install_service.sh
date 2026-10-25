#!/usr/bin/env bash
#
# install_service.sh — install + enable the `sentry_nav` systemd service so the
# full Nav2 stack + waypoint patrol comes up on boot (and restarts on crash).
# Modeled on AutoX's utils/setup_boot_script.sh.
#
# The service just runs `autoboot.sh` with SERVICE_MODE=true (foreground), so the
# command it launches is still the editable EDIT ME block at the top of
# autoboot.sh — change args there, then `sudo systemctl restart sentry_nav`.
#
#   Install / enable:   ./install_service.sh          (prompts for sudo)
#   Check it:           systemctl status sentry_nav
#   Follow logs:        journalctl -u sentry_nav -f
#   Remove:             ./uninstall_service.sh
#
# Safe to re-run: it just re-installs the unit with current paths.
set -euo pipefail

SERVICE_NAME="sentry_nav"
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
AUTOBOOT="$REPO_DIR/autoboot.sh"

# Run the stack as the invoking (non-root) user — that's who is in the `dialout`
# group for /dev/ttyTHS1 and owns the built overlay.
RUN_USER="${SUDO_USER:-$USER}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"

# Optional knobs baked into the unit (env-overridable at install time), e.g.:
#   RUN_PATROL=false ./install_service.sh   # boot the stack but don't drive
USE_RVIZ="${USE_RVIZ:-false}"
USE_UART_ODOM="${USE_UART_ODOM:-true}"
ENABLE_MCU_BRIDGE="${ENABLE_MCU_BRIDGE:-true}"   # ON by default: sentry co-runs AutoX (MCU=ROS) sharing this UART
USE_KEEPOUT="${USE_KEEPOUT:-false}"
RUN_PATROL="${RUN_PATROL:-true}"

# ---------- sanity checks ----------
[ -f "$AUTOBOOT" ]                || { echo "ERROR: $AUTOBOOT not found"; exit 1; }
[ -f "$REPO_DIR/install/setup.bash" ] || { echo "ERROR: workspace not built ($REPO_DIR/install/setup.bash missing) — run 'colcon build --symlink-install' first"; exit 1; }
chmod +x "$AUTOBOOT"

UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
echo "Installing $UNIT_PATH"
echo "  user: $RUN_USER   repo: $REPO_DIR"
echo "  use_rviz=$USE_RVIZ use_uart_odom=$USE_UART_ODOM enable_mcu_bridge=$ENABLE_MCU_BRIDGE use_keepout=$USE_KEEPOUT run_patrol=$RUN_PATROL"

sudo tee "$UNIT_PATH" >/dev/null <<EOF
[Unit]
Description=Sentry Nav2 autoboot (full stack + patrol)
# Keep this After=network.target only. Do NOT add graphical.target: it is ordered
# after multi-user.target (our WantedBy), which creates an ordering cycle that
# makes systemd silently drop the start job at boot.
After=network.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$REPO_DIR
Environment=HOME=$RUN_HOME
Environment=PATH=$RUN_HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin
Environment=SERVICE_MODE=true
Environment=USE_RVIZ=$USE_RVIZ
Environment=USE_UART_ODOM=$USE_UART_ODOM
Environment=ENABLE_MCU_BRIDGE=$ENABLE_MCU_BRIDGE
Environment=USE_KEEPOUT=$USE_KEEPOUT
Environment=RUN_PATROL=$RUN_PATROL
ExecStart=/usr/bin/env bash $AUTOBOOT
Restart=always
RestartSec=3
# Give the stack time to come up before systemd considers a slow start a failure.
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

cat <<EOF

Done. The sentry nav stack will now start on boot.
  status:   systemctl status $SERVICE_NAME
  is-up:    systemctl is-active $SERVICE_NAME
  logs:     journalctl -u $SERVICE_NAME -f
  restart:  sudo systemctl restart $SERVICE_NAME   (after editing autoboot.sh args)
  remove:   $SCRIPT_DIR/uninstall_service.sh
EOF
