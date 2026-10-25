#!/usr/bin/env bash
#
# uninstall_service.sh — stop, disable and remove the `sentry_nav` systemd
# service installed by install_service.sh. Modeled on AutoX's utils/kill_onboot_cv.
#
#   ./uninstall_service.sh        (prompts for sudo)
#
SERVICE_NAME="sentry_nav"

echo "Stopping + disabling $SERVICE_NAME ..."
sudo systemctl stop    "$SERVICE_NAME" 2>/dev/null || true
sudo systemctl disable "$SERVICE_NAME" 2>/dev/null || true
sudo rm -f "/etc/systemd/system/${SERVICE_NAME}.service"
sudo systemctl daemon-reload
sudo systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true

# Backstop: systemd's cgroup teardown usually reaps everything, but sweep for any
# stack processes that outlived the unit (e.g. if it was killed uncleanly).
echo "Sweeping for leftover stack processes ..."
for _ in 1 2 3; do
  sudo pkill -TERM -f "square_patrol.py"              2>/dev/null || true
  sudo pkill -TERM -f "real_lidar_navigate.launch.py" 2>/dev/null || true
  sudo pkill -TERM -f "ldlidar"                       2>/dev/null || true
  sudo pkill -TERM -f "uart_odom"                     2>/dev/null || true
done

echo "Done. Autoboot service removed; it will NOT start on next boot."
