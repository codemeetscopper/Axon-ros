#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo ./scripts/pi_setup_service.sh"
  exit 1
fi

SERVICE_SRC="/opt/axon/scripts/axon_ros.service"
SERVICE_DST="/etc/systemd/system/axon-ros.service"

if [[ ! -f "$SERVICE_SRC" ]]; then
  echo "Expected $SERVICE_SRC. Copy this repo to /opt/axon first."
  exit 1
fi

cp "$SERVICE_SRC" "$SERVICE_DST"
systemctl daemon-reload
systemctl enable axon-ros.service
systemctl restart axon-ros.service

systemctl status axon-ros.service --no-pager
