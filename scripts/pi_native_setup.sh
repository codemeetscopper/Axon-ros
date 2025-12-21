#!/usr/bin/env bash
set -euo pipefail

NON_INTERACTIVE="false"
if [[ "${1:-}" == "--non-interactive" ]]; then
  NON_INTERACTIVE="true"
fi

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (sudo)."
  exit 1
fi

apt-get update
apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-numpy \
  python3-opencv \
  alsa-utils \
  i2c-tools \
  espeak-ng

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init || true
fi
rosdep update || true

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ "$NON_INTERACTIVE" == "true" ]]; then
  "$SCRIPT_DIR/pi_enable_interfaces.sh" --non-interactive
else
  "$SCRIPT_DIR/pi_enable_interfaces.sh"
fi

echo "Pi native setup complete."
