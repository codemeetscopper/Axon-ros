#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--non-interactive" ]]; then
  sudo raspi-config nonint do_i2c 0
else
  sudo raspi-config
fi

if [[ -e /dev/i2c-1 ]]; then
  echo "I2C enabled: /dev/i2c-1 present"
else
  echo "I2C device not found; reboot required"
fi
