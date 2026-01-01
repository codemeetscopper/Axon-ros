#!/usr/bin/env bash
set -euo pipefail

# Basic Raspberry Pi ROS 2 Jazzy install helper.
# Assumes Ubuntu 24.04 or Debian-based with ROS 2 Jazzy repo available.

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo ./scripts/pi_install.sh"
  exit 1
fi

apt-get update
apt-get install -y curl gnupg lsb-release

if ! grep -q "packages.ros.org" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
fi

apt-get update
apt-get install -y \
  ros-jazzy-ros-base \
  python3-colcon-common-extensions \
  python3-pip \
  python3-venv \
  python3-rosdep \
  alsa-utils

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep not installed"
  exit 1
fi

rosdep init || true
rosdep update

echo "ROS 2 Jazzy installed. Source /opt/ros/jazzy/setup.bash"
