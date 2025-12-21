#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list | grep -q "/axon/system/power_status"
ros2 topic list | grep -q "/axon/system/audio_status"

ros2 service list | grep -q "/axon/system/set_audio_controls"
ros2 service list | grep -q "/axon/actuators/pwm/set_batch"

echo "Distributed verification basic topics/services present"
