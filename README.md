# Axon ROS 2 Workspace

This repository contains a modular ROS 2 workspace for the **Axon** robot. It is designed for Raspberry Pi deployment with a matching development profile for laptops.

## Architecture Overview
The workspace follows a multi-package layout with strict separation of concerns and “thin nodes, thick libraries.” ROS 2 nodes focus on wiring, while reusable logic lives in plain Python modules.

Key principles:
- Explicit dependencies with no circular imports.
- Namespace-aware launch files for multi-robot support.
- Runtime configuration via YAML + dataclass validation.
- Consistent logging and diagnostics hooks.

## Package Map
| Package | Purpose |
| --- | --- |
| `axon_interfaces` | Custom messages (audio, base status). |
| `axon_bringup` | Launch files and parameter profiles. |
| `axon_description` | URDF/xacro description and frame conventions. |
| `axon_utils` | Shared helpers (config, retry, logging, ROS utils). |
| `axon_base_driver` | Serial chassis driver (diff-drive). |
| `axon_audio_driver` | Microphone and speaker nodes. |
| `axon_hri_voice` | Voice pipeline skeleton (STT/TTS stubs). |
| `axon_tcp_bridge` | TCP bridge architecture placeholder. |
| `axon_tests` | Unit tests for shared libraries. |

## Build & Run
From the repository root:

```bash
cd axon_ws
colcon build --symlink-install
source install/setup.bash
```

Launch (dev profile):

```bash
ros2 launch axon_bringup robot.launch.py profile:=dev
```

## Raspberry Pi Setup (Beginner-Friendly)
These steps install ROS 2 Jazzy, build the workspace, and configure the robot to start on boot.

### 1) Install ROS 2 Jazzy
On Raspberry Pi OS (Ubuntu/Debian-based), run:

```bash
./scripts/pi_install.sh
```

This script installs ROS 2 Jazzy, colcon, Python tools, and ALSA utilities for audio.

### 2) Build the workspace
```bash
cd /opt/axon
colcon build --symlink-install
```

### 3) Enable on-boot startup
Install and enable the systemd service:

```bash
sudo ./scripts/pi_setup_service.sh
```

This registers a service that launches:
```bash
ros2 launch axon_bringup robot.launch.py profile:=real
```

Use these commands to manage it:
```bash
sudo systemctl status axon-ros.service
sudo systemctl restart axon-ros.service
sudo journalctl -u axon-ros.service -f
```

## Remote Laptop Control (Yes, supported)
You can connect a laptop to the Axon robot over the network using ROS 2 DDS discovery.

### Requirements
1. Robot and laptop must be on the same network (Wi-Fi or Ethernet).
2. Same ROS 2 distro (Jazzy recommended).
3. Same `ROS_DOMAIN_ID`.

### On the Raspberry Pi (robot)
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
source /opt/axon/axon_ws/install/setup.bash
ros2 launch axon_bringup robot.launch.py profile:=real
```

### On the laptop (dev machine)
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
source ~/axon_ws/install/setup.bash  # or your local workspace path
ros2 topic list
ros2 topic echo /base/status
```

### Example: Send a command from the laptop
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

If discovery does not work, check firewall rules and confirm both machines are on the same subnet.

### Quick Tests
Publish a velocity command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Send a text-to-speech message:

```bash
ros2 topic pub /hri/tts/text std_msgs/msg/String "{data: 'hello axon'}"
```

## Topics & Services
| Topic | Type | Direction | Description |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Base velocity command. |
| `/odom` | `nav_msgs/Odometry` | Pub | Odometry (dummy or wheel/external). |
| `/base/status` | `axon_interfaces/BaseStatus` | Pub | Base driver status. |
| `/audio/mic/pcm` | `axon_interfaces/AudioChunk` | Pub | Microphone audio frames. |
| `/audio/speaker/pcm` | `axon_interfaces/AudioChunk` | Sub | Speaker playback frames. |
| `/hri/stt/text` | `std_msgs/String` | Pub | Speech-to-text stub output. |
| `/hri/tts/text` | `std_msgs/String` | Sub | Text-to-speech stub input. |

## Parameters
Parameters are stored under `axon_bringup/config/` and validated at runtime via dataclasses in `axon_utils`.
- Base driver parameters: `axon_bringup/config/base.yaml`
- Audio driver parameters: `axon_bringup/config/audio.yaml`
- HRI parameters: `axon_bringup/config/hri.yaml`
- Profile selection: `axon_bringup/config/profiles/{real,dev,sim}.yaml`

## Testing
```bash
./scripts/format.sh
./scripts/lint.sh
./scripts/test.sh
```

For additional design details, see [ARCHITECTURE.md](ARCHITECTURE.md).
