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
