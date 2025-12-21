# Axon ROS 2 Distributed Robot Platform

This repository implements the Axon distributed ROS 2 system with a strict allowlist gateway between a Raspberry Pi (ROS 2 Humble, Domain 42) and a laptop (ROS 2 Jazzy, Domain 43). The Pi remains safe and functional without the laptop and provides production-ready mock backends for power, audio, and PWM hardware integration.

## Architecture

- **Domain A (Pi / ROS 2 Humble / Domain 42)**
  - Hardware and safety nodes: power, audio, PWM, system diagnostics, watchdog, and mode manager.
- **Domain B (Laptop / ROS 2 Jazzy / Domain 43)**
  - Operator console UI, teleop/perception (extensible), and allowlist gateway.
- **Gateway (Laptop)**
  - Explicit allowlist for cross-domain topics/services.

## Wiring & Device Expectations

- Camera: `/dev/video0`
- USB soundcard: `/dev/snd`
- PWM/I2C board: `/dev/i2c-1`

### Pi OS device verification

```bash
arecord -l
aplay -l
ls /dev/i2c*
```

## Admin Token

Store the admin token at `~/.axon/admin_token` (Pi and laptop). Admin-gated actions include:
- MAINTENANCE mode
- PWM debug operations
- Shutdown requests
- Audio control changes

## Bringup

### Pi (Domain 42, Docker)

```bash
cd docker/pi
sudo docker compose up --build
```

### Laptop Gateway

```bash
./scripts/install_gateway.sh
./scripts/run_gateway.sh
```

### Laptop UI

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ROS_DOMAIN_ID=43 ros2 run axon_operator_console operator_console
```

## Verification

```bash
./tests/test_allowlist_enforced.sh
./tests/verify_pi.sh
./tests/verify_distributed.sh
```

## Production Demo (End-to-End)

1. Bring up Pi core with Docker.
2. Bring up the gateway containers on the laptop.
3. Launch the operator console.
4. Confirm camera preview (future extension), power/audio status, and gateway status.
5. Drive teleop and pan/tilt (future extension).
6. Trigger TTS playback.
7. Review power status and perform E-STOP if needed.

## Notes

- Hardware backends default to `mock` for power/audio/PWM.
- Real UPS and PWM drivers can be added by extending backend modules.
- Gateway allowlist entries live in `scripts/gateway_allowlist.txt`.
