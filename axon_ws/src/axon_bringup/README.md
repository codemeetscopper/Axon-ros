# axon_bringup

Launch files and configuration profiles for the Axon robot.

## Launch Files
- `robot.launch.py`: top-level bringup entrypoint.
- `base.launch.py`: chassis driver.
- `audio.launch.py`: mic + speaker.
- `hri.launch.py`: voice pipeline stubs.

## Parameters
Config files live under `config/` and are merged with the selected profile:
- `config/base.yaml`
- `config/audio.yaml`
- `config/hri.yaml`
- `config/profiles/{real,dev,sim}.yaml`

## Usage
```bash
ros2 launch axon_bringup robot.launch.py profile:=dev namespace:=
```
