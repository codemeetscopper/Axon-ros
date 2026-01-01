# axon_base_driver

Serial chassis driver for the Axon robot base.

## Node
- `base_driver` (`axon_base_driver.node`)

## Topics
- Sub: `cmd_vel` (`geometry_msgs/Twist`)
- Pub: `odom` (`nav_msgs/Odometry`)
- Pub: `base/status` (`axon_interfaces/BaseStatus`)

## Parameters
- `port` (string)
- `baudrate` (int)
- `wheel_base` (float)
- `max_speed` (float)
- `publish_tf` (bool)
- `odom_mode` (dummy|wheel|external)
- `reconnect_backoff_s` (float)
- `reconnect_max_backoff_s` (float)
- `dry_run` (bool)

## Notes
Uses the vendor JSON command set with `T=1` speed control. Replace the
protocol encoder in `protocol.py` if hardware protocol changes.
