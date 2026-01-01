"""ROS 2 node for the Axon base driver."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from axon_interfaces.msg import BaseStatus as BaseStatusMsg
from axon_utils.config import ConfigError, dataclass_from_dict, validate_dataclass
from axon_utils.ros_helpers import declare_parameters, get_params

from .kinematics import DiffDriveKinematics
from .protocol import SpeedCommand, encode_speed_ctrl
from .serial_transport import SerialConfig, SerialTransport
from .state import DriverState


@dataclass
class BaseDriverConfig:
    port: str
    baudrate: int
    wheel_base: float
    max_speed: float
    publish_tf: bool
    odom_mode: str
    reconnect_backoff_s: float
    reconnect_max_backoff_s: float
    dry_run: bool

    def __post_init__(self) -> None:
        if self.wheel_base <= 0:
            raise ConfigError("wheel_base must be > 0")
        if self.max_speed <= 0:
            raise ConfigError("max_speed must be > 0")
        if self.odom_mode not in {"dummy", "wheel", "external"}:
            raise ConfigError("odom_mode must be dummy|wheel|external")


class BaseDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("base_driver")

        defaults = {
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "wheel_base": 0.3,
            "max_speed": 0.5,
            "publish_tf": True,
            "odom_mode": "dummy",
            "reconnect_backoff_s": 0.5,
            "reconnect_max_backoff_s": 5.0,
            "dry_run": False,
        }
        declare_parameters(self, defaults)
        params = get_params(self, defaults)

        config = dataclass_from_dict(BaseDriverConfig, params)
        validate_dataclass(config)

        self._config = config
        self._state = DriverState()
        self._kinematics = DiffDriveKinematics(config.wheel_base, config.max_speed)
        self._transport = SerialTransport(
            SerialConfig(
                port=config.port,
                baudrate=config.baudrate,
                reconnect_backoff_s=config.reconnect_backoff_s,
                reconnect_max_backoff_s=config.reconnect_max_backoff_s,
                dry_run=config.dry_run,
            )
        )

        self._cmd_sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._status_pub = self.create_publisher(BaseStatusMsg, "base/status", 10)

        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if config.publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info("Base driver initialized")

    def _on_cmd(self, msg: Twist) -> None:
        self._state.command.linear_x = msg.linear.x
        self._state.command.angular_z = msg.angular.z

    def _tick(self) -> None:
        if not self._transport.ensure_connection():
            self._state.status.state = "disconnected"
            self._state.status.detail = "Unable to connect to serial port"
            self._state.status.reconnect_count += 1
            return

        left, right = self._kinematics.twist_to_wheel_speeds(
            self._state.command.linear_x, self._state.command.angular_z
        )
        left_cmd = self._scale_to_pwm_range(left)
        right_cmd = self._scale_to_pwm_range(right)
        payload = encode_speed_ctrl(SpeedCommand(left=left_cmd, right=right_cmd))
        if not self._transport.write(payload):
            self._state.status.state = "write_error"
            self._state.status.detail = "Failed to write command"
            return

        self._state.status.state = "ok"
        self._state.status.detail = "command_sent"

        if self._config.odom_mode == "dummy":
            self._publish_dummy_odom()

    def _scale_to_pwm_range(self, speed: float) -> float:
        normalized = speed / self._config.max_speed
        scaled = max(-1.0, min(1.0, normalized)) * 0.5
        return scaled

    def _publish_dummy_odom(self) -> None:
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = float(self._state.command.linear_x)
        odom.twist.twist.angular.z = float(self._state.command.angular_z)
        self._odom_pub.publish(odom)

        if self._tf_broadcaster:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_status(self) -> None:
        status = BaseStatusMsg()
        status.stamp = self.get_clock().now().to_msg()
        status.state = self._state.status.state
        status.detail = self._state.status.detail
        status.reconnect_count = self._state.status.reconnect_count
        self._status_pub.publish(status)


def main() -> None:
    rclpy.init()
    node = BaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
