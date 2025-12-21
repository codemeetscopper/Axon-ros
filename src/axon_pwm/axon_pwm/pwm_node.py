import os
from typing import Iterable

import rclpy
from rclpy.node import Node

from axon_msgs.msg import PwmBatchCommand
from axon_msgs.srv import SetPwmBatch
from axon_interfaces import Topics, QosProfiles, ParamDefaults, Limits
from axon_pwm.backends.base import PwmBackend
from axon_pwm.backends.mock import MockPwmBackend
from axon_pwm.backends.pca9685 import Pca9685Backend


class PwmNode(Node):
    def __init__(self) -> None:
        super().__init__("axon_pwm")
        self.declare_parameter("backend", ParamDefaults.PWM_BACKEND)
        self.declare_parameter("i2c_bus", ParamDefaults.PWM_I2C_BUS)
        self.declare_parameter("i2c_address", ParamDefaults.PWM_I2C_ADDRESS)
        self.declare_parameter("allowed_channels", ParamDefaults.PWM_ALLOWED_CHANNELS)
        self.declare_parameter("admin_token_path", ParamDefaults.ADMIN_TOKEN_PATH)
        self._allowed_channels = list(self.get_parameter("allowed_channels").value)
        self._admin_token_path = self.get_parameter("admin_token_path").value
        self._backend = self._init_backend(
            self.get_parameter("backend").value,
            int(self.get_parameter("i2c_bus").value),
            int(self.get_parameter("i2c_address").value),
        )

        self._pub = self.create_publisher(PwmBatchCommand, Topics.PWM_STATUS, QosProfiles.STATUS)
        self._srv = self.create_service(SetPwmBatch, Topics.SET_PWM_BATCH, self._handle_service)
        self._sub = self.create_subscription(
            PwmBatchCommand, Topics.PWM_BATCH, self._handle_batch, QosProfiles.CONTROL
        )
        self._timer = self.create_timer(1.0, self._publish_status)

    def _init_backend(self, name: str, bus: int, address: int) -> PwmBackend:
        if name == "pca9685":
            self.get_logger().info("Using PCA9685 backend on bus %s address %s", bus, address)
            return Pca9685Backend(bus, address)
        if name == "mock":
            self.get_logger().info("Using mock PWM backend")
            return MockPwmBackend()
        self.get_logger().warning("Unknown PWM backend '%s', falling back to mock", name)
        return MockPwmBackend()

    def _read_admin_token(self) -> str:
        path = os.path.expanduser(self._admin_token_path)
        if not os.path.exists(path):
            return ""
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()

    def _within_limits(self, command) -> bool:
        if command.channel not in self._allowed_channels:
            return False
        if not (Limits.PWM_DUTY_MIN <= command.duty_0_65535 <= Limits.PWM_DUTY_MAX):
            return False
        if not (Limits.PWM_FREQ_MIN <= command.freq_hz <= Limits.PWM_FREQ_MAX):
            return False
        return True

    def _filter_commands(self, commands: Iterable) -> list:
        return [cmd for cmd in commands if self._within_limits(cmd)]

    def _apply_commands(self, commands: Iterable) -> None:
        command_list = list(commands)
        valid = self._filter_commands(command_list)
        if len(valid) != len(command_list):
            self.get_logger().warning("Filtered unsafe PWM commands")
        self._backend.apply(valid)

    def _handle_batch(self, msg: PwmBatchCommand) -> None:
        self._apply_commands(msg.commands)

    def _handle_service(self, request: SetPwmBatch.Request, response: SetPwmBatch.Response):
        admin_token = self._read_admin_token()
        if request.admin_token.strip() != admin_token and admin_token:
            response.ok = False
            response.message = "unauthorized"
            return response
        self._apply_commands(request.commands)
        response.ok = True
        response.message = "batch applied"
        return response

    def _publish_status(self) -> None:
        state = self._backend.get_state()
        msg = PwmBatchCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.commands = state.commands
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PwmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
