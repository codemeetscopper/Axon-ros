import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from axon_msgs.msg import PowerStatus
from axon_msgs.srv import RequestShutdown
from axon_interfaces import Topics, QosProfiles, ParamDefaults
from axon_power.backends.base import PowerBackend, PowerSample
from axon_power.backends.mock import MockPowerBackend


class PowerNode(Node):
    def __init__(self) -> None:
        super().__init__("axon_power")
        self.declare_parameter("machine_id", "pi")
        self.declare_parameter("backend", ParamDefaults.POWER_BACKEND)
        self.declare_parameter("low_battery_pct", ParamDefaults.POWER_LOW_BATTERY_PCT)
        self.declare_parameter("shutdown_delay_s", ParamDefaults.POWER_SHUTDOWN_DELAY_S)
        self.declare_parameter("admin_token_path", ParamDefaults.ADMIN_TOKEN_PATH)
        self._machine_id = self.get_parameter("machine_id").value
        self._low_battery_pct = float(self.get_parameter("low_battery_pct").value)
        self._shutdown_delay_s = float(self.get_parameter("shutdown_delay_s").value)
        self._admin_token_path = self.get_parameter("admin_token_path").value
        self._backend = self._init_backend(self.get_parameter("backend").value)
        self._last_external_power: Optional[bool] = None
        self._last_low_battery = False
        self._last_event = "boot"
        self._low_battery_triggered_at: Optional[float] = None

        self._pub = self.create_publisher(PowerStatus, Topics.POWER_STATUS, QosProfiles.STATUS)
        self._srv = self.create_service(
            RequestShutdown, Topics.REQUEST_SHUTDOWN, self._handle_shutdown
        )
        self._timer = self.create_timer(1.0, self._tick)

    def _init_backend(self, name: str) -> PowerBackend:
        if name == "mock":
            self.get_logger().info("Using mock power backend")
            return MockPowerBackend()
        self.get_logger().warning("Unknown power backend '%s', falling back to mock", name)
        return MockPowerBackend()

    def _read_admin_token(self) -> str:
        path = os.path.expanduser(self._admin_token_path)
        if not os.path.exists(path):
            return ""
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()

    def _handle_shutdown(self, request: RequestShutdown.Request, response: RequestShutdown.Response):
        reason = request.reason.lower().strip()
        token = request.admin_token.strip()
        admin_token = self._read_admin_token()
        low_battery = self._last_low_battery
        allow = low_battery and "auto" in reason
        if not allow and token and admin_token and token == admin_token:
            allow = True
        if not allow:
            response.ok = False
            response.message = "unauthorized"
            return response
        response.ok = True
        response.message = "shutdown request accepted"
        self.get_logger().warning("Shutdown requested: %s", request.reason)
        return response

    def _evaluate_events(self, sample: PowerSample) -> None:
        if self._last_external_power is None:
            self._last_external_power = sample.external_power_present
        if sample.external_power_present != self._last_external_power:
            event = "external_power_restored" if sample.external_power_present else "external_power_lost"
            self._last_event = event
            self.get_logger().warning("Power event: %s", event)
            self._last_external_power = sample.external_power_present
        low_battery = (sample.battery1_pct < self._low_battery_pct) or (
            sample.battery2_pct < self._low_battery_pct
        )
        if low_battery and not self._last_low_battery:
            self._last_event = "low_battery"
            self._low_battery_triggered_at = time.time()
            self.get_logger().error("Low battery detected")
        if not low_battery and self._last_low_battery:
            self._last_event = "battery_ok"
            self._low_battery_triggered_at = None
        self._last_low_battery = low_battery

    def _tick(self) -> None:
        sample = self._backend.read()
        self._evaluate_events(sample)
        msg = PowerStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.bus_v = float(sample.bus_v)
        msg.bus_a = float(sample.bus_a)
        msg.battery1_v = float(sample.battery1_v)
        msg.battery1_pct = float(sample.battery1_pct)
        msg.battery2_v = float(sample.battery2_v)
        msg.battery2_pct = float(sample.battery2_pct)
        msg.ups1_present = bool(sample.ups1_present)
        msg.ups2_present = bool(sample.ups2_present)
        msg.external_power_present = bool(sample.external_power_present)
        msg.on_battery = bool(sample.on_battery)
        msg.low_battery = bool(self._last_low_battery)
        msg.last_power_event = self._last_event
        self._pub.publish(msg)
        if self._last_low_battery and self._shutdown_delay_s > 0:
            triggered_at = self._low_battery_triggered_at
            if triggered_at and time.time() - triggered_at > self._shutdown_delay_s:
                self.get_logger().error("Low battery shutdown timer expired")
                self._low_battery_triggered_at = None


def main() -> None:
    rclpy.init()
    node = PowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
