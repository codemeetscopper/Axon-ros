import rclpy
from rclpy.node import Node

from axon_msgs.msg import PowerStatus
from axon_msgs.srv import SetRobotMode
from axon_interfaces import Topics, QosProfiles


class WatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("axon_watchdog")
        self._mode_client = self.create_client(SetRobotMode, Topics.SET_ROBOT_MODE)
        self._sub = self.create_subscription(
            PowerStatus, Topics.POWER_STATUS, self._on_power, QosProfiles.STATUS
        )

    def _on_power(self, msg: PowerStatus) -> None:
        if msg.low_battery:
            if not self._mode_client.service_is_ready():
                return
            request = SetRobotMode.Request()
            request.mode = "SAFE_STOP"
            request.reason = "low_battery"
            request.admin_token = ""
            self._mode_client.call_async(request)


def main() -> None:
    rclpy.init()
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
