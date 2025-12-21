import os

import rclpy
from rclpy.node import Node

from axon_msgs.msg import RobotMode
from axon_msgs.srv import SetRobotMode
from axon_interfaces import Topics, QosProfiles


class ModeManager(Node):
    def __init__(self) -> None:
        super().__init__("axon_mode_manager")
        self.declare_parameter("admin_token_path", "~/.axon/admin_token")
        self._admin_token_path = self.get_parameter("admin_token_path").value
        self._mode = "IDLE"
        self._pub = self.create_publisher(RobotMode, Topics.ROBOT_MODE, QosProfiles.LATCHED)
        self._srv = self.create_service(SetRobotMode, Topics.SET_ROBOT_MODE, self._handle_mode)
        self._timer = self.create_timer(1.0, self._tick)

    def _read_admin_token(self) -> str:
        path = os.path.expanduser(self._admin_token_path)
        if not os.path.exists(path):
            return ""
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()

    def _handle_mode(self, request: SetRobotMode.Request, response: SetRobotMode.Response):
        admin_token = self._read_admin_token()
        if request.mode == "MAINTENANCE" and request.admin_token.strip() != admin_token:
            response.ok = False
            response.message = "unauthorized"
            return response
        if request.mode == "SAFE_STOP":
            self._mode = "SAFE_STOP"
        else:
            self._mode = request.mode
        response.ok = True
        response.message = "mode updated"
        return response

    def _tick(self) -> None:
        msg = RobotMode()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = self._mode
        msg.reason = "mode_manager"
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
