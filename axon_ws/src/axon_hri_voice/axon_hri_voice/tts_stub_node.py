"""TTS stub node."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TtsStubNode(Node):
    def __init__(self) -> None:
        super().__init__("tts_stub")
        self._sub = self.create_subscription(String, "hri/tts/text", self._on_text, 10)
        self.get_logger().info("TTS stub ready")

    def _on_text(self, msg: String) -> None:
        self.get_logger().info("TTS request received: %s", msg.data)


def main() -> None:
    rclpy.init()
    node = TtsStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
