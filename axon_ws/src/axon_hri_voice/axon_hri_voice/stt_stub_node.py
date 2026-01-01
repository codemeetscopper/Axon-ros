"""STT stub node."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from axon_interfaces.msg import AudioChunk


class SttStubNode(Node):
    def __init__(self) -> None:
        super().__init__("stt_stub")
        self._pub = self.create_publisher(String, "hri/stt/text", 10)
        self._sub = self.create_subscription(AudioChunk, "audio/mic/pcm", self._on_audio, 10)
        self.get_logger().info("STT stub ready")

    def _on_audio(self, msg: AudioChunk) -> None:
        text = String()
        text.data = "[stub] audio received"
        self._pub.publish(text)


def main() -> None:
    rclpy.init()
    node = SttStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
