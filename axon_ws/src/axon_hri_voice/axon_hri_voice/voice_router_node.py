"""Voice router node: placeholder audio routing for HRI."""
from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from axon_interfaces.msg import AudioChunk
from axon_utils.ros_helpers import declare_parameters, get_params


@dataclass
class VoiceRouterConfig:
    passthrough_audio: bool


class VoiceRouterNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_router")
        defaults = {"passthrough_audio": True}
        declare_parameters(self, defaults)
        params = get_params(self, defaults)
        self._config = VoiceRouterConfig(**params)

        self._mic_sub = self.create_subscription(
            AudioChunk, "audio/mic/pcm", self._on_mic, 10
        )
        self._speaker_pub = self.create_publisher(AudioChunk, "audio/speaker/pcm", 10)

        self.get_logger().info("Voice router ready")

    def _on_mic(self, msg: AudioChunk) -> None:
        if self._config.passthrough_audio:
            self._speaker_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = VoiceRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
