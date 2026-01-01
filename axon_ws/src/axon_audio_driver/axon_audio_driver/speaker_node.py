"""Speaker node playing audio frames from ROS topics."""
from __future__ import annotations

import subprocess
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from axon_interfaces.msg import AudioChunk
from axon_utils.ros_helpers import declare_parameters, get_params

from .audio_frames import AudioConfig


@dataclass
class SpeakerConfig:
    output_device: str
    sample_rate: int
    channels: int
    chunk_ms: int
    encoding: str
    use_pulseaudio: bool


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__("speaker_node")
        defaults = {
            "output_device": "default",
            "sample_rate": 16000,
            "channels": 1,
            "chunk_ms": 20,
            "encoding": "pcm_s16le",
            "use_pulseaudio": False,
        }
        declare_parameters(self, defaults)
        params = get_params(self, defaults)
        self._config = SpeakerConfig(**params)
        self._audio_config = AudioConfig(
            sample_rate=self._config.sample_rate,
            channels=self._config.channels,
            chunk_ms=self._config.chunk_ms,
            encoding=self._config.encoding,
        )

        self._process_lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None

        self._sub = self.create_subscription(
            AudioChunk, "audio/speaker/pcm", self._on_audio, 10
        )

        self.get_logger().info("Speaker node started")

    def _ensure_process(self) -> Optional[subprocess.Popen]:
        with self._process_lock:
            if self._process and self._process.poll() is None:
                return self._process
            device = self._config.output_device
            if self._config.use_pulseaudio and device == "default":
                device = "pulse"
            args = [
                "aplay",
                "-D",
                device,
                "-f",
                "S16_LE",
                "-c",
                str(self._config.channels),
                "-r",
                str(self._config.sample_rate),
                "-t",
                "raw",
            ]
            try:
                self._process = subprocess.Popen(args, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
            except FileNotFoundError:
                self.get_logger().error("aplay not found; install ALSA utils")
                self._process = None
            return self._process

    def _on_audio(self, msg: AudioChunk) -> None:
        process = self._ensure_process()
        if process is None or process.stdin is None:
            return
        data = bytes(msg.data)
        try:
            process.stdin.write(data)
            process.stdin.flush()
        except BrokenPipeError:
            self.get_logger().warning("audio playback pipe closed")
            with self._process_lock:
                self._process = None


def main() -> None:
    rclpy.init()
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
