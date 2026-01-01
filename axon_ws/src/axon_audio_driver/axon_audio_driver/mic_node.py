"""Microphone node publishing audio frames."""
from __future__ import annotations

import subprocess
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from axon_interfaces.msg import AudioChunk
from axon_utils.retry import Backoff
from axon_utils.ros_helpers import declare_parameters, get_params

from .audio_frames import AudioConfig


@dataclass
class MicConfig:
    input_device: str
    sample_rate: int
    channels: int
    chunk_ms: int
    encoding: str
    use_pulseaudio: bool


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__("mic_node")

        defaults = {
            "input_device": "default",
            "sample_rate": 16000,
            "channels": 1,
            "chunk_ms": 20,
            "encoding": "pcm_s16le",
            "use_pulseaudio": False,
        }
        declare_parameters(self, defaults)
        params = get_params(self, defaults)
        self._config = MicConfig(**params)
        self._audio_config = AudioConfig(
            sample_rate=self._config.sample_rate,
            channels=self._config.channels,
            chunk_ms=self._config.chunk_ms,
            encoding=self._config.encoding,
        )

        self._publisher = self.create_publisher(AudioChunk, "audio/mic/pcm", 10)
        self._thread = threading.Thread(target=self._run_capture, daemon=True)
        self._thread.start()

        self.get_logger().info("Mic node started")

    def _run_capture(self) -> None:
        backoff = Backoff(0.5, 5.0)
        for _ in backoff.sequence():
            process = self._start_process()
            if process is None:
                continue
            try:
                self._read_loop(process)
            finally:
                process.kill()

    def _start_process(self) -> Optional[subprocess.Popen]:
        device = self._config.input_device
        if self._config.use_pulseaudio and device == "default":
            device = "pulse"
        args = [
            "arecord",
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
            return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except FileNotFoundError:
            self.get_logger().error("arecord not found; install ALSA utils")
            return None

    def _read_loop(self, process: subprocess.Popen) -> None:
        if process.stdout is None:
            return
        frame_bytes = self._audio_config.frame_bytes
        while rclpy.ok():
            data = process.stdout.read(frame_bytes)
            if not data:
                self.get_logger().warning("audio capture stream ended")
                break
            msg = AudioChunk()
            msg.stamp = self.get_clock().now().to_msg()
            msg.sample_rate = self._config.sample_rate
            msg.channels = self._config.channels
            msg.encoding = self._config.encoding
            msg.data = list(data)
            self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = MicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
