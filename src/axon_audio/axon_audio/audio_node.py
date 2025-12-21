import os
import queue
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from axon_msgs.msg import AudioStatus, TtsRequest
from axon_msgs.srv import SetAudioControls
from axon_interfaces import Topics, QosProfiles, ParamDefaults, Limits
from axon_audio.backends.base import AudioBackend
from axon_audio.backends.mock import MockAudioBackend
from axon_audio.backends.tts import TtsBackend


class AudioWorker(threading.Thread):
    def __init__(self, backend: AudioBackend) -> None:
        super().__init__(daemon=True)
        self._backend = backend
        self._queue: queue.Queue[tuple[str, str, float]] = queue.Queue()
        self._running = True

    def run(self) -> None:
        while self._running:
            try:
                text, voice, volume = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._backend.speak(text, voice, volume)

    def stop(self) -> None:
        self._running = False

    def enqueue_tts(self, text: str, voice: str, volume: float) -> None:
        self._queue.put((text, voice, volume))


class AudioNode(Node):
    def __init__(self) -> None:
        super().__init__("axon_audio")
        self.declare_parameter("machine_id", "pi")
        self.declare_parameter("backend", ParamDefaults.AUDIO_BACKEND)
        self.declare_parameter("output_device", ParamDefaults.AUDIO_OUTPUT_DEVICE)
        self.declare_parameter("input_device", ParamDefaults.AUDIO_INPUT_DEVICE)
        self.declare_parameter("volume", ParamDefaults.AUDIO_VOLUME)
        self.declare_parameter("admin_token_path", ParamDefaults.ADMIN_TOKEN_PATH)
        self._machine_id = self.get_parameter("machine_id").value
        self._admin_token_path = self.get_parameter("admin_token_path").value
        backend_name = self.get_parameter("backend").value
        output_device = self.get_parameter("output_device").value
        input_device = self.get_parameter("input_device").value
        volume = float(self.get_parameter("volume").value)
        self._backend = self._init_backend(backend_name, output_device, input_device, volume)
        self._worker = AudioWorker(self._backend)
        self._worker.start()

        self._pub = self.create_publisher(AudioStatus, Topics.AUDIO_STATUS, QosProfiles.STATUS)
        self._mic_level_pub = self.create_publisher(Float32, Topics.MIC_LEVEL, QosProfiles.SENSOR)
        self._srv = self.create_service(
            SetAudioControls, Topics.SET_AUDIO_CONTROLS, self._handle_controls
        )
        self._sub = self.create_subscription(
            TtsRequest, Topics.TTS_REQUEST, self._handle_tts, QosProfiles.CONTROL
        )
        self._timer = self.create_timer(1.0, self._tick)

    def _init_backend(self, name: str, output_device: str, input_device: str, volume: float) -> AudioBackend:
        if name == "espeak" or name == "pico2wave":
            self.get_logger().info("Using TTS backend: %s", name)
            return TtsBackend(output_device, input_device, volume)
        if name == "mock":
            self.get_logger().info("Using mock audio backend")
            return MockAudioBackend(output_device, input_device, volume)
        self.get_logger().warning("Unknown audio backend '%s', falling back to mock", name)
        return MockAudioBackend(output_device, input_device, volume)

    def _read_admin_token(self) -> str:
        path = os.path.expanduser(self._admin_token_path)
        if not os.path.exists(path):
            return ""
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()

    def _handle_controls(self, request: SetAudioControls.Request, response: SetAudioControls.Response):
        admin_token = self._read_admin_token()
        if request.admin_token.strip() != admin_token and admin_token:
            response.ok = False
            response.message = "unauthorized"
            return response
        volume = max(Limits.VOLUME_MIN, min(Limits.VOLUME_MAX, request.output_volume_0_1))
        self._backend.set_controls(volume, request.mic_muted, request.speaker_muted)
        response.ok = True
        response.message = "controls updated"
        return response

    def _handle_tts(self, msg: TtsRequest) -> None:
        volume = max(Limits.VOLUME_MIN, min(Limits.VOLUME_MAX, msg.volume_0_1))
        self._worker.enqueue_tts(msg.text, msg.voice, volume)

    def _tick(self) -> None:
        state = self._backend.get_state()
        msg = AudioStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.output_device = state.output_device
        msg.input_device = state.input_device
        msg.output_volume_0_1 = float(state.output_volume)
        msg.mic_muted = bool(state.mic_muted)
        msg.speaker_muted = bool(state.speaker_muted)
        msg.last_audio_event = state.last_event
        self._pub.publish(msg)
        mic_level = Float32()
        mic_level.data = 0.0
        self._mic_level_pub.publish(mic_level)

    def destroy_node(self) -> bool:
        self._worker.stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AudioNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
