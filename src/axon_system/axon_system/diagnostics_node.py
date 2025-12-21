import rclpy
from rclpy.node import Node

from axon_msgs.msg import AxonHealth, PowerStatus, AudioStatus
from axon_interfaces import Topics, QosProfiles


class DiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("axon_diagnostics")
        self.declare_parameter("machine_id", "pi")
        self._machine_id = self.get_parameter("machine_id").value
        self._last_power_event = "unknown"
        self._last_audio_event = "unknown"
        self._pub = self.create_publisher(AxonHealth, Topics.AXON_HEALTH, QosProfiles.STATUS)
        self._power_sub = self.create_subscription(
            PowerStatus, Topics.POWER_STATUS, self._on_power, QosProfiles.STATUS
        )
        self._audio_sub = self.create_subscription(
            AudioStatus, Topics.AUDIO_STATUS, self._on_audio, QosProfiles.STATUS
        )
        self._timer = self.create_timer(1.0, self._tick)

    def _on_power(self, msg: PowerStatus) -> None:
        self._last_power_event = msg.last_power_event

    def _on_audio(self, msg: AudioStatus) -> None:
        self._last_audio_event = msg.last_audio_event

    def _tick(self) -> None:
        msg = AxonHealth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.machine_id = self._machine_id
        msg.status = "ok"
        msg.last_error = f"power:{self._last_power_event} audio:{self._last_audio_event}"
        msg.cpu_temp_c = 0.0
        msg.cpu_load_0_1 = 0.0
        msg.mem_used_0_1 = 0.0
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
