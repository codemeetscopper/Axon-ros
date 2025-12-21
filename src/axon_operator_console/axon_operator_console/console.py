import argparse
import os
import threading
import time
import tkinter as tk
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from axon_msgs.msg import PowerStatus, AudioStatus, AxonHealth, PwmChannelCommand
from axon_msgs.srv import SetAudioControls, SetPwmBatch, RequestShutdown, SetRobotMode
from axon_interfaces import Topics, QosProfiles, Limits


@dataclass
class UiState:
    power: str = "unknown"
    audio: str = "unknown"
    health: str = "unknown"


class ConsoleNode(Node):
    def __init__(self, state: UiState) -> None:
        super().__init__("axon_operator_console")
        self._state = state
        self._power_sub = self.create_subscription(
            PowerStatus, Topics.POWER_STATUS, self._on_power, QosProfiles.STATUS
        )
        self._audio_sub = self.create_subscription(
            AudioStatus, Topics.AUDIO_STATUS, self._on_audio, QosProfiles.STATUS
        )
        self._health_sub = self.create_subscription(
            AxonHealth, Topics.AXON_HEALTH, self._on_health, QosProfiles.STATUS
        )
        self._audio_client = self.create_client(SetAudioControls, Topics.SET_AUDIO_CONTROLS)
        self._pwm_client = self.create_client(SetPwmBatch, Topics.SET_PWM_BATCH)
        self._shutdown_client = self.create_client(RequestShutdown, Topics.REQUEST_SHUTDOWN)
        self._mode_client = self.create_client(SetRobotMode, Topics.SET_ROBOT_MODE)

    def _on_power(self, msg: PowerStatus) -> None:
        self._state.power = f"bus {msg.bus_v:.1f}V battery {msg.battery1_pct:.0%} low={msg.low_battery}"

    def _on_audio(self, msg: AudioStatus) -> None:
        self._state.audio = f"vol {msg.output_volume_0_1:.2f} mute={msg.speaker_muted}"

    def _on_health(self, msg: AxonHealth) -> None:
        self._state.health = msg.last_error

    def send_audio(self, volume: float, mic_muted: bool, speaker_muted: bool, token: str) -> None:
        if not self._audio_client.service_is_ready():
            return
        request = SetAudioControls.Request()
        request.output_volume_0_1 = max(Limits.VOLUME_MIN, min(Limits.VOLUME_MAX, volume))
        request.mic_muted = mic_muted
        request.speaker_muted = speaker_muted
        request.admin_token = token
        self._audio_client.call_async(request)

    def request_shutdown(self, reason: str, token: str) -> None:
        if not self._shutdown_client.service_is_ready():
            return
        request = RequestShutdown.Request()
        request.reason = reason
        request.admin_token = token
        self._shutdown_client.call_async(request)

    def send_pwm(self, channel: int, duty: int, freq: int, token: str) -> None:
        if not self._pwm_client.service_is_ready():
            return
        cmd = PwmChannelCommand()
        cmd.channel = channel
        cmd.duty_0_65535 = duty
        cmd.freq_hz = freq
        request = SetPwmBatch.Request()
        request.commands = [cmd]
        request.admin_token = token
        self._pwm_client.call_async(request)

    def send_safe_stop(self) -> None:
        if not self._mode_client.service_is_ready():
            return
        request = SetRobotMode.Request()
        request.mode = "SAFE_STOP"
        request.reason = "operator_estop"
        request.admin_token = ""
        self._mode_client.call_async(request)


class OperatorConsoleApp:
    def __init__(self, state: UiState, node: ConsoleNode, admin_token: str) -> None:
        self._state = state
        self._node = node
        self._admin_token = admin_token
        self._root = tk.Tk()
        self._root.title("Axon Operator Console")
        self._build_ui()
        self._update_labels()

    def _build_ui(self) -> None:
        tabs = tk.Frame(self._root)
        tabs.pack(fill=tk.BOTH, expand=True)

        control = tk.LabelFrame(tabs, text="Control")
        control.pack(fill=tk.X, padx=4, pady=4)
        tk.Button(control, text="E-STOP", fg="white", bg="red", command=self._estop).pack(
            side=tk.LEFT, padx=4
        )
        tk.Label(control, text="Teleop").pack(side=tk.LEFT, padx=4)
        self._volume = tk.DoubleVar(value=0.5)
        tk.Label(control, text="Volume").pack(side=tk.LEFT)
        tk.Scale(control, variable=self._volume, from_=0.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL).pack(
            side=tk.LEFT
        )
        tk.Button(control, text="Mute", command=self._mute).pack(side=tk.LEFT, padx=4)
        tk.Button(control, text="Unmute", command=self._unmute).pack(side=tk.LEFT, padx=4)

        pwm = tk.LabelFrame(tabs, text="PWM Debug (Admin)")
        pwm.pack(fill=tk.X, padx=4, pady=4)
        self._pwm_channel = tk.IntVar(value=0)
        self._pwm_duty = tk.IntVar(value=3000)
        self._pwm_freq = tk.IntVar(value=50)
        tk.Label(pwm, text="Channel").pack(side=tk.LEFT)
        tk.Entry(pwm, textvariable=self._pwm_channel, width=4).pack(side=tk.LEFT)
        tk.Label(pwm, text="Duty").pack(side=tk.LEFT)
        tk.Entry(pwm, textvariable=self._pwm_duty, width=6).pack(side=tk.LEFT)
        tk.Label(pwm, text="Freq").pack(side=tk.LEFT)
        tk.Entry(pwm, textvariable=self._pwm_freq, width=4).pack(side=tk.LEFT)
        self._pwm_button = tk.Button(pwm, text="Send", command=self._send_pwm)
        self._pwm_button.pack(side=tk.LEFT, padx=4)
        if not self._admin_token:
            self._pwm_button.configure(state=tk.DISABLED)

        system = tk.LabelFrame(tabs, text="System")
        system.pack(fill=tk.X, padx=4, pady=4)
        self._power_label = tk.Label(system, text="Power: ")
        self._power_label.pack(anchor=tk.W)
        self._audio_label = tk.Label(system, text="Audio: ")
        self._audio_label.pack(anchor=tk.W)
        self._health_label = tk.Label(system, text="Health: ")
        self._health_label.pack(anchor=tk.W)
        tk.Button(system, text="Request Shutdown", command=self._shutdown).pack(anchor=tk.W, pady=4)

    def _mute(self) -> None:
        self._node.send_audio(self._volume.get(), mic_muted=False, speaker_muted=True, token=self._admin_token)

    def _unmute(self) -> None:
        self._node.send_audio(self._volume.get(), mic_muted=False, speaker_muted=False, token=self._admin_token)

    def _shutdown(self) -> None:
        self._node.request_shutdown("operator_request", self._admin_token)

    def _send_pwm(self) -> None:
        self._node.send_pwm(
            self._pwm_channel.get(),
            self._pwm_duty.get(),
            self._pwm_freq.get(),
            self._admin_token,
        )

    def _estop(self) -> None:
        self._node.send_safe_stop()

    def _update_labels(self) -> None:
        self._power_label.configure(text=f"Power: {self._state.power}")
        self._audio_label.configure(text=f"Audio: {self._state.audio}")
        self._health_label.configure(text=f"Health: {self._state.health}")
        self._root.after(500, self._update_labels)

    def run(self) -> None:
        self._root.mainloop()


def _load_admin_token() -> str:
    path = os.path.expanduser("~/.axon/admin_token")
    if not os.path.exists(path):
        return ""
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read().strip()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-test", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    state = UiState()
    node = ConsoleNode(state)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    if args.smoke_test:
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        return

    app = OperatorConsoleApp(state, node, _load_admin_token())
    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
