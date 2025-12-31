from __future__ import annotations

import json
import logging
import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from PySide6 import QtCore, QtWidgets

from .components import ComponentWidget


logger = logging.getLogger("axon-simulator")


@dataclass
class MockState:
    speed_left: float = 0.0
    speed_right: float = 0.0
    pwm_left: int = 0
    pwm_right: int = 0
    battery: float = 12.3
    imu: Dict[str, float] = field(default_factory=lambda: {"yaw": 0.0, "pitch": 0.0, "roll": 0.0})
    oled_lines: Dict[int, str] = field(default_factory=dict)
    feedback_enabled: bool = False
    echo_enabled: bool = False


class ClientConnection:
    def __init__(self, sock: socket.socket, addr: tuple[str, int]):
        self.sock = sock
        self.addr = addr
        self.outbound: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

    def enqueue(self, line: str) -> None:
        self.outbound.put(line)

    def close(self) -> None:
        self._stop.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()

    def _send_loop(self) -> None:
        while not self._stop.is_set():
            try:
                line = self.outbound.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.sock.sendall(f"{line}\n".encode("utf-8"))
            except OSError:
                break


class MockChassisServer:
    def __init__(
        self,
        host: str,
        port: int,
        state: MockState,
        on_log: Optional[Callable[[str], None]] = None,
        on_clients: Optional[Callable[[int], None]] = None,
        on_state: Optional[Callable[[MockState], None]] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.state = state
        self._server: Optional[socket.socket] = None
        self._clients: List[ClientConnection] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._on_log = on_log
        self._on_clients = on_clients
        self._on_state = on_state

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._log(f"Simulator listening on {self.host}:{self.port}")

    def stop(self) -> None:
        self._stop.set()
        if self._server:
            try:
                self._server.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._server.close()
        with self._lock:
            for client in list(self._clients):
                client.close()
            self._clients.clear()
        self._emit_clients()
        self._log("Simulator stopped")

    def broadcast(self, payload: Dict) -> None:
        line = json.dumps(payload, separators=(",", ":"))
        with self._lock:
            for client in list(self._clients):
                client.enqueue(line)

    def _run(self) -> None:
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.host, self.port))
        self._server.listen(5)
        self._server.settimeout(1.0)
        while not self._stop.is_set():
            try:
                client_sock, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                raise
            client = ClientConnection(client_sock, addr)
            with self._lock:
                self._clients.append(client)
            self._emit_clients()
            threading.Thread(target=self._client_loop, args=(client,), daemon=True).start()
            self._log(f"Client connected: {addr}")

    def _client_loop(self, client: ClientConnection) -> None:
        try:
            fileobj = client.sock.makefile("r")
            for line in fileobj:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    self._log(f"Invalid JSON from {client.addr}: {line}")
                    continue
                self._handle_command(obj, client)
        except OSError:
            pass
        finally:
            with self._lock:
                if client in self._clients:
                    self._clients.remove(client)
            self._emit_clients()
            client.close()
            self._log(f"Client disconnected: {client.addr}")

    def _handle_command(self, obj: Dict, client: ClientConnection) -> None:
        if self.state.echo_enabled:
            client.enqueue(json.dumps(obj))

        cmd_type = obj.get("T")
        if cmd_type == 1:
            self.state.speed_left = float(obj.get("L", 0.0))
            self.state.speed_right = float(obj.get("R", 0.0))
        elif cmd_type == 11:
            self.state.pwm_left = int(obj.get("L", 0))
            self.state.pwm_right = int(obj.get("R", 0))
        elif cmd_type == 3:
            line_num = int(obj.get("lineNum", 0))
            text = str(obj.get("Text", ""))
            self.state.oled_lines[line_num] = text
        elif cmd_type == -3:
            self.state.oled_lines.clear()
        elif cmd_type == 130:
            client.enqueue(json.dumps(self._feedback_payload()))
        elif cmd_type == 131:
            self.state.feedback_enabled = bool(obj.get("cmd", 0))
        elif cmd_type == 143:
            self.state.echo_enabled = bool(obj.get("cmd", 0))

        self._emit_state()

    def _feedback_payload(self) -> Dict:
        return {
            "T": 130,
            "ts": time.time(),
            "L": self.state.speed_left,
            "R": self.state.speed_right,
            "pwmL": self.state.pwm_left,
            "pwmR": self.state.pwm_right,
            "battery": self.state.battery,
            "imu": self.state.imu,
            "oled": self.state.oled_lines,
        }

    def _log(self, message: str) -> None:
        logger.info(message)
        if self._on_log:
            self._on_log(message)

    def _emit_state(self) -> None:
        if self._on_state:
            self._on_state(self.state)

    def _emit_clients(self) -> None:
        if self._on_clients:
            self._on_clients(len(self._clients))


class SimulatorSignals(QtCore.QObject):
    log = QtCore.Signal(str)
    state = QtCore.Signal(MockState)
    clients = QtCore.Signal(int)


class SimulatorComponent(ComponentWidget):
    component_name = "Chassis Simulator"
    component_description = "Simulated Wave Rover chassis server"

    def __init__(self) -> None:
        super().__init__()
        self.state = MockState()
        self.server: Optional[MockChassisServer] = None
        self.signals = SimulatorSignals()
        self._build_ui()

        self.signals.log.connect(self._append_log)
        self.signals.state.connect(self._render_state)
        self.signals.clients.connect(self._update_clients)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._publish_feedback)

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        header = QtWidgets.QLabel("Axon Chassis Simulator")
        header.setObjectName("SectionTitle")
        subheader = QtWidgets.QLabel("Local TCP simulator for chassis commands")
        subheader.setObjectName("Muted")

        layout.addWidget(header)
        layout.addWidget(subheader)

        controls = QtWidgets.QGroupBox("Server")
        controls_layout = QtWidgets.QGridLayout(controls)

        self.host_input = QtWidgets.QLineEdit("127.0.0.1")
        self.port_input = QtWidgets.QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(7000)

        self.interval_input = QtWidgets.QDoubleSpinBox()
        self.interval_input.setRange(0.05, 10.0)
        self.interval_input.setValue(0.2)
        self.interval_input.setSingleStep(0.05)

        self.feedback_checkbox = QtWidgets.QCheckBox("Continuous feedback")
        self.echo_checkbox = QtWidgets.QCheckBox("Echo commands")

        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)

        controls_layout.addWidget(QtWidgets.QLabel("Host"), 0, 0)
        controls_layout.addWidget(self.host_input, 0, 1)
        controls_layout.addWidget(QtWidgets.QLabel("Port"), 0, 2)
        controls_layout.addWidget(self.port_input, 0, 3)
        controls_layout.addWidget(QtWidgets.QLabel("Feedback interval (s)"), 1, 0)
        controls_layout.addWidget(self.interval_input, 1, 1)
        controls_layout.addWidget(self.feedback_checkbox, 1, 2)
        controls_layout.addWidget(self.echo_checkbox, 1, 3)
        controls_layout.addWidget(self.start_button, 2, 2)
        controls_layout.addWidget(self.stop_button, 2, 3)

        layout.addWidget(controls)

        stats = QtWidgets.QGroupBox("Live State")
        stats_layout = QtWidgets.QGridLayout(stats)
        self.client_label = QtWidgets.QLabel("0")
        self.speed_label = QtWidgets.QLabel("L 0.00 / R 0.00")
        self.pwm_label = QtWidgets.QLabel("L 0 / R 0")
        self.battery_label = QtWidgets.QLabel("12.3 V")
        self.imu_label = QtWidgets.QLabel("Yaw 0.0 | Pitch 0.0 | Roll 0.0")
        self.oled_label = QtWidgets.QLabel("(empty)")
        self.oled_label.setWordWrap(True)

        stats_layout.addWidget(QtWidgets.QLabel("Clients"), 0, 0)
        stats_layout.addWidget(self.client_label, 0, 1)
        stats_layout.addWidget(QtWidgets.QLabel("Speed"), 0, 2)
        stats_layout.addWidget(self.speed_label, 0, 3)
        stats_layout.addWidget(QtWidgets.QLabel("PWM"), 1, 0)
        stats_layout.addWidget(self.pwm_label, 1, 1)
        stats_layout.addWidget(QtWidgets.QLabel("Battery"), 1, 2)
        stats_layout.addWidget(self.battery_label, 1, 3)
        stats_layout.addWidget(QtWidgets.QLabel("IMU"), 2, 0)
        stats_layout.addWidget(self.imu_label, 2, 1, 1, 3)
        stats_layout.addWidget(QtWidgets.QLabel("OLED"), 3, 0)
        stats_layout.addWidget(self.oled_label, 3, 1, 1, 3)

        layout.addWidget(stats)

        log_group = QtWidgets.QGroupBox("Simulator Log")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        log_layout.addWidget(self.log)
        layout.addWidget(log_group)

        layout.addStretch()

        self.start_button.clicked.connect(self.start_server)
        self.stop_button.clicked.connect(self.stop_server)
        self.feedback_checkbox.toggled.connect(self._toggle_feedback)
        self.echo_checkbox.toggled.connect(self._toggle_echo)

    def start_server(self) -> None:
        host = self.host_input.text().strip()
        port = int(self.port_input.value())
        self.server = MockChassisServer(
            host,
            port,
            self.state,
            on_log=self.signals.log.emit,
            on_clients=self.signals.clients.emit,
            on_state=self.signals.state.emit,
        )
        self.server.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.timer.start(int(self.interval_input.value() * 1000))
        self.signals.log.emit(f"Server started on {host}:{port}")

    def stop_server(self) -> None:
        if self.server:
            self.server.stop()
            self.server = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.timer.stop()

    def on_deactivate(self) -> None:
        self.stop_server()

    def _publish_feedback(self) -> None:
        if not self.server or not self.state.feedback_enabled:
            return
        payload = self.server._feedback_payload()
        self.server.broadcast(payload)
        self.signals.log.emit(f"Feedback: {payload}")

    def _toggle_feedback(self, checked: bool) -> None:
        self.state.feedback_enabled = checked

    def _toggle_echo(self, checked: bool) -> None:
        self.state.echo_enabled = checked

    def _append_log(self, message: str) -> None:
        self.log.appendPlainText(message)

    def _update_clients(self, count: int) -> None:
        self.client_label.setText(str(count))

    def _render_state(self, state: MockState) -> None:
        self.speed_label.setText(f"L {state.speed_left:.2f} / R {state.speed_right:.2f}")
        self.pwm_label.setText(f"L {state.pwm_left} / R {state.pwm_right}")
        self.battery_label.setText(f"{state.battery:.1f} V")
        self.imu_label.setText(
            f"Yaw {state.imu['yaw']:.1f} | Pitch {state.imu['pitch']:.1f} | Roll {state.imu['roll']:.1f}"
        )
        if state.oled_lines:
            lines = [f"{idx}: {text}" for idx, text in sorted(state.oled_lines.items())]
            self.oled_label.setText("\n".join(lines))
        else:
            self.oled_label.setText("(empty)")
