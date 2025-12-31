from __future__ import annotations

import json
import queue
import socket
import threading
from dataclasses import dataclass
from typing import Optional

from PySide6 import QtCore, QtWidgets

from .components import ComponentWidget


@dataclass
class ClientStatus:
    connected: bool = False
    last_feedback: str = ""


class ClientSignals(QtCore.QObject):
    log = QtCore.Signal(str)
    feedback = QtCore.Signal(str)
    connection = QtCore.Signal(bool)


class TcpJsonClient:
    def __init__(self, signals: ClientSignals) -> None:
        self._signals = signals
        self._socket: Optional[socket.socket] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._outbound: queue.Queue[str] = queue.Queue()

    def connect(self, host: str, port: int) -> None:
        self._stop.clear()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((host, port))
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        threading.Thread(target=self._writer_loop, daemon=True).start()
        self._signals.connection.emit(True)
        self._signals.log.emit(f"Connected to {host}:{port}")

    def disconnect(self) -> None:
        self._stop.set()
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
        self._socket = None
        self._signals.connection.emit(False)
        self._signals.log.emit("Disconnected")

    def send(self, payload: dict) -> None:
        self._outbound.put(json.dumps(payload))

    def _reader_loop(self) -> None:
        if not self._socket:
            return
        try:
            fileobj = self._socket.makefile("r")
            for line in fileobj:
                if self._stop.is_set():
                    break
                line = line.strip()
                if not line:
                    continue
                self._signals.feedback.emit(line)
                self._signals.log.emit(f"RX: {line}")
        except OSError:
            pass
        finally:
            if not self._stop.is_set():
                self.disconnect()

    def _writer_loop(self) -> None:
        while not self._stop.is_set():
            try:
                payload = self._outbound.get(timeout=0.5)
            except queue.Empty:
                continue
            if not self._socket:
                break
            try:
                self._socket.sendall(f"{payload}\n".encode("utf-8"))
            except OSError:
                break


class TesterComponent(ComponentWidget):
    component_name = "Chassis Tester"
    component_description = "Send commands and inspect feedback"

    def __init__(self) -> None:
        super().__init__()
        self.status = ClientStatus()
        self.signals = ClientSignals()
        self.client = TcpJsonClient(self.signals)
        self._build_ui()

        self.signals.log.connect(self._append_log)
        self.signals.feedback.connect(self._update_feedback)
        self.signals.connection.connect(self._update_connection)

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        header = QtWidgets.QLabel("Axon Chassis Tester")
        header.setObjectName("SectionTitle")
        subheader = QtWidgets.QLabel("Drive, PWM, OLED, and feedback tools")
        subheader.setObjectName("Muted")
        layout.addWidget(header)
        layout.addWidget(subheader)

        connection_group = QtWidgets.QGroupBox("Connection")
        connection_layout = QtWidgets.QGridLayout(connection_group)
        self.host_input = QtWidgets.QLineEdit("127.0.0.1")
        self.port_input = QtWidgets.QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(7000)
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)

        connection_layout.addWidget(QtWidgets.QLabel("Host"), 0, 0)
        connection_layout.addWidget(self.host_input, 0, 1)
        connection_layout.addWidget(QtWidgets.QLabel("Port"), 0, 2)
        connection_layout.addWidget(self.port_input, 0, 3)
        connection_layout.addWidget(self.connect_button, 1, 2)
        connection_layout.addWidget(self.disconnect_button, 1, 3)
        layout.addWidget(connection_group)

        drive_group = QtWidgets.QGroupBox("Drive")
        drive_layout = QtWidgets.QGridLayout(drive_group)
        self.speed_left = QtWidgets.QDoubleSpinBox()
        self.speed_left.setRange(-1.0, 1.0)
        self.speed_left.setSingleStep(0.05)
        self.speed_right = QtWidgets.QDoubleSpinBox()
        self.speed_right.setRange(-1.0, 1.0)
        self.speed_right.setSingleStep(0.05)
        self.send_speed = QtWidgets.QPushButton("Send speed")

        drive_layout.addWidget(QtWidgets.QLabel("Left"), 0, 0)
        drive_layout.addWidget(self.speed_left, 0, 1)
        drive_layout.addWidget(QtWidgets.QLabel("Right"), 0, 2)
        drive_layout.addWidget(self.speed_right, 0, 3)
        drive_layout.addWidget(self.send_speed, 1, 3)
        layout.addWidget(drive_group)

        pwm_group = QtWidgets.QGroupBox("PWM")
        pwm_layout = QtWidgets.QGridLayout(pwm_group)
        self.pwm_left = QtWidgets.QSpinBox()
        self.pwm_left.setRange(-255, 255)
        self.pwm_right = QtWidgets.QSpinBox()
        self.pwm_right.setRange(-255, 255)
        self.send_pwm = QtWidgets.QPushButton("Send PWM")

        pwm_layout.addWidget(QtWidgets.QLabel("Left"), 0, 0)
        pwm_layout.addWidget(self.pwm_left, 0, 1)
        pwm_layout.addWidget(QtWidgets.QLabel("Right"), 0, 2)
        pwm_layout.addWidget(self.pwm_right, 0, 3)
        pwm_layout.addWidget(self.send_pwm, 1, 3)
        layout.addWidget(pwm_group)

        oled_group = QtWidgets.QGroupBox("OLED")
        oled_layout = QtWidgets.QGridLayout(oled_group)
        self.oled_line = QtWidgets.QSpinBox()
        self.oled_line.setRange(0, 3)
        self.oled_text = QtWidgets.QLineEdit()
        self.send_oled = QtWidgets.QPushButton("Send line")
        self.clear_oled = QtWidgets.QPushButton("Clear")

        oled_layout.addWidget(QtWidgets.QLabel("Line"), 0, 0)
        oled_layout.addWidget(self.oled_line, 0, 1)
        oled_layout.addWidget(QtWidgets.QLabel("Text"), 0, 2)
        oled_layout.addWidget(self.oled_text, 0, 3)
        oled_layout.addWidget(self.send_oled, 1, 2)
        oled_layout.addWidget(self.clear_oled, 1, 3)
        layout.addWidget(oled_group)

        feedback_group = QtWidgets.QGroupBox("Feedback")
        feedback_layout = QtWidgets.QGridLayout(feedback_group)
        self.request_feedback = QtWidgets.QPushButton("Request snapshot")
        self.enable_feedback = QtWidgets.QPushButton("Enable stream")
        self.disable_feedback = QtWidgets.QPushButton("Disable stream")
        self.toggle_echo = QtWidgets.QPushButton("Toggle echo")

        feedback_layout.addWidget(self.request_feedback, 0, 0)
        feedback_layout.addWidget(self.enable_feedback, 0, 1)
        feedback_layout.addWidget(self.disable_feedback, 0, 2)
        feedback_layout.addWidget(self.toggle_echo, 0, 3)
        layout.addWidget(feedback_group)

        status_group = QtWidgets.QGroupBox("Latest Feedback")
        status_layout = QtWidgets.QVBoxLayout(status_group)
        self.feedback_label = QtWidgets.QLabel("No feedback yet")
        self.feedback_label.setWordWrap(True)
        status_layout.addWidget(self.feedback_label)
        layout.addWidget(status_group)

        log_group = QtWidgets.QGroupBox("Tester Log")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        log_layout.addWidget(self.log)
        layout.addWidget(log_group)

        layout.addStretch()

        self.connect_button.clicked.connect(self._connect)
        self.disconnect_button.clicked.connect(self._disconnect)
        self.send_speed.clicked.connect(self._send_speed)
        self.send_pwm.clicked.connect(self._send_pwm)
        self.send_oled.clicked.connect(self._send_oled)
        self.clear_oled.clicked.connect(self._clear_oled)
        self.request_feedback.clicked.connect(self._request_feedback)
        self.enable_feedback.clicked.connect(self._enable_feedback)
        self.disable_feedback.clicked.connect(self._disable_feedback)
        self.toggle_echo.clicked.connect(self._toggle_echo)

    def on_deactivate(self) -> None:
        self._disconnect()

    def _connect(self) -> None:
        host = self.host_input.text().strip()
        port = int(self.port_input.value())
        try:
            self.client.connect(host, port)
        except OSError as exc:
            self.signals.log.emit(f"Connection failed: {exc}")

    def _disconnect(self) -> None:
        if self.status.connected:
            self.client.disconnect()

    def _send_speed(self) -> None:
        self._send_payload({"T": 1, "L": self.speed_left.value(), "R": self.speed_right.value()})

    def _send_pwm(self) -> None:
        self._send_payload({"T": 11, "L": self.pwm_left.value(), "R": self.pwm_right.value()})

    def _send_oled(self) -> None:
        self._send_payload({"T": 3, "lineNum": self.oled_line.value(), "Text": self.oled_text.text()})

    def _clear_oled(self) -> None:
        self._send_payload({"T": -3})

    def _request_feedback(self) -> None:
        self._send_payload({"T": 130})

    def _enable_feedback(self) -> None:
        self._send_payload({"T": 131, "cmd": 1})

    def _disable_feedback(self) -> None:
        self._send_payload({"T": 131, "cmd": 0})

    def _toggle_echo(self) -> None:
        self._send_payload({"T": 143, "cmd": 1})

    def _send_payload(self, payload: dict) -> None:
        if not self.status.connected:
            self.signals.log.emit("Not connected")
            return
        self.client.send(payload)
        self.signals.log.emit(f"TX: {json.dumps(payload)}")

    def _append_log(self, message: str) -> None:
        self.log.appendPlainText(message)

    def _update_feedback(self, payload: str) -> None:
        self.feedback_label.setText(payload)

    def _update_connection(self, connected: bool) -> None:
        self.status.connected = connected
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
