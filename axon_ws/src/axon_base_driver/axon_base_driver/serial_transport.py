"""Serial transport with reconnect and backoff."""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import serial

from axon_utils.retry import Backoff


@dataclass
class SerialConfig:
    port: str
    baudrate: int
    reconnect_backoff_s: float
    reconnect_max_backoff_s: float
    dry_run: bool = False


class SerialTransport:
    def __init__(self, config: SerialConfig) -> None:
        self._config = config
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._backoff = Backoff(config.reconnect_backoff_s, config.reconnect_max_backoff_s)

    def connect(self) -> bool:
        if self._config.dry_run:
            return True
        try:
            self._serial = serial.Serial(self._config.port, self._config.baudrate, timeout=0.1)
            return True
        except serial.SerialException:
            self._serial = None
            return False

    def ensure_connection(self) -> bool:
        if self._config.dry_run:
            return True
        if self._serial and self._serial.is_open:
            return True
        for delay in self._backoff.sequence():
            if self.connect():
                return True
            if delay >= self._config.reconnect_max_backoff_s:
                return False
        return False

    def write(self, payload: bytes) -> bool:
        if self._config.dry_run:
            return True
        if not self._serial or not self._serial.is_open:
            return False
        with self._lock:
            self._serial.write(payload)
        return True

    def read_line(self) -> Optional[str]:
        if self._config.dry_run:
            return None
        if not self._serial or not self._serial.is_open:
            return None
        with self._lock:
            line = self._serial.readline().decode("utf-8").strip()
        return line or None

    def close(self) -> None:
        if self._serial:
            self._serial.close()
            self._serial = None

    @property
    def is_connected(self) -> bool:
        if self._config.dry_run:
            return True
        return bool(self._serial and self._serial.is_open)
