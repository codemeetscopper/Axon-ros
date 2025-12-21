from typing import Iterable

from axon_msgs.msg import PwmChannelCommand
from .base import PwmBackend, PwmState


class Pca9685Backend(PwmBackend):
    def __init__(self, bus: int, address: int) -> None:
        self._bus = bus
        self._address = address
        self._commands: list[PwmChannelCommand] = []
        self._last_event = "boot"

    def apply(self, commands: Iterable[PwmChannelCommand]) -> None:
        self._commands = list(commands)
        self._last_event = "pca9685_stub"

    def get_state(self) -> PwmState:
        return PwmState(last_event=self._last_event, commands=list(self._commands))
