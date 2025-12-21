from typing import Iterable

from axon_msgs.msg import PwmChannelCommand
from .base import PwmBackend, PwmState


class MockPwmBackend(PwmBackend):
    def __init__(self) -> None:
        self._last_event = "boot"
        self._commands: list[PwmChannelCommand] = []

    def apply(self, commands: Iterable[PwmChannelCommand]) -> None:
        self._commands = list(commands)
        self._last_event = f"applied:{len(self._commands)}"

    def get_state(self) -> PwmState:
        return PwmState(last_event=self._last_event, commands=list(self._commands))
