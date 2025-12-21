from dataclasses import dataclass
from typing import Iterable

from axon_msgs.msg import PwmChannelCommand


@dataclass
class PwmState:
    last_event: str
    commands: list[PwmChannelCommand]


class PwmBackend:
    def apply(self, commands: Iterable[PwmChannelCommand]) -> None:
        raise NotImplementedError

    def get_state(self) -> PwmState:
        raise NotImplementedError
