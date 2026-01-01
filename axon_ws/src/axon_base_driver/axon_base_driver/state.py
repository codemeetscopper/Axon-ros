"""State containers for the chassis driver."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class BaseCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass
class BaseStatus:
    state: str = "init"
    detail: str = ""
    reconnect_count: int = 0
    last_error: Optional[str] = None


@dataclass
class DriverState:
    command: BaseCommand = field(default_factory=BaseCommand)
    status: BaseStatus = field(default_factory=BaseStatus)
