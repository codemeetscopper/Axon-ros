"""Retry and backoff utilities."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable


@dataclass
class Backoff:
    initial_s: float
    max_s: float
    multiplier: float = 2.0

    def sequence(self) -> Iterable[float]:
        delay = self.initial_s
        while True:
            yield delay
            delay = min(self.max_s, delay * self.multiplier)

    def sleep_generator(self) -> Iterable[float]:
        for delay in self.sequence():
            time.sleep(delay)
            yield delay
