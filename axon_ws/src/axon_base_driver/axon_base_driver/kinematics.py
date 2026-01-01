"""Diff-drive kinematics helpers."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DiffDriveKinematics:
    wheel_base: float
    max_speed: float

    def twist_to_wheel_speeds(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        left = linear_x - (angular_z * self.wheel_base / 2.0)
        right = linear_x + (angular_z * self.wheel_base / 2.0)
        return self._clamp(left), self._clamp(right)

    def _clamp(self, speed: float) -> float:
        if speed > self.max_speed:
            return self.max_speed
        if speed < -self.max_speed:
            return -self.max_speed
        return speed
