"""Protocol encoding/decoding for the chassis controller.

Uses JSON commands as documented by the vendor.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict


@dataclass
class SpeedCommand:
    left: float
    right: float


def encode_speed_ctrl(cmd: SpeedCommand) -> bytes:
    payload = {"T": 1, "L": cmd.left, "R": cmd.right}
    return (json.dumps(payload) + "\n").encode("utf-8")


def encode_pwm_ctrl(left: int, right: int) -> bytes:
    payload = {"T": 11, "L": left, "R": right}
    return (json.dumps(payload) + "\n").encode("utf-8")


def decode_message(raw: str) -> Dict[str, Any]:
    """Decode a JSON payload from the chassis (placeholder)."""
    return json.loads(raw)
