"""Audio framing helpers."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AudioConfig:
    sample_rate: int
    channels: int
    chunk_ms: int
    encoding: str

    def __post_init__(self) -> None:
        if self.sample_rate <= 0:
            raise ValueError("sample_rate must be > 0")
        if self.channels <= 0:
            raise ValueError("channels must be > 0")
        if self.chunk_ms <= 0:
            raise ValueError("chunk_ms must be > 0")

    @property
    def bytes_per_sample(self) -> int:
        if self.encoding != "pcm_s16le":
            raise ValueError("Only pcm_s16le is supported by default")
        return 2

    @property
    def frame_bytes(self) -> int:
        samples = int(self.sample_rate * (self.chunk_ms / 1000.0))
        return samples * self.channels * self.bytes_per_sample
