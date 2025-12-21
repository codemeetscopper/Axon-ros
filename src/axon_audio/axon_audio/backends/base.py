from dataclasses import dataclass


@dataclass
class AudioState:
    output_device: str
    input_device: str
    output_volume: float
    mic_muted: bool
    speaker_muted: bool
    last_event: str


class AudioBackend:
    def set_controls(self, volume: float, mic_muted: bool, speaker_muted: bool) -> None:
        raise NotImplementedError

    def speak(self, text: str, voice: str, volume: float) -> None:
        raise NotImplementedError

    def get_state(self) -> AudioState:
        raise NotImplementedError
