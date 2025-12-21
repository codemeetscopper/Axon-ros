from .base import AudioBackend, AudioState


class MockAudioBackend(AudioBackend):
    def __init__(self, output_device: str, input_device: str, volume: float) -> None:
        self._output_device = output_device
        self._input_device = input_device
        self._volume = volume
        self._mic_muted = False
        self._speaker_muted = False
        self._last_event = "boot"

    def set_controls(self, volume: float, mic_muted: bool, speaker_muted: bool) -> None:
        self._volume = volume
        self._mic_muted = mic_muted
        self._speaker_muted = speaker_muted
        self._last_event = "controls_updated"

    def speak(self, text: str, voice: str, volume: float) -> None:
        self._last_event = f"tts:{voice}:{text[:24]}"

    def get_state(self) -> AudioState:
        return AudioState(
            output_device=self._output_device,
            input_device=self._input_device,
            output_volume=self._volume,
            mic_muted=self._mic_muted,
            speaker_muted=self._speaker_muted,
            last_event=self._last_event,
        )
