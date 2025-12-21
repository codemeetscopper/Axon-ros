import subprocess
import shutil

from .base import AudioBackend, AudioState


class TtsBackend(AudioBackend):
    def __init__(self, output_device: str, input_device: str, volume: float) -> None:
        self._output_device = output_device
        self._input_device = input_device
        self._volume = volume
        self._mic_muted = False
        self._speaker_muted = False
        self._last_event = "boot"
        self._tts_cmd = self._detect_tts()

    def _detect_tts(self) -> str:
        if shutil.which("espeak-ng"):
            return "espeak-ng"
        if shutil.which("espeak"):
            return "espeak"
        if shutil.which("pico2wave"):
            return "pico2wave"
        return ""

    def set_controls(self, volume: float, mic_muted: bool, speaker_muted: bool) -> None:
        self._volume = volume
        self._mic_muted = mic_muted
        self._speaker_muted = speaker_muted
        self._last_event = "controls_updated"

    def speak(self, text: str, voice: str, volume: float) -> None:
        if not self._tts_cmd:
            self._last_event = "tts_missing"
            return
        if self._tts_cmd.startswith("pico2wave"):
            self._last_event = "tts_playback"
            subprocess.run(["pico2wave", "-w", "/tmp/axon_tts.wav", "-l", "en-US", text], check=False)
            subprocess.run(["aplay", "/tmp/axon_tts.wav"], check=False)
            return
        cmd = [self._tts_cmd, "-s", "150", "-v", voice or "en", text]
        self._last_event = "tts_playback"
        subprocess.run(cmd, check=False)

    def get_state(self) -> AudioState:
        return AudioState(
            output_device=self._output_device,
            input_device=self._input_device,
            output_volume=self._volume,
            mic_muted=self._mic_muted,
            speaker_muted=self._speaker_muted,
            last_event=self._last_event,
        )
