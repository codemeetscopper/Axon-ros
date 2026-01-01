import pytest

from axon_audio_driver.audio_frames import AudioConfig


def test_frame_bytes() -> None:
    config = AudioConfig(sample_rate=16000, channels=1, chunk_ms=20, encoding="pcm_s16le")
    assert config.frame_bytes == 640


def test_invalid_encoding() -> None:
    config = AudioConfig(sample_rate=16000, channels=1, chunk_ms=20, encoding="mp3")
    with pytest.raises(ValueError):
        _ = config.frame_bytes
