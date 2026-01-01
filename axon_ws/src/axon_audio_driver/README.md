# axon_audio_driver

Microphone and speaker nodes for Axon.

## Nodes
- `mic_node`: publishes audio frames on `audio/mic/pcm`.
- `speaker_node`: subscribes to `audio/speaker/pcm` and plays audio.

## Topics
- Pub: `audio/mic/pcm` (`axon_interfaces/AudioChunk`)
- Sub: `audio/speaker/pcm` (`axon_interfaces/AudioChunk`)

## Parameters
- `input_device`, `output_device` (string)
- `sample_rate` (int)
- `channels` (int)
- `chunk_ms` (int)
- `encoding` (string, default `pcm_s16le`)
- `use_pulseaudio` (bool)

## Dependencies
Uses ALSA utilities (`arecord`/`aplay`). Install via `alsa-utils` on Debian/Raspberry Pi.
