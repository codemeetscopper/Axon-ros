# axon_hri_voice

Voice pipeline skeleton that wires microphone and speaker topics to stub processing nodes.

## Nodes
- `voice_router_node`: optional passthrough from mic to speaker.
- `stt_stub_node`: publishes placeholder text on `hri/stt/text`.
- `tts_stub_node`: logs text messages from `hri/tts/text`.

## Topics
- Sub: `audio/mic/pcm` (`axon_interfaces/AudioChunk`)
- Pub: `audio/speaker/pcm` (`axon_interfaces/AudioChunk`)
- Pub: `hri/stt/text` (`std_msgs/String`)
- Sub: `hri/tts/text` (`std_msgs/String`)
