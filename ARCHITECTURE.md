# Axon ROS 2 Architecture

This document describes the package layout and runtime composition for the Axon robot workspace.

## Package Dependency Graph
```mermaid
graph TD
  axon_bringup --> axon_base_driver
  axon_bringup --> axon_audio_driver
  axon_bringup --> axon_hri_voice
  axon_bringup --> axon_description
  axon_base_driver --> axon_utils
  axon_audio_driver --> axon_utils
  axon_hri_voice --> axon_utils
  axon_tests --> axon_utils
  axon_tests --> axon_base_driver
  axon_tests --> axon_audio_driver
  axon_interfaces --> axon_base_driver
  axon_interfaces --> axon_audio_driver
  axon_tcp_bridge --> axon_utils
```

## Runtime Node Graph (Topics)
```mermaid
graph LR
  cmd_vel[/cmd_vel/] --> base_driver((axon_base_driver))
  base_driver --> odom[/odom/]
  base_driver --> base_status[/base/status/]

  mic((axon_audio_driver/mic_node)) --> mic_pcm[/audio/mic/pcm/]
  speaker((axon_audio_driver/speaker_node)) <-- spk_pcm[/audio/speaker/pcm/]

  mic_pcm --> stt((axon_hri_voice/stt_stub_node))
  stt --> stt_text[/hri/stt/text/]

  tts_text[/hri/tts/text/] --> tts((axon_hri_voice/tts_stub_node))
  tts --> spk_pcm
```

## Launch Composition Graph
```mermaid
graph TD
  robot_launch[robot.launch.py]
  robot_launch --> base_launch[base.launch.py]
  robot_launch --> audio_launch[audio.launch.py]
  robot_launch --> hri_launch[hri.launch.py]
  robot_launch --> description_pkg[axon_description]
```
