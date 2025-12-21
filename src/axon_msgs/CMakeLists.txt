cmake_minimum_required(VERSION 3.8)
project(axon_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  msg/AxonHealth.msg
  msg/RobotMode.msg
  msg/PanTiltCommand.msg
  msg/Heartbeat.msg
  msg/FaceDetections.msg
  msg/PowerStatus.msg
  msg/AudioStatus.msg
  msg/TtsRequest.msg
  msg/PwmChannelCommand.msg
  msg/PwmBatchCommand.msg
)

set(srv_files
  srv/SetRobotMode.srv
  srv/SetPanTilt.srv
  srv/RequestShutdown.srv
  srv/SetAudioControls.srv
  srv/SetPwmBatch.srv
)

set(action_files
  action/NavigateToPose.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
