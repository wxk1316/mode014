cmake_minimum_required(VERSION 3.8)
project(control_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/TurnState.msg"
  "msg/UdpConnectState.msg"
  "msg/WheelFootData.msg"
  "msg/ObstacleData.msg"
  "msg/TurnSrvDebug.msg"
  "msg/CarCtr.msg"
  "msg/CarState.msg"
  "msg/BitcarState.msg"
  "msg/MotorState.msg"
  "msg/Battery.msg"
  "msg/WorkState.msg"
  "msg/ChargeStation.msg"
  "msg/TrackState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_package()
