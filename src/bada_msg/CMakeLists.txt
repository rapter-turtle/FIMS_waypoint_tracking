cmake_minimum_required(VERSION 3.8)
project(bada_msg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mode.msg"
  "msg/ActuatorOutputs.msg"
  "msg/ControlParameters.msg"
  "msg/PlcStatus.msg"
  "msg/Attitude.msg"
  "msg/GlobalPosition.msg"
  "msg/Speed.msg"
  "msg/MissionCurrent.msg"
  "msg/MissionItemReached.msg"
  "msg/RcChannel.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()