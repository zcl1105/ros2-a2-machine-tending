cmake_minimum_required(VERSION 3.8)
project(a2_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmMove.action"
  "msg/ArmState.msg"
  "msg/MachineState.msg"
  "msg/TaskStatus.msg"
  "msg/WorkpieceState.msg"
  "srv/MachineCommand.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
