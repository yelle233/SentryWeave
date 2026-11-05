cmake_minimum_required(VERSION 3.16)
project(sentry_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotObservation.msg"
  "msg/ObservationArray.msg"
  "msg/RobotTrack.msg"
  "msg/RefereeState.msg"
  "msg/WorldState.msg"
  "msg/TeammateIntent.msg"
  "msg/TeammateIntentArray.msg"
  "msg/ZoneThreat.msg"
  "msg/ThreatState.msg"
  "msg/TacticalTask.msg"
  "msg/TaskFeedback.msg"
  "msg/MotionPermit.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
