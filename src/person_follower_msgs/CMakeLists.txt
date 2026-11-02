cmake_minimum_required(VERSION 3.8)
project(person_follower_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonDetection.msg"
  "msg/PersonDetectionArray.msg"
  "msg/TrackedPerson.msg"
  "msg/TrackedPersonArray.msg"
  "msg/FollowTarget.msg"
  "msg/RobotDecision.msg"
  "srv/Imprint.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
