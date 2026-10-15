cmake_minimum_required(VERSION 3.8)
project(smartboat_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FusionDetection.msg"
  "msg/RadarStatus.msg"
  "msg/RadarTarget.msg"
  "msg/RadarTargetArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
