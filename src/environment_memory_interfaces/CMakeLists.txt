cmake_minimum_required(VERSION 3.8)
project(environment_memory_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExplorationStatus.msg"
  "msg/GeometricObjectObservation.msg"
  "msg/LocalizedObjectObservation.msg"
  "msg/MemoryObject.msg"
  "msg/VlmObservation.msg"
  "srv/QueryMemory.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs robot_interfaces sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
