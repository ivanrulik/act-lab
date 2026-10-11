cmake_minimum_required(VERSION 3.8)
project(act_lab_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionTelemetry.msg" "msg/ExecutionEvent.msg"
  "msg/CameraSample.msg" "msg/RecordedSample.msg" "msg/RecordingPacket.msg"
  "msg/CartesianCommand.msg" "msg/RobotState.msg" "msg/CommandReport.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
