cmake_minimum_required(VERSION 3.8)
project(microwunderland_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/StringUint16Pair.msg"
  "msg/ListNamed2DPositions.msg"
  "msg/Named2DPosition.msg"
  "msg/Vector2D.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxList.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjects.msg"
)

set(srv_files
  "srv/ExportImage.srv"
  "srv/GetAvailableStates.srv"
  "srv/GetCurrentState.srv"
  "srv/SetState.srv"
  "srv/SetStringKeyValue.srv"
  "srv/SetString.srv"
  "srv/AssigneTravelPlan.srv"
  "srv/StringArray.srv"
  "srv/GetUInt32.srv"
  "srv/SetUint32KeyValue.srv"
  "srv/GetFloat32.srv"
  "srv/GetStringArray.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ADD_LINTER_TESTS
  DEPENDENCIES sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
