cmake_minimum_required(VERSION 3.8)
project(servo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(use_wiring_pi ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
if(use_wiring_pi)
  find_library(wiringPi_LIB wiringPi)
endif()
# generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StringUint16Pair.msg"
  "srv/GetAvailableStates.srv"
  "srv/GetCurrentState.srv"
  "srv/SetState.srv"
  ADD_LINTER_TESTS
)

# set support for using custom interfaces in c++ from this package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


# include header files
include_directories(include)

# add auxiliary src source files to label
file(GLOB_RECURSE AUX_SOURCES "src/*.cpp")
list(FILTER AUX_SOURCES EXCLUDE REGEX "src/ros_nodes/*")

# add executeables
add_executable(SwitchService "src/ros_nodes/SwitchService.cpp" ${AUX_SOURCES})
add_executable(ServoTester "src/ros_nodes/ServoTester.cpp" ${AUX_SOURCES})

ament_target_dependencies(SwitchService rclcpp yaml-cpp)
ament_target_dependencies(ServoTester rclcpp yaml-cpp)

if(use_wiring_pi)
  target_link_libraries(SwitchService ${wiringPi_LIB} yaml-cpp ${cpp_typesupport_target})
  target_link_libraries(ServoTester ${wiringPi_LIB} yaml-cpp ${cpp_typesupport_target})
else()
  target_link_libraries(SwitchService yaml-cpp ${cpp_typesupport_target})
  target_link_libraries(ServoTester yaml-cpp ${cpp_typesupport_target})

endif()

#config ros2 cli tools
install(TARGETS
  SwitchService
  ServoTester
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
