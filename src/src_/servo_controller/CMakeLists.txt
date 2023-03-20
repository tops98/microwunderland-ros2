cmake_minimum_required(VERSION 3.8)
project(servo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(wiringPi_LIB wiringPi)

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

# add src folder to label
file(GLOB_RECURSE SOURCES "src/*.cpp")

# add executeables
add_executable(SwitchService ${SOURCES})
ament_target_dependencies(SwitchService rclcpp yaml-cpp)
target_link_libraries(SwitchService ${wiringPi_LIB} yaml-cpp ${cpp_typesupport_target})


#config ros2 cli tools
install(TARGETS
  SwitchService
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
