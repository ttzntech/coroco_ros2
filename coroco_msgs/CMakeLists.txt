cmake_minimum_required(VERSION 3.8)
project(coroco_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set to c++ 14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SysStatus.msg"
  "msg/MoveCtrl.msg"
  "msg/MotorInfoFb.msg"
  "msg/WarnFb.msg"
  "msg/ModeCtrl.msg"
  "msg/LightCtrl.msg"
  "msg/OdomFb.msg"
  "msg/BMSFb.msg"
  "msg/TurnTweak.msg"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)

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
