cmake_minimum_required(VERSION 3.8)
project(coroco_chassis)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(coroco_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ttzn_sdk REQUIRED)

# Include directories
include_directories(include)

file(GLOB SRCS src/*)
add_executable(${PROJECT_NAME}_node ${SRCS})

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  coroco_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# link ttzn_sdk library
target_link_libraries(${PROJECT_NAME}_node ttzn_sdk)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
