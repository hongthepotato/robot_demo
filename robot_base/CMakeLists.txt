cmake_minimum_required(VERSION 3.8)
project(robot_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(my_msg REQUIRED)

# Include directories
include_directories(
  include
)

# Declare executables
add_executable(robot_odom src/robot_odom.cpp)
add_executable(robot_odom_time src/robot_odom_time.cpp)
add_executable(robot_cmd src/robot_cmd.cpp)
add_executable(robot_rod_cmd src/robot_rod_cmd.cpp)
add_executable(robot_imu src/robot_imu.cpp)

# Add dependencies
ament_target_dependencies(robot_odom
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  my_msg
)

ament_target_dependencies(robot_odom_time
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  my_msg
)

ament_target_dependencies(robot_cmd
  rclcpp
  geometry_msgs
  my_msg
)

ament_target_dependencies(robot_rod_cmd
  rclcpp
  geometry_msgs
  my_msg
)

ament_target_dependencies(robot_imu
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS
  robot_odom
  robot_odom_time
  robot_cmd
  robot_rod_cmd
  robot_imu
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install params directory if it exists
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
