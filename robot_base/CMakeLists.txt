cmake_minimum_required(VERSION 3.5)
project(robot_base)

## Compile as C++14, supported in ROS2-foxy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ------------------------------------------------------------------------------
# Find Dependencies
# ------------------------------------------------------------------------------

# In ROS 2 we use ament_cmake instead of catkin.
find_package(ament_cmake REQUIRED)

# Find the required ROS 2 packages:
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(my_msg REQUIRED)  # This is your custom message package.
# If needed, you can add other dependencies like nav_msgs here:
find_package(nav_msgs REQUIRED)


# ------------------------------------------------------------------------------
# Include Directories
# ------------------------------------------------------------------------------

include_directories(
  include
)

# ------------------------------------------------------------------------------
# Declare Executables and Link Dependencies
# ------------------------------------------------------------------------------

# For each executable, we create a target and link it with appropriate libraries
add_executable(robot_odom src/robot_odom.cpp)
ament_target_dependencies(robot_odom
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  my_msg
)

add_executable(robot_odom_time src/robot_odom_time.cpp)
ament_target_dependencies(robot_odom_time
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  my_msg  
)

add_executable(robot_cmd src/robot_cmd.cpp)
ament_target_dependencies(robot_cmd
  rclcpp
  geometry_msgs
  my_msg
)

add_executable(robot_rod_cmd src/robot_rod_cmd.cpp)
ament_target_dependencies(robot_rod_cmd
  rclcpp
  std_msgs
)

add_executable(robot_imu src/robot_imu.cpp)
ament_target_dependencies(robot_imu
  rclcpp
  sensor_msgs
)

# ------------------------------------------------------------------------------
# Install Targets
# ------------------------------------------------------------------------------

install(TARGETS
  robot_odom
  robot_odom_time
  robot_cmd
  robot_rod_cmd
  robot_imu
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Optionally, install header files from the include directory.
install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

# ------------------------------------------------------------------------------
# Ament Package
# ------------------------------------------------------------------------------

ament_package()
