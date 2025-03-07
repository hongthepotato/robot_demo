cmake_minimum_required(VERSION 3.5)
project(robot_slam)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 如果您有自己的源代码，可以在这里添加可执行文件
# 例如：
# add_executable(your_node src/your_node.cpp)
# ament_target_dependencies(your_node
#   rclcpp
#   slam_toolbox
#   nav_msgs
#   sensor_msgs
#   tf2
#   tf2_ros
# )

# 安装可执行文件
# install(TARGETS
#   your_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装地图文件（如果有）
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
