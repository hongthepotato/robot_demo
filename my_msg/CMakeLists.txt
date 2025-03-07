cmake_minimum_required(VERSION 3.8)
project(my_msg)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriverOdo.msg"
  "msg/IrState.msg"
  "msg/DriverControl.msg"
  DEPENDENCIES std_msgs
)

# Add executable
add_executable(trans src/trans.cpp)
ament_target_dependencies(trans
  rclcpp
  std_msgs
)

# 使用新的推荐方法替代rosidl_target_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(trans "${cpp_typesupport_target}")

# 确保在编译trans之前生成消息
add_dependencies(trans "${PROJECT_NAME}")

# Install executables
install(TARGETS
  trans
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
