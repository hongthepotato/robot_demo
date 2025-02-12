cmake_minimum_required(VERSION 3.5)
project(my_msg)

# Set the C++ standard if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake and the ROS 2 message generation package
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find additional dependencies that your messages use
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)  # needed if your executable uses ROS 2 C++ client libraries

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
set(msg_files
  "msg/DriverOdo.msg"
  "msg/IrState.msg"
  "msg/DriverControl.msg"
)

## Generate interfaces (messages) from the .msg files.
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

## Build ##
add_executable(trans src/trans.cpp)

## Ensure that the trans target is built after the message interfaces are generated.
rosidl_target_interfaces(trans ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Link libraries to the executable. Use rclcpp if needed.
target_link_libraries(trans
  rclcpp::rclcpp
)

# Export dependencies so that downstream packages can use the generated interfaces.
ament_export_dependencies(rosidl_default_runtime)

# Declare the package as an ament package.
ament_package()

#############
## Install ##
#############

install(TARGETS
  trans
  DESTINATION lib/${PROJECT_NAME})

