#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Create a node with the name "robot_rod_cmd"
  auto node = rclcpp::Node::make_shared("robot_rod_cmd");
  
  // Create a publisher for std_msgs/msg/Int8 on the "rod_cmd" topic with a queue size of 1
  auto publisher = node->create_publisher<std_msgs::msg::Int8>("rod_cmd", 1);
  
  // Create and initialize the message
  std_msgs::msg::Int8 cmd;
  cmd.data = 0;
  
  // Publish the message
  publisher->publish(cmd);
  
  // Process any pending callbacks (if needed)
  rclcpp::spin_some(node);
  
  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}