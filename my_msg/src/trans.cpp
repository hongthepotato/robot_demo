#include "rclcpp/rclcpp.hpp"
#include "my_msg/msg/driver_odo.hpp"   // ROS 2 header for your custom message
#include "std_msgs/msg/string.hpp"
#include <sstream>

class Rostalker : public rclcpp::Node
{
public:
  Rostalker() : Node("rostalker")
  {
    // Create a publisher for std_msgs::msg::String on the "stm32odo" topic with a queue size of 10.
    pub_ = this->create_publisher<std_msgs::msg::String>("stm32odo", 10);

    // Create a subscription to my_msg::msg::DriverOdo on the "stm32_odo" topic with a queue size of 10.
    sub_ = this->create_subscription<my_msg::msg::DriverOdo>(
      "stm32_odo",
      10,
      std::bind(&Rostalker::callback, this, std::placeholders::_1)
    );
  }

private:
  void callback(const my_msg::msg::DriverOdo::SharedPtr odo)
  {
    // Log the received data
    RCLCPP_INFO(this->get_logger(), "odo数据为l:%d, r:%d", odo->odol, odo->odor);

    // Create a std_msgs::msg::String message and populate it
    std_msgs::msg::String pubstr;
    std::stringstream ss;
    ss << "odo数据为l:" << odo->odol << ",r:" << odo->odor;
    pubstr.data = ss.str();

    // Publish the message
    pub_->publish(pubstr);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<my_msg::msg::DriverOdo>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rostalker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
