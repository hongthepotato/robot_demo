#include "rclcpp/rclcpp.hpp"
#include "my_msg/msg/driver_odo.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>

class TransNode : public rclcpp::Node
{
public:
  TransNode() : Node("rostalker")
  {
    // 创建发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("stm32odo", 10);
    
    // 创建订阅者
    subscription_ = this->create_subscription<my_msg::msg::DriverOdo>(
      "stm32_odo", 10,
      std::bind(&TransNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const my_msg::msg::DriverOdo::SharedPtr odo)
  {
    RCLCPP_INFO(this->get_logger(), "odo数据为l:%d,r:%d", odo->odol, odo->odor);

    auto message = std_msgs::msg::String();
    std::stringstream ss;
    ss << "odo数据为l:" << odo->odol << ",r:" << odo->odor;
    message.data = ss.str();

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<my_msg::msg::DriverOdo>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
