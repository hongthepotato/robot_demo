#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

class RobotRodCmdNode : public rclcpp::Node
{
public:
  RobotRodCmdNode() : Node("robot_rod_cmd")
  {
    // 创建发布者
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("rod_cmd", 1);
    
    // 发布命令
    auto cmd = std_msgs::msg::Int8();
    cmd.data = 0;
    publisher_->publish(cmd);
    
    // 由于只需要发布一次，我们可以在构造函数中完成后关闭节点
    RCLCPP_INFO(this->get_logger(), "命令已发送，节点将关闭");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotRodCmdNode>();
  rclcpp::spin_some(node);  // 只处理当前可用的回调
  rclcpp::shutdown();
  return 0;
}