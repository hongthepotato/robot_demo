#include "rclcpp/rclcpp.hpp"
#include "my_msg/msg/driver_control.hpp"
#include "robot_base/robot_base.h"
#include "geometry_msgs/msg/twist.hpp"

class RobotCmdNode : public rclcpp::Node
{
public:
  RobotCmdNode() : Node("robot_vel_cmd")
  {
    // 创建发布者
    publisher_ = this->create_publisher<my_msg::msg::DriverControl>("dc_cmd", 5);
    
    // 创建订阅者
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 50, 
      std::bind(&RobotCmdNode::cmdCallback, this, std::placeholders::_1));
    
    // 清理编码器数据
    auto dc_cmd = my_msg::msg::DriverControl();
    rclcpp::sleep_for(std::chrono::seconds(6));
    dc_cmd.mod = 0x04;      // 清理编码器数据
    dc_cmd.quick_stop = 0;
    publisher_->publish(dc_cmd);
    RCLCPP_INFO(this->get_logger(), "send clear encode");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double RobotV_, RobotYawRate_, speedl, speedr;
    auto dc_cmd = my_msg::msg::DriverControl();
    dc_cmd.mod = 3;
    dc_cmd.quick_stop = 0;

    RobotV_ = msg->linear.x;  // m/s
    RobotYawRate_ = msg->angular.z;  // rad/s
    // RCLCPP_INFO(this->get_logger(), "RobotV_:%f,RobotYawRate_:%f", RobotV_, RobotYawRate_);
    double r = RobotV_ / RobotYawRate_;  // m

    if (RobotV_ == 0)  // 旋转
    {
      speedl = (-RobotYawRate_ * ROBOT_RADIUS);  // m/s
      speedr = (RobotYawRate_ * ROBOT_RADIUS);  // m/s
    }
    else if (RobotYawRate_ == 0)  // 直线
    {
      speedl = RobotV_;  // m/s
      speedr = RobotV_;
    }
    else  // 速度不一致
    {
      speedl = (RobotYawRate_ * (r - ROBOT_RADIUS));  // m/s
      speedr = (RobotYawRate_ * (r + ROBOT_RADIUS));
    }
    speedl = speedl / length_per_circle * 60.0f;
    speedr = speedr / length_per_circle * 60.0f;

    if (speedl < 1 && speedl > -1) {
      if (speedl < -0.05) speedl = -1;
      else if (speedl > 0.05) speedl = 1;
    }
    if (speedr < 1 && speedr > -1) {
      if (speedr < -0.05) speedr = -1;
      else if (speedr > 0.05) speedr = 1;
    }

    dc_cmd.speedl = (int16_t)speedl;
    dc_cmd.speedr = (int16_t)speedr;

    publisher_->publish(dc_cmd);
    // RCLCPP_INFO(this->get_logger(), "speedr:%d,speedl:%d", dc_cmd.speedr, dc_cmd.speedl);
  }

  rclcpp::Publisher<my_msg::msg::DriverControl>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCmdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}