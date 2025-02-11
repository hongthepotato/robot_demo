#include "rclcpp/rclcpp.hpp"
#include "my_msg/msg/driver_control.hpp"       // ROS 2 custom message header
#include "geometry_msgs/msg/twist.hpp"
#include "robot_base/robot_base.h"             // Contains constants like ROBOT_RADIUS, length_per_circle
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class RobotCmdNode : public rclcpp::Node
{
public:
  RobotCmdNode() : Node("robot_vel_cmd")
  {
    // Create a subscription to "cmd_vel" (Twist messages)
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 50,
      std::bind(&RobotCmdNode::cmdCallback, this, std::placeholders::_1)
    );

    // Create a publisher for "dc_cmd" (custom DriverControl messages)
    pub_ = this->create_publisher<my_msg::msg::DriverControl>("dc_cmd", 5);

    // Wait 6 seconds for lower-level initialization
    RCLCPP_INFO(this->get_logger(), "Waiting 6 seconds for lower-level initialization...");
    std::this_thread::sleep_for(6s);

    // Send an initial command to clear encoder data
    my_msg::msg::DriverControl dc_cmd;
    dc_cmd.mod = 0x04;      // Clear encoder data
    dc_cmd.quick_stop = 0;
    pub_->publish(dc_cmd);
    RCLCPP_INFO(this->get_logger(), "Sent clear encoder command");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double RobotV_, RobotYawRate_, speedl, speedr;
    my_msg::msg::DriverControl dc_cmd;
    dc_cmd.mod = 3;
    dc_cmd.quick_stop = 0;

    // Extract linear and angular velocity from the Twist message
    RobotV_ = msg->linear.x;      // m/s
    RobotYawRate_ = msg->angular.z; // rad/s

    // Compute turning radius if angular velocity is not zero.
    double r = (RobotYawRate_ != 0) ? (RobotV_ / RobotYawRate_) : 0.0;

    // Compute wheel speeds based on the motion mode.
    if (RobotV_ == 0)  // In-place rotation
    {
      speedl = (-RobotYawRate_ * ROBOT_RADIUS); // m/s
      speedr = (RobotYawRate_ * ROBOT_RADIUS);  // m/s
    }
    else if (RobotYawRate_ == 0) // Straight-line motion
    {
      speedl = RobotV_;
      speedr = RobotV_;
    }
    else // Curved motion
    {
      speedl = (RobotYawRate_ * (r - ROBOT_RADIUS)); // m/s
      speedr = (RobotYawRate_ * (r + ROBOT_RADIUS));
    }

    // Convert the computed speeds into desired units
    speedl = speedl / length_per_circle * 60.0f;
    speedr = speedr / length_per_circle * 60.0f;

    // Apply a threshold to ensure non-negligible speeds
    if (speedl < 1 && speedl > -1) {
      if (speedl < -0.05)
        speedl = -1;
      else if (speedl > 0.05)
        speedl = 1;
    }
    if (speedr < 1 && speedr > -1) {
      if (speedr < -0.05)
        speedr = -1;
      else if (speedr > 0.05)
        speedr = 1;
    }

    // Set the wheel speeds in the custom message (casting to int16_t)
    dc_cmd.speedl = static_cast<int16_t>(speedl);
    dc_cmd.speedr = static_cast<int16_t>(speedr);

    // Publish the command message
    pub_->publish(dc_cmd);
    // Optionally, you can log the sent speeds:
    // RCLCPP_INFO(this->get_logger(), "Published speeds: left=%d, right=%d", dc_cmd.speedl, dc_cmd.speedr);
  }

  // Member variables for publisher and subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<my_msg::msg::DriverControl>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCmdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}