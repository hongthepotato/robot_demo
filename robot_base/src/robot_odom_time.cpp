#include "rclcpp/rclcpp.hpp"
#include "my_msg/msg/driver_odo.hpp"          // ROS 2 version of your custom message
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// For TF2: to create a quaternion from a yaw angle
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Include your robot_base header that defines constants like meters_per_tick, ROBOT_LENGTH, etc.
#include "robot_base/robot_base.h"

#include <boost/array.hpp>
#include <cmath>
#include <chrono>
#include <thread>

class RobotOdomTimeNode : public rclcpp::Node
{
public:
  RobotOdomTimeNode()
  : Node("robot_base_odom"),
    odomx(0.0),
    odomy(0.0),
    odomth(0.0)
  {
    // Create a publisher for odometry messages on the "odom_wheel" topic.
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_wheel", 50);
    
    // Create a subscription for the custom odometry data on the "stm32_odo" topic.
    subscription_ = this->create_subscription<my_msg::msg::DriverOdo>(
      "stm32_odo", 2,
      std::bind(&RobotOdomTimeNode::callback, this, std::placeholders::_1)
    );
    
    // Initialize old odometry data (assuming driver_odo has fields 'odor' and 'odol').
    old_odo_data.odor = 0;
    old_odo_data.odol = 0;
    
    RCLCPP_INFO(this->get_logger(), "Waiting for lower-level initialization...");
    // Wait 10 seconds (blocking) to allow the lower-level hardware to initialize.
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
  
private:
  void callback(const my_msg::msg::DriverOdo::SharedPtr odo)
  {
    static int callbackcount = 0;
    double dr, dl, dxy_ave, dth, vxy, vth, dx, dy;
    rclcpp::Time current_time = this->now();
    
    // Convert interval (assumed in milliseconds) to seconds.
    double interval_time = static_cast<double>(odo->interval) / 1000.0;
    
    // Compute distance traveled by each wheel.
    dr = (static_cast<double>(odo->odor) - old_odo_data.odor) * meters_per_tick;
    dl = (static_cast<double>(odo->odol) - old_odo_data.odol) * meters_per_tick;
    
    callbackcount++;
    if (callbackcount > 25) {
      RCLCPP_DEBUG(this->get_logger(), "dr: %.4f, dl: %.4f, interval: %d",
                   static_cast<int>(odo->odor * meters_per_tick),
                   static_cast<int>(odo->odol * meters_per_tick),
                   odo->interval);
      RCLCPP_DEBUG(this->get_logger(), "dr: %d, dl: %d, interval: %d",
                   odo->odor, odo->odol, odo->interval);
      callbackcount = 0;
    }
    
    // Save the current encoder readings for the next callback.
    old_odo_data.odor = odo->odor;
    old_odo_data.odol = odo->odol;
    
    // Compute average travel distance and heading change.
    dxy_ave = (dr + dl) / 2.0;
    dth = (dr - dl) / ROBOT_LENGTH;
    vxy = dxy_ave / interval_time;
    vth = dth / interval_time;
    
    // Update the robot's estimated position.
    if (dxy_ave != 0) {
      dx = std::cos(dth) * dxy_ave;
      dy = -std::sin(dth) * dxy_ave;
      odomx += (std::cos(odomth) * dx - std::sin(odomth) * dy);
      odomy += (std::sin(odomth) * dx + std::cos(odomth) * dy);
    }
    if (dth != 0) {
      odomth += dth;
    }
    
    // Create a quaternion from the yaw angle using tf2.
    tf2::Quaternion q;
    q.setRPY(0, 0, odomth);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);
    
    // Prepare the odometry message.
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";
    
    msg.pose.pose.position.x = odomx;
    msg.pose.pose.position.y = odomy;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_quat;
    
    msg.twist.twist.linear.x = vxy;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.angular.z = vth;
    
    // Set pose and twist covariances depending on whether the robot is moving.
    if (vxy == 0 && vth == 0) {
      msg.pose.covariance = ODOM_POSE_COVARIANCE2;
      msg.twist.covariance = ODOM_TWIST_COVARIANCE2;
    } else {
      msg.pose.covariance = ODOM_POSE_COVARIANCE;
      msg.twist.covariance = ODOM_TWIST_COVARIANCE;
    }
    
    // Publish the odometry message.
    publisher_->publish(msg);
  }
  
  // Member variables.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<my_msg::msg::DriverOdo>::SharedPtr subscription_;
  
  // Odometry state.
  double odomx;
  double odomy;
  double odomth;
  my_msg::msg::DriverOdo old_odo_data;
  
  // Covariance arrays.
  const boost::array<double, 36> ODOM_POSE_COVARIANCE = {{
      1e-3, 0,    0,    0, 0, 0,
      0,    1e-3, 0,    0, 0, 0,
      0,    0,    1e6,  0, 0, 0,
      0,    0,    0,    1e6, 0, 0,
      0,    0,    0,    0, 1e6, 0,
      0,    0,    0,    0, 0,   1e3
  }};
  
  const boost::array<double, 36> ODOM_POSE_COVARIANCE2 = {{
      1e-9, 0,     0,    0, 0, 0,
      0,    1e-3,  1e-9, 0, 0, 0,
      0,    0,     1e6,  0, 0, 0,
      0,    0,     0,    1e6, 0, 0,
      0,    0,     0,    0, 1e6, 0,
      0,    0,     0,    0, 0,   1e-9
  }};
  
  const boost::array<double, 36> ODOM_TWIST_COVARIANCE = {{
      1e-3, 0,    0,    0, 0, 0,
      0,    1e-3, 0,    0, 0, 0,
      0,    0,    1e6,  0, 0, 0,
      0,    0,    0,    1e6, 0, 0,
      0,    0,    0,    0, 1e6, 0,
      0,    0,    0,    0, 0,   1e3
  }};
  
  const boost::array<double, 36> ODOM_TWIST_COVARIANCE2 = {{
      1e-9, 0,     0,    0, 0, 0,
      0,    1e-3,  1e-9, 0, 0, 0,
      0,    0,     1e6,  0, 0, 0,
      0,    0,     0,    1e6, 0, 0,
      0,    0,     0,    0, 1e6, 0,
      0,    0,     0,    0, 0,   1e-9
  }};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotOdomTimeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}