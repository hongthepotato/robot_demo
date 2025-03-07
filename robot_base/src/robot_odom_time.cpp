#include "rclcpp/rclcpp.hpp"
#include "my_msg/msg/driver_odo.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include "robot_base/robot_base.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotOdomTimeNode : public rclcpp::Node
{
public:
  RobotOdomTimeNode() : Node("robot_base_odom"), odomx(0.0), odomy(0.0), odomth(0.0)
  {
    // 初始化协方差矩阵
    ODOM_POSE_COVARIANCE = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e3
    };

    ODOM_POSE_COVARIANCE2 = {
      1e-9, 0, 0, 0, 0, 0,
      0, 1e-3, 1e-9, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e-9
    };

    ODOM_TWIST_COVARIANCE = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e3
    };

    ODOM_TWIST_COVARIANCE2 = {
      1e-9, 0, 0, 0, 0, 0,
      0, 1e-3, 1e-9, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e-9
    };
    
    // 初始化里程计数据
    old_odo_data.odol = 0;
    old_odo_data.odor = 0;
    
    // 创建发布者
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_wheel", 50);
    
    // 创建订阅者
    subscription_ = this->create_subscription<my_msg::msg::DriverOdo>(
      "stm32_odo", 2, 
      std::bind(&RobotOdomTimeNode::callback, this, std::placeholders::_1));
    
    // 等待底层初始化
    RCLCPP_INFO(this->get_logger(), "等待底层初始化...");
    rclcpp::sleep_for(std::chrono::seconds(10));
    RCLCPP_INFO(this->get_logger(), "初始化完成");
  }

private:
  void callback(const my_msg::msg::DriverOdo::SharedPtr odo)
  {
    static int callbackcount = 0;

    double dr, dl, dxy_ave, dth, vxy, vth, dx, dy;
    auto current_time_ = this->now();
    double interval_time = (odo->interval) / 1000.0f;
    dr = (odo->odor - old_odo_data.odor) * meters_per_tick;
    dl = (odo->odol - old_odo_data.odol) * meters_per_tick;
    
    callbackcount++;
    if(callbackcount > 25)
    {
      RCLCPP_DEBUG(this->get_logger(), "dr:%.4f,dl:%.4f,interval:%d",
                  (odo->odor) * meters_per_tick, (odo->odol) * meters_per_tick, odo->interval);
      RCLCPP_DEBUG(this->get_logger(), "dr:%d,dl:%d,interval:%d",
                  (odo->odor), (odo->odol), odo->interval);
      callbackcount = 0;
    }

    old_odo_data.odor = odo->odor;
    old_odo_data.odol = odo->odol;

    dxy_ave = (dr + dl) / 2;
    dth = (dr - dl) / ROBOT_LENGTH;
    vxy = dxy_ave / interval_time;
    vth = dth / interval_time;
    
    if (dxy_ave != 0)
    {
      dx = cos(dth) * dxy_ave;
      dy = -sin(dth) * dxy_ave;
      odomx += (cos(odomth) * dx - sin(odomth) * dy);
      odomy += (sin(odomth) * dx + cos(odomth) * dy);
    }
    
    if (dth != 0)
    {
      odomth += dth;
    }
    
    // 创建四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, odomth);
    
    // 发布里程计消息
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = odomx;
    odom_msg.pose.pose.position.y = odomy;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vxy;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = vth;

    if (vxy == 0 && vth == 0)
    {
      for (size_t i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = ODOM_POSE_COVARIANCE2[i];
        odom_msg.twist.covariance[i] = ODOM_TWIST_COVARIANCE2[i];
      }
    }
    else
    {
      for (size_t i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = ODOM_POSE_COVARIANCE[i];
        odom_msg.twist.covariance[i] = ODOM_TWIST_COVARIANCE[i];
      }
    }

    odom_publisher_->publish(odom_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<my_msg::msg::DriverOdo>::SharedPtr subscription_;
  
  my_msg::msg::DriverOdo old_odo_data;
  double odomx, odomy, odomth;
  std::array<double, 36> ODOM_POSE_COVARIANCE;
  std::array<double, 36> ODOM_POSE_COVARIANCE2;
  std::array<double, 36> ODOM_TWIST_COVARIANCE;
  std::array<double, 36> ODOM_TWIST_COVARIANCE2;
};

int main(int argc, char* argv[])
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotOdomTimeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}