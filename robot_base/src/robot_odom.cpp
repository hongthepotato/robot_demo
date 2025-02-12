#include "rclcpp/rclcpp.hpp"

// Use the ROS 2 style header for custom message.
#include "my_msg/msg/driver_odo.hpp"

// Include the header that defines robot-specific constants
#include "robot_base/robot_base.h"

// ROS2 message headers
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

// TF2 includes for ROS 2 transform broadcasting
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Standard libraries
#include <sstream>
#include <cmath>
#include <boost/array.hpp>

class RobotOdomNode : public rclcpp::Node
{
    public:
        RobotOdomNode()
        : Node("robot_base_odom"),
          odomx(0.0),
          odomy(0.0),
          odomth(0.0)
        {
            // Create a publisher for Odometry messages on the "odom" topic
            pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);

            // Create a a subsrciption to custom odometry messages from the low-level node.
            subscription_ = this->create_subscription<my_msg::msg::DriverOdo>("stm32_odo", 2,
                std::bind(&RobotOdomNode::callback, this, std::placeholders::_1));

            // Create the TF broadcaster
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            // Optionally, wait a bit for the lower-level hardware to initialize.
            // (Avoid long sleeps in production.)
            rclcpp::sleep_for(std::chrono::seconds(10));
        }

    private:
        void callback(const my_msg::msg::DriverOdo::SharedPtr odo)
        {
            static int callbackcount = 0;

            double dr, dl, dxy_ave, dth, vxy, vth, dx, dy;
            // Get the current time from the node's clock
            rclcpp::Time current_time = this->now();

            // Compute the time interval in seconds (assuming odo-> interval is in milliseconds)
            double interval_time = static_cast<double>(odo->interval) / 1000.0;

            // Calculate differences based on encoder ticks.
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

            // Save the current encoder reading for the next callback.
            old_odo_data.odor = odo->odor;
            old_odo_data.odol = odo->odol;

            // Compute average distance and change in heading.
            dxy_ave = (dr + dl) / 2.0;
            dth = (dr - dl) / ROBOT_LENGTH;
            vxy = dxy_ave / interval_time;
            vth = dth / interval_time;

            // Update the odometry pose.
            if (dxy_ave !=0) {
                dx = std::cos(dth) * dxy_ave;
                dy = -std::sin(dth) * dxy_ave;
                odomx += (std::cos(odomth) * dx - std::sin(odomth) * dy);
                odomy += (std::sin(odomth) * dx + std::cos(odomth) * dy);
            }
            if (dth != 0) {
                odomth += dth;
            }

            // Create a quaternion from the yaw angle using tf2
            tf2::Quaternion q;
            q.setRPY(0, 0, odomth);
            geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

            // Create and populate the transform stamped message for TF broadcasting.
            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";
            odom_trans.transform.translation.x = odomx;
            odom_trans.transform.translation.y = odomy;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // Broadcasting the transform
            tf_broadcaster_->sendTransform(odom_trans);

            // Create and pupolate the Odometry message.
            nav_msgs::msg::Odometry msgl;
            msgl.header.stamp = current_time;
            msgl.header.frame_id = "odom";
            msgl.child_frame_id = "base_footprint";
            msgl.pose.pose.position.x = odomx;
            msgl.pose.pose.position.y = odomy;
            msgl.pose.pose.position.z = 0.0;
            msgl.pose.pose.orientation = odom_quat;

            // Copy the covariance data.
            for (size_t i = 0; i < 36; ++i) {
                msgl.pose.covariance[i] = odom_pose_covariance[i];
                msgl.twist.covariance[i] = odom_twist_covariance[i];
            }

            msgl.twist.twist.linear.x = vxy;
            msgl.twist.twist.linear.y = 0.0;
            msgl.twist.twist.angular.z = vth;

            // Publish the odometry message
            pub_->publish(msgl);
        }

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
        rclcpp::Subscription<my_msg::msg::DriverOdo>::SharedPtr subscription_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      
        my_msg::msg::DriverOdo old_odo_data;
        
        double odomx;
        double odomy;
        double odomth;
      
        std::array<double, 36> odom_pose_covariance = {{
            1e-9, 0,    0,    0, 0, 0,
            0,    1e-3, 1e-9, 0, 0, 0,
            0,    0,    1e6,  0, 0, 0,
            0,    0,    0,    1e6, 0, 0,
            0,    0,    0,    0, 1e6, 0,
            0,    0,    0,    0, 0, 1e-9
        }};
      
        std::array<double, 36> odom_twist_covariance = {{
            1e-9, 0,    0,    0, 0, 0,
            0,    1e-3, 1e-9, 0, 0, 0,
            0,    0,    1e6,  0, 0, 0,
            0,    0,    0,    1e6, 0, 0,
            0,    0,    0,    0, 1e6, 0,
            0,    0,    0,    0, 0, 1e-9
        }};
};

int main(int argc, char *argv[])
{
  // Initialize ROS 2.
  rclcpp::init(argc, argv);

  // Create the node.
  auto node = std::make_shared<RobotOdomNode>();

  // Spin the node.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
