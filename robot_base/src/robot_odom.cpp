#include "rclcpp/rclcpp.hpp"

// Use the ROS 2 style header for custom message.
#include "my_msg/msg/driver_odo.hpp" ## This line may be problematic

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
            odom_trans.transform.roration = odom_quat;

            // Broadcasting the transform
            tf_broadcaster_->sendTransform(odom_trans);

            // Create and pupolate the Odometry message.
            nav_msgs::msg::Odometry msgl;
            msgl.header.stamp = current_time;
            msgl.header.frame_id = "odom";
            msgl.child.frame_id = "base_footprint";
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
      
        boost::array<double, 36> odom_pose_covariance = {{
            1e-9, 0,    0,    0, 0, 0,
            0,    1e-3, 1e-9, 0, 0, 0,
            0,    0,    1e6,  0, 0, 0,
            0,    0,    0,    1e6, 0, 0,
            0,    0,    0,    0, 1e6, 0,
            0,    0,    0,    0, 0, 1e-9
        }};
      
        boost::array<double, 36> odom_twist_covariance = {{
            1e-9, 0,    0,    0, 0, 0,
            0,    1e-3, 1e-9, 0, 0, 0,
            0,    0,    1e6,  0, 0, 0,
            0,    0,    0,    1e6, 0, 0,
            0,    0,    0,    0, 1e6, 0,
            0,    0,    0,    0, 0, 1e-9
        }};
}













ros::Publisher pub;
ros::Time last_time_;
my_msg::driver_odo old_odo_data;
double odomx,odomy,odomth;

boost::array<double, 36> odom_pose_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};

// boost::array<double, 36> odom_pose_covariance ={{1e3, 0, 0, 0, 0, 0,
//                                                 0, 1e3, 0, 0, 0, 0,
//                                                 0, 0, 1e3, 0, 0, 0,
//                                                 0, 0, 0, 1e3, 0, 0,
//                                                 0, 0, 0, 0, 1e3, 0,
//                                                 0, 0, 0, 0, 0, 1e3}};
// boost::array<double, 36> odom_twist_covariance={{1e3, 0, 0, 0, 0, 0,
//                                                 0, 1e3, 0, 0, 0, 0,
//                                                 0, 0, 1e3, 0, 0, 0,
//                                                 0, 0, 0, 1e3, 0, 0,
//                                                 0, 0, 0, 0, 1e3, 0,
//                                                 0, 0, 0, 0, 0, 1e3}};

void callback(const my_msg::driver_odo::ConstPtr& odo)
{
    // ROS_INFO("odo数据为l:%d,r:%d",odo->odol , odo->odor);
    
 

    static int callbackcount = 0;

    double dr,dl,dxy_ave,dth,vxy,vth,dx,dy;
    ros::Time current_time_ = ros::Time::now();
    double interval_time = (odo->interval) / 1000.0f;
    dr = (odo->odor - old_odo_data.odor) * meters_per_tick;
    dl = (odo->odol - old_odo_data.odol) * meters_per_tick;
    
    callbackcount ++;
    if(callbackcount > 25)
    {
        // ROS_DEBUG();
        ROS_DEBUG("dr:%.4f,dl:%.4f,interval:%d",(odo->odor) * meters_per_tick ,(odo->odol) * meters_per_tick,odo->interval);
        ROS_DEBUG("dr:%d,dl:%d,interval:%d",(odo->odor),(odo->odol),odo->interval);
        // ROS_INFO("dr:%f,dl:%f,interval:%.3f",dr,dl,interval_time);
        callbackcount =0;
    }

    old_odo_data.odor = odo->odor;
    old_odo_data.odol = odo->odol;

    dxy_ave = (dr +dl )/2;
    dth = (dr -dl ) / ROBOT_LENGTH;
    vxy = dxy_ave / interval_time;
    vth = dth / interval_time;
    if (dxy_ave != 0)
    {
        dx = cos(dth) * dxy_ave;
        dy = -sin(dth) * dxy_ave;
        odomx += (cos(odomth) * dx -sin(odomth) * dy);
        odomy += (sin(odomth) * dx +cos(odomth) * dy);
    }
    if (dth!=0)
    {
        odomth += dth;
    }
    static geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(odomth);
    // 发布TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_footprint";

    odom_trans.transform.translation.x = odomx;
    odom_trans.transform.translation.y = odomy;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    static tf::TransformBroadcaster odom_broadcaster_;
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = current_time_;
    msgl.header.frame_id = "odom";
    msgl.child_frame_id = "base_footprint";

    msgl.pose.pose.position.x = odomx;
    msgl.pose.pose.position.y = odomy;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.twist.twist.linear.x = vxy;
    msgl.twist.twist.linear.y = 0;
    msgl.twist.twist.angular.z = vth;
    msgl.twist.covariance = odom_twist_covariance;

    pub.publish(msgl);

    // ROS_INFO("odor:%d,odol:%d",old_odo_data.odor,old_odo_data.odol);
    // ROS_INFO("vxy:%f,vth:%f",vxy,vth);

}

int main(int argc, char  *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"robot_base_odom");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::Odometry>("odom", 50);	
    ros::Subscriber sub =nh.subscribe<my_msg::driver_odo>("stm32_odo",2,callback,ros::TransportHints().tcpNoDelay());

    old_odo_data.odol = 0;
    old_odo_data.odor = 0;

    ros::Duration(10).sleep();          //等待底层初始化
    // last_time_ = ros::Time::now();

    ros::spin();
    return 0;
}