#include "ros/ros.h"
#include "my_msg/driver_odo.h"
#include "std_msgs/String.h"
#include <sstream>
#include "robot_base/robot_base.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

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