#include "ros/ros.h"
#include "my_msg/driver_control.h"
#include "robot_base/robot_base.h"
#include <geometry_msgs/Twist.h>
// #include <ros/time.h>

ros::Publisher pub;

void cmdCallback(const geometry_msgs::Twist& msg)
{
    double RobotV_,RobotYawRate_,speedl,speedr;
    my_msg::driver_control dc_cmd;
    dc_cmd.mod = 3;
    dc_cmd.quick_stop =0;

    RobotV_  = msg.linear.x;//m/s
    RobotYawRate_ = msg.angular.z;//rad/s
    // ROS_INFO("RobotV_:%f,RobotYawRate_:%f",RobotV_,RobotYawRate_);
    double r = RobotV_ / RobotYawRate_;//m

    if(RobotV_ == 0)      //旋转
    {
        speedl  =(-RobotYawRate_ * ROBOT_RADIUS);//m/s
        speedr =(RobotYawRate_ * ROBOT_RADIUS);//m/s
    } 
    else if(RobotYawRate_ == 0)//直线
    {
        speedl =RobotV_;//m/s
        speedr = RobotV_;
    }
    else//速度不一致
    {
        speedl  = (RobotYawRate_ * (r - ROBOT_RADIUS));//m/s
        speedr = (RobotYawRate_ * (r + ROBOT_RADIUS));
    }
    speedl = speedl / length_per_circle * 60.0f;  
    speedr = speedr / length_per_circle * 60.0f;  

    if(speedl <1 && speedl>-1)
        if(speedl <-0.05) speedl = -1;
        else if(speedl >0.05) speedl = 1;
    if(speedr <1 && speedr>-1)
        if(speedr <-0.05) speedr = -1;
        else if(speedr >0.05) speedr = 1;

    dc_cmd.speedl = (int16_t)speedl;
    dc_cmd.speedr = (int16_t)speedr;

    pub.publish(dc_cmd);
    // ROS_INFO("speedr:%d,speedl:%d",dc_cmd.speedr,dc_cmd.speedl);
}

int main(int argc, char  *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"robot_vel_cmd");
    ros::NodeHandle nh;
    ros::Subscriber sub =nh.subscribe("cmd_vel", 50, cmdCallback);
    pub = nh.advertise<my_msg::driver_control>("dc_cmd",5);

    my_msg::driver_control dc_cmd;
    ros::Duration(6).sleep();
    dc_cmd.mod = 0x04;      //清理编码器数据
    dc_cmd.quick_stop= 0;
    pub.publish(dc_cmd);
    ROS_INFO("send clear encode");
    ros::spin();
    return 0;
}