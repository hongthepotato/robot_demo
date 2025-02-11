#include "ros/ros.h"
#include "std_msgs/Int8.h"

int main(int argc, char  *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"robot_rod_cmd");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<std_msgs::Int8>("rod_cmd", 1);	
    std_msgs::Int8 cmd;
    cmd .data=0;
    pub.publish(cmd);
    ros::spinOnce();

    return 0;
}