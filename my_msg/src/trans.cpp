#include "ros/ros.h"
#include "my_msg/driver_odo.h"
#include "std_msgs/String.h"
#include <sstream>

ros::Publisher pub;

void callback(const my_msg::driver_odo::ConstPtr& odo)
{
    ROS_INFO("odo数据为l:%d,r:%d",odo->odol , odo->odor);

    std_msgs::String pubstr;

    std::stringstream ss;
    ss << "odo数据为l:" << odo->odol << ",r:" << odo->odor;
    pubstr.data = ss.str();

    pub.publish(pubstr);

}

int main(int argc, char  *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"rostalker");
    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::String>("stm32odo",10);

    ros::Subscriber sub =nh.subscribe<my_msg::driver_odo>("stm32_odo",10,callback);

    ros::spin();

    return 0;
}
