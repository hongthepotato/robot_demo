#ifndef ROBOT_ODOM_H
#define ROBOT_ODOM_H

// #include "ros/ros.h"
// #include "my_msg/driver_odo.h"
// #include "robot_odom/robot_odom.h"

#define length_per_circle   0.32986722862692829003857755524435    //m   105mm
// #define length_per_circle   0.3361504139341078 //m  107mm
// #define ticks_per_meter     0.00032213596545598465824079839379331
// #define meters_per_tick     0.00032213596545598465824079839379331
#define meters_per_tick     0.000080533991363996164560199598448327
#define ROBOT_LENGTH        0.37        //m    wheel track
#define ROBOT_RADIUS        0.185       //m

// namespace robot
// {
//     class robot_odom
//     {
    
//         public:
//             ros::NodeHandle nh;
//             os::Publisher pub;
//             ros::Time last_time_;
//             my_msg::driver_odo old_odo_data;
//             double odomx,odomy,odomth;
//             tf::TransformBroadcaster odom_broadcaster_;
//             boost::array<double, 36> odom_pose_covariance = {
//                 {1e-9, 0, 0, 0, 0, 0, 
//                 0, 1e-3,1e-9, 0, 0, 0, 
//                 0, 0, 1e6, 0, 0, 0,
//                 0, 0, 0, 1e6, 0, 0, 
//                 0, 0, 0, 0, 1e6, 0, 
//                 0, 0, 0, 0, 0, 1e-9}};
//             boost::array<double, 36> odom_twist_covariance = {
//                 {1e-9, 0, 0, 0, 0, 0, 
//                 0, 1e-3,1e-9, 0, 0, 0, 
//                 0, 0, 1e6, 0, 0, 0, 
//                 0, 0, 0, 1e6, 0, 0, 
//                 0, 0, 0, 0, 1e6, 0, 
//                 0, 0, 0, 0, 0, 1e-9}};
//             void callback(const my_msg::driver_odo::ConstPtr& odo);
//             void init();
//         private:



//     };
// }




#endif