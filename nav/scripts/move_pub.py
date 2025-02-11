#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from nav.msg import driver_control


if __name__ == "__main__":
    #1.初始化 ROS 节点
    rospy.init_node("move")
    #2.创建发布者对象
    pub = rospy.Publisher("dc_cmd",driver_control,queue_size=10)
    #3.组织消息
    p = driver_control()
    p.mod = 1
    p.quick_stop = 0
    p.speedl = 1
    p.speedr = 1
    p.posl=0
    p.posr=0
    #4.编写消息发布逻辑
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(p)  #发布消息
        rate.sleep()  #休眠
        rospy.loginfo("工作模式:%d, 是否急停:%d, 左电机速度:%d, 右电机速度:%d, 左电机位置值:%d, 右电机位置值:%d",p.mod,p.quick_stop,p.speedl,p.speedr,p.posl,p.posr)
