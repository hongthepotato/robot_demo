"""
机器人启动文件：
    当不包含机器人模型时，需要发布坐标变换
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rplidar2basefootprint',
            arguments=['0.1086', '0', '0.28', '3.1415926535', '0', '0', 'base_footprint', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_link2basefootprint',
            arguments=['0.075', '-0.05', '0.1', '3.1415926535', '0', '0', 'base_footprint', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='wheel_link2basefootprint',
            arguments=['0', '0', '0.01', '0', '0', '0', 'base_footprint', 'wheel_link']
        )
    ]) 