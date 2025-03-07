from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 使用micro-ROS代替rosserial
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyACM0', '--baud', '115200']
        ),
        # 启用trans节点
        # Node(
        #     package='my_msg',
        #     executable='trans',
        #     name='trans_odo_data',
        #     output='screen'
        # )
    ]) 