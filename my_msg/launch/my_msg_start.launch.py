import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the rosserial node (Python executable)
    rosserial_node = Node(
        package='rosserial_python',
        executable='serial_node',  # In ROS 2, you typically omit the .py extension.
        name='rosserial_stm32',
        output='screen',
        arguments=['_port:=/dev/ttyACM0', '_baud:=115200']
    )

    # Uncomment the following block if you want to launch the trans node from my_msg package.
    # trans_node = Node(
    #     package='my_msg',
    #     executable='trans',
    #     name='trans_odo_data',
    #     output='screen'
    # )

    return LaunchDescription([
        rosserial_node,
        # trans_node  # Uncomment if needed.
    ])