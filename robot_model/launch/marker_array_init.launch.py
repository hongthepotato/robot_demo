from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 启动一个节点，用于确保MarkerArray中的四元数正确初始化
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='frontier_orientation_initializer',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'frontier']
    )

    # 返回LaunchDescription
    return LaunchDescription([
        static_transform_publisher_node
    ]) 