from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数
    map_file_path = LaunchConfiguration('map_file_path')
    map_file_path_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value='~/maps/map',
        description='Path and filename of the map to save (without extension)'
    )

    # 配置map_saver节点
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=['-f', map_file_path]
    )

    # 返回LaunchDescription
    return LaunchDescription([
        map_file_path_arg,
        map_saver_node
    ]) 