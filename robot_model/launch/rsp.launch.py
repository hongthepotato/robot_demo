from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # 获取包路径
    robot_model_pkg = FindPackageShare('robot_model')
    
    # 获取机器人描述文件路径
    xacro_file = PathJoinSubstitution([robot_model_pkg, 'description', 'robot.urdf.xacro'])
    
    # 使用xacro处理URDF - 修复命令格式
    robot_description = Command(['xacro ', xacro_file])

    # 配置joint_state_publisher节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50.0
        }]
    )

    # 配置robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )

    # 返回LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        joint_state_publisher_node,
        robot_state_publisher_node
    ]) 