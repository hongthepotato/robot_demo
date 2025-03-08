from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # 获取包路径
    robot_model_pkg = FindPackageShare('robot_model')
    
    # 包含robot_state_publisher启动文件
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_model_pkg, 'launch', 'rsp.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 启动Gazebo Classic
    gazebo_pkg = FindPackageShare('gazebo_ros')
    world_file = PathJoinSubstitution([robot_model_pkg, 'config', 'easy', 'easy.world'])
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'paused': 'false',
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # 获取机器人描述文件路径
    xacro_file = PathJoinSubstitution([robot_model_pkg, 'description', 'robot.urdf.xacro'])
    
    # 使用xacro处理URDF - 修复命令格式
    robot_description = Command(['xacro ', xacro_file])

    # 在Gazebo中生成机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'robot_model',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # 返回LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        rsp_launch,
        gazebo_launch,
        spawn_entity_node
    ]) 