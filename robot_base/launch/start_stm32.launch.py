from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 仿真组
    sim_group = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('robot_model'),
                        'launch',
                        'launch_sim.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # 真实机器人组
    real_robot_group = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                name='micro_ros_agent',
                output='screen',
                arguments=['serial', '--dev', '/dev/stm32', '--baud', '1000000']
            ),
            Node(
                package='robot_base',
                executable='robot_odom_time',
                name='robot_odom_time',
                output='screen'
            ),
            Node(
                package='robot_base',
                executable='robot_cmd',
                name='robot_cmd',
                output='screen'
            ),
            Node(
                package='robot_base',
                executable='robot_imu',
                name='robot_imu',
                output='screen'
            )
        ]
    )
    
    # EKF launch文件
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_base'),
                'launch',
                'ekf_imu_odom.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        sim_group,
        real_robot_group,
        ekf_launch
    ]) 