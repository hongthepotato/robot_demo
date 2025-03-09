from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数 - 只使用一个参数控制仿真和时间
    use_simulation = LaunchConfiguration('use_simulation')
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='Use simulation if true, real robot if false (also controls use_sim_time)'
    )

    # 获取包路径
    robot_slam_pkg = FindPackageShare('robot_slam')
    robot_base_pkg = FindPackageShare('robot_base')
    
    # 包含STM32启动文件（控制是启动仿真还是实际机器人）
    stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_base_pkg, 'launch', 'start_stm32.launch.py'])
        ]),
        launch_arguments={
            'use_simulation': use_simulation
        }.items()
    )
    
    # 包含SLAM启动文件
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_slam_pkg, 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_simulation,  # 使用use_simulation作为use_sim_time
            'slam_params_file': PathJoinSubstitution([robot_slam_pkg, 'config', 'mapper_params_online_async.yaml'])
        }.items()
    )

    # 返回LaunchDescription
    return LaunchDescription([
        use_simulation_arg,
        stm32_launch,
        slam_launch
    ]) 