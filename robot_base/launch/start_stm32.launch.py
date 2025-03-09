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
    # 声明参数 - 只使用一个参数控制仿真和时间
    use_simulation = LaunchConfiguration('use_simulation')
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='Use simulation if true, real robot if false (also controls use_sim_time)'
    )
    
    # 仿真组
    sim_group = GroupAction(
        condition=IfCondition(use_simulation),
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
                    'use_sim_time': use_simulation  # 使用use_simulation作为use_sim_time
                }.items()
            )
        ]
    )
    
    # 真实机器人组
    real_robot_group = GroupAction(
        condition=UnlessCondition(use_simulation),
        actions=[
            # 包含robot_state_publisher启动文件，确保TF树完整
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('robot_model'),
                        'launch',
                        'rsp.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_simulation  # 使用use_simulation作为use_sim_time
                }.items()
            ),
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
        ]),
        launch_arguments={
            'use_sim_time': use_simulation,  # 使用use_simulation作为use_sim_time
            'use_simulation': use_simulation  # 传递use_simulation参数
        }.items()
    )
    
    return LaunchDescription([
        use_simulation_arg,
        sim_group,
        real_robot_group,
        ekf_launch
    ]) 