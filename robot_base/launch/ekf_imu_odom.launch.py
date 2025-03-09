from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # 获取参数文件路径
    config_file_path = os.path.join(
        get_package_share_directory('robot_base'),
        'params',
        'ekf_config.yaml'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_imu_odom',
            output='screen',
            parameters=[
                config_file_path,
                {'use_sim_time': use_sim_time}  # 添加use_sim_time参数
            ],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        )
    ]) 