import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明use_sim_time参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 声明launch参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Locate the configuration file within the 'robot_base' package.
    config_file = os.path.join(
        get_package_share_directory('robot_base'),
        'param',
        'ekf_config.yaml'
    )

    # Create the EKF node from robot_localization package.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_imu_odom',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom')]
        # If needed, add additional remappings here
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node
    ])