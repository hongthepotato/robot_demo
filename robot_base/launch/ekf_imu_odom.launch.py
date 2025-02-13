import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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
        parameters=[config_file],
        remappings=[('odometry/filtered', 'odom')]
        # If needed, add additional remappings here
    )

    return LaunchDescription([ekf_node])