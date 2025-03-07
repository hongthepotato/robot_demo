from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件路径
    config_file_path = os.path.join(
        get_package_share_directory('robot_base'),
        'params',
        'ekf_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_imu_odom',
            output='screen',
            parameters=[config_file_path],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        )
    ]) 