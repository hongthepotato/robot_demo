from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
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
    
    use_simulation = LaunchConfiguration('use_simulation')
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Use simulation environment if true'
    )
    
    # 获取参数文件路径
    config_file_path = os.path.join(
        get_package_share_directory('robot_base'),
        'params',
        'ekf_config.yaml'
    )
    
    # 创建EKF节点（模拟环境）
    ekf_node_sim = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_imu_odom',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}  # 添加use_sim_time参数
        ],
        remappings=[
            ('odometry/filtered', 'odom'),  # EKF节点的输出重映射到/odom
            ('/odom_input', '/diff_drive/odom')  # 在模拟环境中，将/odom_input重映射到/diff_drive/odom
        ],
        condition=IfCondition(use_simulation)  # 仅在use_simulation为true时启动
    )
    
    # 创建EKF节点（实际机器人环境）
    ekf_node_real = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_imu_odom',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}  # 添加use_sim_time参数
        ],
        remappings=[
            ('odometry/filtered', 'odom'),  # EKF节点的输出重映射到/odom
            ('/odom_input', '/odom_wheel')  # 在实际环境中，将/odom_input重映射到/odom_wheel
        ],
        condition=UnlessCondition(use_simulation)  # 仅在use_simulation为false时启动
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_simulation_arg,
        ekf_node_sim,
        ekf_node_real
    ]) 