import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory for the robot_base package.
    robot_base_share = get_package_share_directory('robot_base')
    
    # Declare a launch argument for simulation mode
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Set to "true" to run in simulation mode'
    )
    
    # Reference to the sim_mode argument
    sim_mode = LaunchConfiguration('sim_mode')
    
    # Include the start_tf launch file (assumed to be migrated to ROS 2 as a Python launch file).
    start_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_base_share, 'launch', 'start_tf.launch.py'))
    )
    
    # Conditionally include the rosserial node based on sim_mode
    rosserial_node = Node(
        package='rosserial_python',
        executable='serial_node',  # Adjust if the executable name is different in ROS 2.
        name='rosserial_stm32',
        output='screen',
        arguments=['_port:=/dev/stm32', '_baud:=1000000'],
        condition=UnlessCondition(sim_mode)
    )
    
    # Launch the robot_odom_time node from the robot_base package.
    robot_odom_time_node = Node(
        package='robot_base',
        executable='robot_odom_time',
        name='robot_odom_time',
        output='screen'
    )
    
    # Launch the robot_cmd node from the robot_base package.
    robot_cmd_node = Node(
        package='robot_base',
        executable='robot_cmd',
        name='robot_cmd',
        output='screen'
    )
    
    # Launch the robot_imu node from the robot_base package.
    robot_imu_node = Node(
        package='robot_base',
        executable='robot_imu',
        name='robot_imu',
        output='screen'
    )
    
    # Include the ekf_imu_odom launch file (assumed to be migrated as ekf_imu_odom.launch.py).
    ekf_imu_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_base_share, 'launch', 'ekf_imu_odom.launch.py'))
    )
    
    # Create and return the complete LaunchDescription.
    return LaunchDescription([
        sim_mode_arg,
        start_tf_launch,
        rosserial_node,
        robot_odom_time_node,
        robot_cmd_node,
        robot_imu_node,
        ekf_imu_odom_launch,
    ])