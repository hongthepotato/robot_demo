import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node to publish static transform from base_footprint to laser
    rplidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar2basefootprint',
        output='screen',
        arguments=[
            '0.1086', '0', '0.28',    # Translation: x, y, z
            '3.1415926535', '0', '0',  # Rotation: roll, pitch, yaw (in radians)
            '/base_footprint', '/laser'  # Parent frame, Child frame
        ]
    )

    # Node to publish static transform from base_footprint to imu_link
    imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link2basefootprint',
        output='screen',
        arguments=[
            '0.075', '-0.05', '0.1',   # Translation: x, y, z
            '3.1415926535', '0', '0',   # Rotation: roll, pitch, yaw (in radians)
            '/base_footprint', '/imu_link'  # Parent frame, Child frame
        ]
    )

    # Node to publish static transform from base_footprint to wheel_link
    wheel_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wheel_link2basefootprint',
        output='screen',
        arguments=[
            '0', '0', '0.01',   # Translation: x, y, z
            '0', '0', '0',      # Rotation: roll, pitch, yaw (in radians)
            '/base_footprint', '/wheel_link'  # Parent frame, Child frame
        ]
    )

    return LaunchDescription([
        rplidar_node,
        imu_node,
        wheel_node,
    ])