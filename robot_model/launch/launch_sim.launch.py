import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    ## Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name = 'robot_model'
    
    # 获取包路径
    pkg_share = get_package_share_directory(package_name)
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 自定义world文件路径
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # 声明launch参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 启动robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 启动Gazebo服务器
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # 启动Gazebo客户端
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # 运行spawn_entity.py脚本
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'robot_model'],
        output='screen'
    )

    # 创建并返回launch描述
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_use_sim_time)
    
    # 添加节点和进程
    ld.add_action(rsp)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_entity)
    
    return ld