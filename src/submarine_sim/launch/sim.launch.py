import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'submarine_sim'

    # File Paths
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'robot.xacro')
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'underwater.world')
    joy_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'joystick.yaml')

    # 1. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # 2. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 3. Spawn Robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'submarine_robot', '-z', '1.0'],
        output='screen'
    )

    # ==========================================
    # TAMBAHAN: JOYSTICK XBOX NODES
    # ==========================================
    
    # Node 1: Membaca raw data dari hardware joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0', # Port joystick default
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )

    # Node 2: Mengubah data joystick jadi cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
        remappings=[('/cmd_vel', '/cmd_vel')] # Pastikan topic sama dengan plugin C++
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn,
        joy_node,
        teleop_node
    ])