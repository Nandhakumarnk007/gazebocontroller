#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    package_name = 'mobilerobot'
    pkg_share_dir = get_package_share_directory(package_name)

    # Path to your robot's URDF/Xacro file
    urdf_file_name = 'mobilebot.urdf.xacro'
    urdf_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)

    # Process the URDF file
    robot_description_content = Command(['xacro ', urdf_path])
    
    # Create a parameter for robot_description
    robot_description_param = {'robot_description': robot_description_content}


    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': True}]
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 
            'launch', 'gazebo.launch.py'
        )])
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobilebot'
        ],
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])