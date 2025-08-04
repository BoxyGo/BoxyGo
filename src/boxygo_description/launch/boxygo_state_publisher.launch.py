#!/usr/bin/env python3
"""
Launch RViz visualization for the BoxyGo robot.

This launch file sets up the complete visualization environment for the BoxyGo robot,
including robot state publisher, joint state publisher, and RViz2. It handles loading
and processing of URDF/XACRO files and controller configurations.

:author: BoxyGo
:date: 05.08.2025
"""

from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, Command

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_boxygo = FindPackageShare('boxygo')
    urdf_path = Path(pkg_boxygo, 'urdf', '6_wheel_robot.urdf.xacro')

    robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': True
            }]
        )
    
    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_cmd)

    return ld