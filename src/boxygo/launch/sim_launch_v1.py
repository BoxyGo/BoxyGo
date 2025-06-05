from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')

    urdf_path = os.path.join(pkg_boxygo, 'urdf', '6_wheel_robot_v3.urdf.xacro') 
    controller_config = os.path.join(pkg_boxygo, 'config', 'diff_drive_controller_v1.yaml')
    world_path = os.path.join(pkg_boxygo, 'worlds', 'small_city.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': True
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'delivery_robot',
                '-topic', 'robot_description',
                '-x', '1.0',
                '-y', '1.0',
                '-z', '0.5',
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad', '--controller-manager', '/controller_manager', '--param-file', controller_config],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', '--controller-manager', '/controller_manager', '--param-file', controller_config],
            remappings=[
                ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
            ],
            output='screen'
        ),
    ])

