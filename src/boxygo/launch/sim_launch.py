from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')

    urdf_path = os.path.join(pkg_boxygo, 'urdf', '6_wheel_robot.urdf.xacro') # 6_wheel_robot.urdf.xacro / new_robot.urdf.xacro
    controller_config = os.path.join(pkg_boxygo, 'config', 'diff_drive_controller.yaml')
    slam_params       = os.path.join(pkg_boxygo, 'config', 'slam_toolbox_params.yaml')
    world_path = os.path.join(pkg_boxygo, 'worlds', 'small_city.world')
    ekf_config = os.path.join(pkg_boxygo, 'config', 'ekf.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'use_sim_time': 'true'}.items()
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
                '-x', '-14.0',
                '-y', '0.0',
                '-z', '0.5',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '3.14' 
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_cont','--controller-manager','controller_manager','--param-file', controller_config],
                    remappings=[
                        ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
                    ],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),

        # TimerAction(
        #     period=8.0,
        #     actions=[
        #         Node(
        #             package='slam_toolbox',
        #             executable='sync_slam_toolbox_node',
        #             name='slam_toolbox',
        #             output='screen',
        #             parameters=[
        #                 slam_params,
        #                 {'use_sim_time': True}
        #             ],
        #             remappings=[
        #                 ('scan', '/scan'),
        #                 ('odom', '/odometry/filtered'), # 'diff_cont/odom'
        #             ]
        #         ),
        #     ]
        # ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}],
        ),

    ])
