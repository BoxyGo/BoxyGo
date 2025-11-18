from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    boxygo_localization_pkg = get_package_share_directory('boxygo_localization')
    slam_params = os.path.join(boxygo_localization_pkg, 'config', 'slam_toolbox_params.yaml')
    ekf_config = os.path.join(boxygo_localization_pkg, 'config', 'ekf.yaml')

    return LaunchDescription([

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        slam_params,
                        {'use_sim_time': True}
                    ],
                    remappings=[
                        ('scan', '/scan'),
                        ('odom', '/odometry/filtered'),
                    ]
                ),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(ekf_config, 'rviz_slam.rviz')],
            parameters=[{'use_sim_time': True}]
        ),

    ])
