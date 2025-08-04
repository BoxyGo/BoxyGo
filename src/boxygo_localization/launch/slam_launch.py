from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')
    slam_params = os.path.join(pkg_boxygo, 'config', 'slam_toolbox_params.yaml')

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
                        ('odom', '/odometry/filtered'), # 'diff_cont/odom'
                    ]
                ),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_boxygo, 'config', 'rviz_slam.rviz')],
            parameters=[{'use_sim_time': True}]
        ),

    ])
