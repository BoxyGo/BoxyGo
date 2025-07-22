from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import SetRemap

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')
    map_file = os.path.join(pkg_boxygo, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_boxygo, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        
        TimerAction(
            period=5.0,
            actions=[
                SetRemap(src='/cmd_vel', dst='/diff_cont/cmd_vel_unstamped'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={'params_file': params_file, 'map': map_file, 'use_sim_time': 'true', 'autostart': 'true'}.items()
                ),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_boxygo, 'config', 'rviz_nav2.rviz')],
            parameters=[{'use_sim_time': True}]
        ),
    ])
