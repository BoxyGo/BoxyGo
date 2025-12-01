from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    boxygo_localization_pkg = get_package_share_directory('boxygo_localization')
    rviz_config = os.path.join(boxygo_localization_pkg, 'rviz', 'rviz_slam.rviz')

    return LaunchDescription([

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),

    ])
