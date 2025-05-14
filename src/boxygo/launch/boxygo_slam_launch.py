import os

from launch import LaunchDescription
from launch.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    default_params_file = os.path.join(slam_toolbox_share, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
          default_params_file,
          {
            # To włącza korzystanie z zegara symulacji (/clock)
            'use_sim_time': True,
            # Temat, na którym SLAM Toolbox będzie oczekiwał LaserScan
            'scan_topic': '/scan',
          }
        ]
      )
    ])