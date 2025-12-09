from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_gazebo'), 'launch', 'gazebo_launch.py')))

    slam_toolbox_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'mapping_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items())

    mapping_rviz = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('boxygo_localization'),'launch','mapping_rviz_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items())

    return LaunchDescription([
        gazebo_launch,
        slam_toolbox_launch,
        mapping_rviz
    ])
