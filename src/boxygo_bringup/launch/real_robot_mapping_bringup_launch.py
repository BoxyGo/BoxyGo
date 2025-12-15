from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    contoller_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_moteus_control'), 'launch', 'moteus_controller_launch.py')))

    slam_toolbox_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'mapping_launch.py')))

    lidar_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py')))

    realsense_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'realsense_camera_launch.py')))

    return LaunchDescription([
        contoller_launch,
        slam_toolbox_launch,
        lidar_launch,
        realsense_launch
    ])
