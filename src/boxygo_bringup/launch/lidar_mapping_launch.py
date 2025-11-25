from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    lidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    slam_pkg = get_package_share_directory('boxygo_localization')

    lidar_launch = os.path.join(lidar_pkg, 'launch', 'ydlidar_launch.py')
    slam_launch = os.path.join(slam_pkg, 'launch', 'mapping_launch.py')

    lidar_action = IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch))
    slam_action  = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch))

    return LaunchDescription([lidar_action, slam_action])