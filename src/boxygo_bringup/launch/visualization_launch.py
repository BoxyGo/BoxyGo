from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapping_rviz = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'rviz_launch.py')))

    return LaunchDescription([
        mapping_rviz
    ])
