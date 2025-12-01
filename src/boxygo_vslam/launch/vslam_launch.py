from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    visual_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('isaac_ros_visual_slam'),
                'launch',
                'isaac_ros_visual_slam_realsense.launch.py'
            )
        )
    )

    return LaunchDescription([
        visual_slam_launch
    ])