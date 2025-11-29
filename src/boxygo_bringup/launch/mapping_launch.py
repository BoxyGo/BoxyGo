from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    contoller_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_moteus_control'), 'launch', 'joint_controller_launch.py')))

    slam_toolbox_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'mapping_launch.py')))

    lidar_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch_view.py')))

    realsense_splitter_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('isaac_ros_nvblox'), 'nvblox_examples', 'nvblox_examples_bringup' , 'launch', 'realsense_example.launch.py')))

    return LaunchDescription([
        contoller_launch,
        slam_toolbox_launch,
        lidar_launch,
        #realsense_splitter_launch
    ])
