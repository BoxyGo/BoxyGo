from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    pkg_boxygo_controllers = FindPackageShare('boxygo_controllers') # Zakładam, że tu trzymasz launche

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_boxygo_controllers, 'launch', 'common_controllers.launch.py'])
        ),
        launch_arguments={
            'sim_mode': 'real',
            'use_sim_time': 'false',
            'start_control_node': 'true' # Rzeczywisty robot potrzebuje managera
        }.items()
    )

           
    lidar_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')))

    realsense_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'realsense_camera_launch.py')))
    

    return LaunchDescription([
        common_launch,
        lidar_launch,
        realsense_launch
    ])