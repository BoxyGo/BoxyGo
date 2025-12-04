from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    moteus_controller_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([ FindPackageShare('boxygo_moteus_control'), 'launch', 'moteus_controller_launch.py'])))
    
    realsense_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([ FindPackageShare('boxygo_bringup'), 'launch', 'realsense_camera_launch.py'])))
   
    vslam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([ FindPackageShare('boxygo_bringup'), 'launch', 'vslam_launch.py'])))
   
    return LaunchDescription([GroupAction([moteus_controller_launch, realsense_launch, vslam_launch])])
      
    