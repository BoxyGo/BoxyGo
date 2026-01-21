from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import  Node 
import os 

def generate_launch_description():
    pkg_boxygo_controllers = FindPackageShare('boxygo_controllers') # Zakładam, że tu trzymasz launche

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_boxygo_controllers, 'launch', 'common_controllers_launch.py'])
        ),
        launch_arguments={
            'sim_mode': 'real',
            'use_sim_time': 'true',
            'start_control_node': 'true' # Rzeczywisty robot potrzebuje managera
        }.items()
    )

    imu_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
        remappings=[
            ("/imu/data_raw", "/camera/imu")
        ]
    )

    lidar_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py')))

    realsense_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'realsense_camera_launch.py')))

    return LaunchDescription([
        common_launch,
        lidar_launch,
        imu_node,
        realsense_launch
    ])