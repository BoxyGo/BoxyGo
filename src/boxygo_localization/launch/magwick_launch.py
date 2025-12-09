from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('boxygo_localization')

    realsense_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource( os.path.join(get_package_share_directory('boxygo_localization'), 'launch', 'realsense_camera_launch.py')))

    imu_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[
            {"use_mag": False},
            {"fixed_frame": "camera_link"}
        ],
        remappings=[
            ("/imu/data_raw", "/camera/imu")
        ]
    )

    return LaunchDescription([
        realsense_launch,
        imu_node
    ])
