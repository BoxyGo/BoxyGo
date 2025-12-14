from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('boxygo_localization')

    default_slam_params = os.path.join(pkg, 'config', 'slam', 'main_slam.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        slam_node,
    ])