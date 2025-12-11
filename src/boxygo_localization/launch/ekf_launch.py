from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('boxygo_localization')

    default_ekf_params = os.path.join(pkg, 'config', 'ekf', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params_file = LaunchConfiguration('ekf_params_file')


    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_params,
        description='Full path to the ROS2 parameters file to use for the ekf_node'
    )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_ekf_params_file,
        ekf_node
    ])