from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    boxygo_navigation_pkg = get_package_share_directory('boxygo_navigation')
    boxygo_localization_pkg = get_package_share_directory('boxygo_localization')
    
    map_file = os.path.join(boxygo_localization_pkg, 'maps', 'playground_map.yaml') # small_city_map.yaml / playground_map.yaml
    params_file = os.path.join(boxygo_navigation_pkg, 'config', 'nav2_params_h_2.yaml')
    ekf_config = os.path.join(boxygo_localization_pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # dla navigation_gazebo
        description='Use simulation time if true'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    nav2_node = TimerAction(
        period=5.0,
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_cont/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                ),
                launch_arguments={'params_file': params_file, 'map': map_file, 'use_sim_time': use_sim_time, 'autostart': 'true'}.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
        nav2_node
    ])