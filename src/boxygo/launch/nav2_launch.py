from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')
    map_file = os.path.join(pkg_boxygo, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_boxygo, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_boxygo, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        # Node(
        #     package='slam_toolbox',
        #     executable='localization_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[
        #         slam_params,
        #         {'use_sim_time': True}
        #     ],
        #     remappings=[
        #         ('scan', '/scan'),
        #         ('odom', '/odometry/filtered'),
        #     ],
        # ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': True}
            ],
            remappings=[
                ('scan', '/scan'),  # jeśli Twój lidar publikuje na innym topicu, zmień
                ('odom', 'odom'),
            ],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'amcl',
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'bt_navigator'
                ]
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_boxygo, 'config', 'rviz#1.rviz')],
            parameters=[{'use_sim_time': True}]
        ),
    ])
