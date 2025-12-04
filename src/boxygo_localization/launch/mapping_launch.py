from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('boxygo_localization')
    slam_params = os.path.join(pkg, 'config', 'mapping_params.yaml')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params,
        {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    imu_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    name='imu_filter_madgwick',
    output='screen',
    parameters=[
        {"use_mag": False},
        {"fixed_frame": "camera_link"},   # dodany parametr
        {'use_sim_time': use_sim_time}
    ],
    remappings=[
        ("/imu/data_raw", "/camera/imu")
    ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params,
        {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        slam_node,
        ekf_node,
        imu_node
    ])
