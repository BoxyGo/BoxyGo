from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME
import os


def generate_launch_description():

    navigation_pkg = get_package_share_directory('boxygo_navigation')

    nav2_params = os.path.join(
        navigation_pkg, 'config', 'nav2_nvblox_params.yaml'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg, 'launch', 'nav2_realsense_launch.py'
            )
        )
    )

    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg, 'launch', 'nav2_vslam_nvblox_launch.py'
            )
        )
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg, 'launch', 'nav2_nvblox_launch.py'
            )
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'False',
            'use_composition': 'True',
            'container_name': 'navigation_container',
        }.items()
    )

    nvblox_container = ComposableNodeContainer(
        name=NVBLOX_CONTAINER_NAME,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )

    return LaunchDescription([
        nvblox_container,
        realsense_launch,
        vslam_launch,
        nvblox_launch,
        nav2_launch,
    ])
