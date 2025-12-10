from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, SetRemap
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME
import os


def generate_launch_description():

    navigation_pkg = get_package_share_directory('boxygo_navigation')
    localization_pkg = get_package_share_directory('boxygo_localization')
    moteus_controller_pkg = get_package_share_directory('boxygo_moteus_control')

    # === Pliki konfigów ===
    map_file = os.path.join(localization_pkg, 'maps', 'map.yaml')
    nav2_params = os.path.join(navigation_pkg, 'config', 'nav2_nvblox_params.yaml')
    ekf_config = os.path.join(localization_pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # === IMU filter (Madgwick) – jak w Twoim działającym launchu ===
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
            ("/imu/data_raw", "/camera/imu")  # Realsense IMU
        ]
    )

    # === EKF (robot_localization) ===
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # === Lidar ===
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    # === Moteus (napęd) ===
    moteus_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moteus_controller_pkg,
                'launch',
                'moteus_controller_launch.py'
            )
        )
    )

    # === Nvblox container (ten sam co NVIDIA: NVBLOX_CONTAINER_NAME) ===
    nvblox_container = ComposableNodeContainer(
        name=NVBLOX_CONTAINER_NAME,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )

    # === RealSense + splitter (Twoje nav2_realsense_launch.py) ===
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg,
                'launch',
                'nav2_realsense_launch.py'
            )
        )
    )

    # === cuVSLAM (Twoje nav2_vslam_nvblox_launch.py) ===
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg,
                'launch',
                'nav2_vslam_nvblox_launch.py'
            )
        )
    )

    # === NVBLOX (Twoje nav2_nvblox_launch.py) ===
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_pkg,
                'launch',
                'nav2_nvblox_launch.py'
            )
        )
    )

    # === Nav2 (bringup_launch.py) – dokładnie ten sam pattern co w działającym launchu ===
    nav2_node = TimerAction(
        period=5.0,   # dajemy czas, żeby kamera / vslam / nvblox wstały
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_cont/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    )
                ),
                launch_arguments={
                    'params_file': nav2_params,
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        imu_node,
        ekf_node,
        lidar_launch,
        moteus_controller_launch,
        nvblox_container,
        realsense_launch,
        vslam_launch,
        nvblox_launch,
        nav2_node,
    ])
