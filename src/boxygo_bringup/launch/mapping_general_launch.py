import os
import datetime
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description():

    controllers_share = get_package_share_directory('boxygo_controllers')
    localization_share = get_package_share_directory('boxygo_localization')
    nvblox_share = get_package_share_directory('boxygo_nvblox')

    
    workspace_dir = '/workspaces/isaac_ros-dev'
    bags_directory = os.path.join(workspace_dir, 'rosbags')
    
    if not os.path.exists(bags_directory):
        try:
            os.makedirs(bags_directory)
        except OSError:
            bags_directory = os.path.expanduser('~/rosbags/boxygo')
            if not os.path.exists(bags_directory):
                os.makedirs(bags_directory)

    try:
        available_bags = [
            d for d in os.listdir(bags_directory) 
            if os.path.isdir(os.path.join(bags_directory, d))
        ]
        available_bags.sort(reverse=True)
    except FileNotFoundError:
        available_bags = []

    available_bags.insert(0, "empty")

    current_time = datetime.datetime.now().strftime("%m%d_%H%M%S")
    
    topics_to_record = [
        '/imu/data',
        '/scan',
        '/camera/imu',
        '/tf',
        '/tf_static',
        '/joint_states',
        '/diff_cont/odom',
        '/tf_static', 
        '/camera/imu', 
        '/camera/color/camera_info', 
        '/camera/color/image_raw',
        '/camera/realsense_splitter_node/output/depth', 
        '/camera/depth/camera_info',
        '/camera/realsense_splitter_node/output/infra_1', 
        '/camera/infra1/camera_info',
        '/camera/realsense_splitter_node/output/infra_2', 
        '/camera/infra2/camera_info'
    ]

    slam_config_dir = os.path.join(localization_share, 'config', 'slam')
    try:
        slam_presets = [f for f in os.listdir(slam_config_dir) if f.endswith('.yaml')]
        slam_presets.sort()
    except FileNotFoundError:
        slam_presets = []

    default_slam = 'main_slam_config.yaml'
    if default_slam not in slam_presets and slam_presets:
        default_slam = slam_presets[0]

    slam_preset_arg = DeclareLaunchArgument(
        'slam_preset',
        default_value=default_slam,
        choices=slam_presets if slam_presets else None,
        description='Select SLAM config file'
    )
    
    slam_config_path = PathJoinSubstitution([
        localization_share, 'config', 'slam', LaunchConfiguration('slam_preset')
    ])

    ekf_config_dir = os.path.join(localization_share, 'config', 'ekf')
    try:
        ekf_presets = [f for f in os.listdir(ekf_config_dir) if f.endswith('.yaml')]
        ekf_presets.sort()
    except FileNotFoundError:
        ekf_presets = []

    default_ekf = 'main_ekf_config.yaml'
    if default_ekf not in ekf_presets and ekf_presets:
        default_ekf = ekf_presets[0]

    ekf_preset_arg = DeclareLaunchArgument(
        'ekf_preset',
        default_value=default_ekf,
        choices=ekf_presets if ekf_presets else None,
        description='Select EKF config file'
    )
    
    ekf_config_path = PathJoinSubstitution([
        localization_share, 'config', 'ekf', LaunchConfiguration('ekf_preset')
    ])


    mode_arg = DeclareLaunchArgument(
        'mode_platform',
        default_value='real',
        choices=['real', 'isaac', 'gazebo'],
        description='Select operation mode (hardware interface)'
    )

    rosbag_mode_arg = DeclareLaunchArgument(
        'rosbag_mode',
        default_value='none',
        choices=['none', 'record_all', 'record_sensors', 'play'],
        description='Select rosbag operation mode'
    )

    rosbag_file_arg = DeclareLaunchArgument(
        'rosbag_file',
        default_value="empty",
        choices=available_bags,
        description='Select bag file (ONLY used for Play mode, ignore for Record)'
    )


    mode_config = LaunchConfiguration('mode_platform')
    rosbag_mode = LaunchConfiguration('rosbag_mode')
    rosbag_file = LaunchConfiguration('rosbag_file')

    use_sim_time_val = PythonExpression([
        "'True' if '", rosbag_mode, "' == 'play' else ('False' if '", mode_config, "' == 'real' else 'True')"
    ])

    should_launch_hw = UnlessCondition(
        PythonExpression(["'", rosbag_mode, "' == 'play'"])
    )

    should_launch_algo = UnlessCondition(
        PythonExpression(["'", rosbag_mode, "' == 'record_sensors'"])
    )

    bag_output_path = PathJoinSubstitution([
        bags_directory,
        PythonExpression([
            "'rosbag_{}_{}_{}'.format('", rosbag_mode, "', '", mode_config, "', '", current_time, "')"
        ])
    ])

    hw_interface_filename = PythonExpression(["'", mode_config, "_hw_interface_launch.py'"])
    hw_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([controllers_share, 'launch', hw_interface_filename])
        ),
        launch_arguments={'use_sim_time': use_sim_time_val}.items(),
        condition=should_launch_hw
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_share, 'launch', 'slam_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time_val,
            'slam_params_file': slam_config_path
        }.items(),
        condition=should_launch_algo
    )

    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nvblox_share, 'launch', 'nvblox_vslam_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time_val
        }.items(),
        condition=should_launch_algo
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nvblox_share, 'launch', 'nvblox_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time_val
        }.items(),
        condition=should_launch_algo
    )

    nvblox_container = ComposableNodeContainer(
        name=NVBLOX_CONTAINER_NAME,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_share, 'launch', 'ekf_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time_val,
            'ekf_params_file': ekf_config_path
        }.items(),
        condition=should_launch_algo
    )

    record_all_action = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_output_path],
        output='screen',
        condition=IfCondition(PythonExpression(["'", rosbag_mode, "' == 'record_all'"]))
    )

    record_sensors_cmd = ['ros2', 'bag', 'record', '-o', bag_output_path] + topics_to_record
    record_sensors_action = ExecuteProcess(
        cmd=record_sensors_cmd,
        output='screen',
        condition=IfCondition(PythonExpression(["'", rosbag_mode, "' == 'record_sensors'"]))
    )

    play_action = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', PathJoinSubstitution([bags_directory, rosbag_file]), '--clock'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", rosbag_mode, "' == 'play'"]))
    )
    
    log_info = LogInfo(
        msg=PythonExpression(["'URUCHAMIANIE W TRYBIE: ' + '", rosbag_mode, "'"])
    )

    return LaunchDescription([
        slam_preset_arg,
        ekf_preset_arg,
        rosbag_mode_arg,
        rosbag_file_arg,
        mode_arg,
   

        log_info,
        nvblox_container,
        slam_launch,
        ekf_launch,
        hw_interface_launch,
        vslam_launch,
        nvblox_launch,
        
        record_all_action,
        record_sensors_action,
        play_action
    ])