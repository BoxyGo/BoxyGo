from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node  

def generate_launch_description():
  
    pkg_boxygo_controllers = FindPackageShare('boxygo_controllers')


    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_boxygo_controllers,          
                'launch', 
                'common_controllers_launch.py'         
            ])
        ),
        launch_arguments={
            'sim_mode': 'isaac',
            'use_sim_time': 'true',
            'start_control_node': 'true'
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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '0.38', '0', '0.225',
            '0', '0', '0',
            'base_link',
            'camera_imu_optical_frame'
        ]
    )

    return LaunchDescription([
        common_launch,
        imu_node,
        static_tf
    ])