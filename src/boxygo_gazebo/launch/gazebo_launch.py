from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_boxygo_description = get_package_share_directory('boxygo_description')
    pkg_boxygo_controllers = get_package_share_directory('boxygo_controllers')
    pkg_boxygo_gazebo = get_package_share_directory('boxygo_gazebo')

    urdf_path = os.path.join(pkg_boxygo_description, 'urdf', 'luksusowy.urdf.xacro')
    controller_config = os.path.join(pkg_boxygo_controllers, 'config', 'diff_drive_controller.yaml')
    world_path = os.path.join(pkg_boxygo_gazebo, 'worlds', 'asphalt.world')

    # Jeden argument dla całego launcha
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # ten launch jest pod Gazebo, więc domyślnie true
        description='Use simulation (Gazebo) clock if true'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'use_sim_time': use_sim_time
        }.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path, ' sim_mode:=gazebo']),
            'use_sim_time': use_sim_time
        }]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'delivery_robot',
            '-topic', 'robot_description',
            '-x', '3.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    controllers = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont', '--controller-manager', 'controller_manager', '--param-file', controller_config],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,   # najpierw deklaracja argumentu
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        controllers,
    ])
