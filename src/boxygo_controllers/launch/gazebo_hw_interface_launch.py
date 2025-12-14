import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_boxygo_controllers = FindPackageShare('boxygo_controllers')
    pkg_boxygo_gazebo = get_package_share_directory('boxygo_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_boxygo_gazebo, 'worlds', 'playground.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_boxygo_controllers, 'launch', 'common_controllers_launch.py'])
        ),
        launch_arguments={
            'sim_mode': 'gazebo',
            'use_sim_time': 'true',
            'start_control_node': 'false' 
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'delivery_robot',
            '-topic', 'robot_description', 
            '-x', '3.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        common_launch,
        spawn_entity
    ])