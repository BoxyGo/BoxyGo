from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')
    # This launch file referenced a URDF that does not exist. Use the
    # available robot description instead.
    urdf_path = os.path.join(pkg_boxygo, 'urdf', '6_wheel_robot.urdf.xacro')

    # Pusty świat z Gazebo (domyślny empty.world, możesz podać swój jeśli masz)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        # argument 'world' pomijamy, wtedy ładuje domyślny pusty świat
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'delivery_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_pub,
        spawn_entity,
    ])
