import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('boxygo')
    # Świat i URDF
    world = os.path.join(pkg, 'worlds', 'street_world.world')
    urdf  = os.path.join(pkg, 'urdf', 'boxygo.urdf.xacro')

    gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
      ),
      launch_arguments={'world': world, 'verbose':'true'}.items()
    )

    rsp = Node(
      package='robot_state_publisher', executable='robot_state_publisher',
      output='screen', parameters=[{'use_sim_time':True}], arguments=[urdf]
    )

    spawn = Node(
      package='gazebo_ros', executable='spawn_entity.py',
      arguments=['-entity','boxyGo','-topic','robot_description','-x','0','-y','0','-z','0.1'],
      output='screen'
    )

    return LaunchDescription([gazebo, rsp, spawn])
