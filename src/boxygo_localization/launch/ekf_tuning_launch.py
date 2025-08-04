from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')

    sim_launch_path = path.join(pkg_boxygo, 'launch', 'gazebo_launch.py')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    rviz_node = Node(
            package='rviz2',       
            executable='rviz2',           
            name='rviz2',                    
            output='screen',                 
            arguments=['-d', path.join(pkg_boxygo, 'config', 'ekf_tuning.rviz')] 
        )
    
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        )
    
    teleop_node = Node(
            package='boxygo_joy2twist',      
            executable='joy2twist',
            name='joy2twist',
            output='screen',
        )

    ld = LaunchDescription([
        sim_launch,
        rviz_node,
        joy_node,
        teleop_node
    ])
    
    return ld


