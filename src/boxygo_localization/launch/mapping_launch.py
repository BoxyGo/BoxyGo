from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('boxygo_localization')
    slam_params = os.path.join(pkg, 'config', 'mapping_params.yaml')

    slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',   
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'rviz_slam.rviz')],
        )
    
    return LaunchDescription([
                              rviz_node,
                              slam_node
                            ])