
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')
    gazebo_launch = os.path.join(pkg_boxygo, 'launch', 'gazebo_launch.py')

    return LaunchDescription([
        
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch)
        ),
        Node(
            package='moteus_control',
            executable='moteus_node',
            name='mjbots_moteus',
            namespace='mjbots_moteus',
            output='screen',
            parameters=[{
              'ids': [7,2,3,4,5,6],
            }],
        ),
        Node(
            package='boxygo',
            executable='boxygo_moteus',
            name='boxygo_moteus',
            output='screen',
            parameters=[{
                'servo_ids': [7,2,3,4,5,6],
                'joint_names': [
                    'left_wheel_1_joint',
                    'left_wheel_2_joint',
                    'left_wheel_3_joint',

                    'right_wheel_1_joint',
                    'right_wheel_2_joint',
                    'right_wheel_3_joint'
                ],
                'wheel_radius': 0.085,
                'wheel_separation': 0.55,
                'cmd_topic': '/diff_cont/cmd_vel_out',
                'loop_rate_hz': 60.0,
                'timeout_ms': 100,
            }],
        ),
    ])