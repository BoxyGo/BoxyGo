
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
        # Node(
        #     package='moteus_control',
        #     executable='moteus_node',
        #     name='boxygo_moteus',
        #     namespace='boxygo_moteus',
        #     output='screen',
        #     parameters=[{
        #       'servo.ids': [1],
        #       'servo.update_rate': 50.0,
        #       'servo.default_timeout_s': 0.1,
        #     }],
        # ),
        # Node(
        #     package='boxygo',
        #     executable='wheel_moteus',
        #     name='wheel_moteus',
        #     output='screen',
        # ),
        Node(
            package='moteus_control',
            executable='moteus_node',
            name='boxygo_moteus',
            namespace='boxygo_moteus',
            output='screen',
            parameters=[{
              'servo.ids': [1, 2, 3, 4, 5, 6],
              'servo.update_rate': 50.0,
              'servo.default_timeout_s': 0.1,
            }],
        ),
        Node(
            package='boxygo',
            executable='multi_wheel_moteus',
            name='multi_wheel_moteus',
            output='screen',
            parameters=[{
                'servo_ids': [1, 2, 3, 4, 5, 6],
                'joint_names': [
                    'left_wheel_1_joint',
                    'left_wheel_2_joint',
                    'left_wheel_3_joint',
                    'right_wheel_1_joint',
                    'right_wheel_2_joint',
                    'right_wheel_3_joint'
                ],
                'use_velocity': True,
            }],
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
        ),
    ])