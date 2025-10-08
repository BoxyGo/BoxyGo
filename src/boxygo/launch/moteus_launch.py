
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_boxygo = get_package_share_directory('boxygo')
    gazebo_launch = os.path.join(pkg_boxygo, 'launch', 'gazebo_launch.py')
    keyboard_launch = os.path.join(pkg_boxygo, 'launch', 'keyboard_launch.py')

    return LaunchDescription([
        
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch)
        ),
        # IncludeLaunchDescription(
        #          PythonLaunchDescriptionSource(keyboard_launch)
        # ),
        Node(
            package='moteus_control',
            executable='moteus_node',
            name='boxygo_moteus',
            namespace='boxygo_moteus',
            output='screen',
            parameters=[{
              'ids': [1,2,3,4,5,6],
            }],
        ),
        Node(
            package='boxygo',
            executable='boxygo_moteus',
            name='boxygo_moteus',
            output='screen',
            parameters=[{
                'servo_ids': [1,2,3,4,5,6],
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
        Node(
            package='boxygo',
            executable='boxygo_moteus_diagnostic',
            name='boxygo_moteus_diagnostic',
            output='screen',
            parameters=[{
                'servo_ids': [1, 2, 3, 4, 5, 6],
                'cmd_topic': '/diff_cont/cmd_vel_out',
                'cmd_prefix': '/boxygo_moteus/id_',
                'state_prefix': '/boxygo_moteus/id_',
                'report_period_s': 1.0,
                'stats_window_s': 5.0,
                'warn_cmd_vel_hz_min': 20.0,
                'warn_can_hz_min': 20.0,
                'heartbeat_timeout_s': 0.3,
                'v_warn': 21.0,
                'v_error': 20.0,
                'motor_temp_warn': 80.0,
                'motor_temp_error': 95.0,
                'can_interface': 'can0', # check "ip link show" in terminal
                'snapshot_on_incident': True,
                'enabled': True,
            }],
        ),
    ])