from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boxygo',
            executable='boxygo_stop_moteus',
            name='boxygo_stop_moteus',
            output='screen',
            parameters=[{
                'servo_ids': [1,2,3,4,5,6],
                'method': 'stop',                 
                'topic_prefix': '/boxygo_moteus', 
                'remote_node': '/boxygo_moteus', 
                'disable_remote': True,           

            }]
    )
    ])
