from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='boxygo_joy2twist',        # Zmień na nazwę Twojej paczki!
            executable='joy2twist',
            name='joy2twist',
            output='screen',
        ),
    ])
