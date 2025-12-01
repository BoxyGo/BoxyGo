from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():

    config = os.path.join(
        os.path.dirname(__file__),    
        '..',                      
        'config',                     
        'joy2twist.yaml'             
    )

    config = os.path.normpath(config)
    
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='boxygo_joy2twist',   
            executable='joy2twist',
            name='joy2twist',
            output='screen',
            parameters=[config]
        )
    ])
