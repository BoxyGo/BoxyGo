from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    imu_remap_node = Node(
        package="boxygo_localization",
        executable="realsense_imu_remap",
        name="realsense_imu_remap",
        output="screen",
        parameters=[{
            "input_topic": "camera/imu",
            "output_topic": "imu/corrected"  
            }]
    )

    return LaunchDescription([
        imu_remap_node
    ])
