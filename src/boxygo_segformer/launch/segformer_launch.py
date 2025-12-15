from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    isaac_ros_ws = os.environ.get("ISAAC_ROS_WS", "/workspaces/isaac_ros-dev")

    launch_fragments_arg = DeclareLaunchArgument(
        "launch_fragments",
        default_value="realsense_mono_rect,segformer",
        description="Launch fragments to include"
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="cityscapes",
        description="SegFormer model name"
    )

    model_repository_paths_arg = DeclareLaunchArgument(
        "model_repository_paths",
        default_value=f"[{isaac_ros_ws}/segformer/models]",
        description="List of model repository paths"
    )

    network_image_width_arg = DeclareLaunchArgument(
        "network_image_width",
        default_value="224",
        description="Input width for the SegFormer network"
    )

    network_image_height_arg = DeclareLaunchArgument(
        "network_image_height",
        default_value="224",
        description="Input height for the SegFormer network"
    )

    segformer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("isaac_ros_examples"),
                "launch",
                "isaac_ros_examples.launch.py"
            )
        ),
        launch_arguments={
            "launch_fragments":       LaunchConfiguration("launch_fragments"),
            "model_name":             LaunchConfiguration("model_name"),
            "model_repository_paths": LaunchConfiguration("model_repository_paths"),
            "network_image_width":    LaunchConfiguration("network_image_width"),
            "network_image_height":   LaunchConfiguration("network_image_height"),
        }.items()
    )

    return LaunchDescription([
        launch_fragments_arg,
        model_name_arg,
        model_repository_paths_arg,
        network_image_width_arg,
        network_image_height_arg,
        segformer_launch,
    ])
