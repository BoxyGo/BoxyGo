from typing import List, Tuple

from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu
from launch.actions import LogInfo

from launch.substitutions import LaunchConfiguration

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

def get_realsense_remappings(mode: NvbloxMode) -> List[Tuple[str, str]]:
    remappings: List[Tuple[str, str]] = []

    remappings.append(('camera_0/depth/image', '/camera/realsense_splitter_node/output/depth'))
    remappings.append(('camera_0/depth/camera_info', '/camera/depth/camera_info'))

    if mode is NvbloxMode.people_segmentation:
        remappings.append(('camera_0/color/image', '/camera/segmentation/image_resized'))
        remappings.append(
            ('camera_0/color/camera_info', '/camera/segmentation/camera_info_resized'))
        remappings.append(('camera_0/mask/image', '/camera/segmentation/people_mask'))
        remappings.append(
            ('camera_0/mask/camera_info', '/camera/segmentation/camera_info_resized'))
    else:
        remappings.append(('camera_0/color/image', '/camera/color/image_raw'))
        remappings.append(('camera_0/color/camera_info', '/camera/color/camera_info'))

        if mode is NvbloxMode.people_detection:
            remappings.append(('camera_0/mask/image', '/camera/detection/people_mask'))
            remappings.append(
                ('camera_0/mask/camera_info', '/camera/color/camera_info'))

    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:
    mode = NvbloxMode[args.mode]

    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')

    custom_params = lu.get_path(
        'boxygo_nvblox', 'config/nvblox_nav2_params.yaml')

    if mode is NvbloxMode.static:
        mode_config = {}
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    remappings = get_realsense_remappings(mode)
    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(custom_params)
    parameters.append({'use_sim_time': use_sim_time})
    parameters.append({'num_cameras': 1})
    parameters.append({'use_lidar': False})
    parameters.append({
        'map_clearing_frame_id': 'camera_link',
        'esdf_slice_bounds_visualization_attachment_frame_id': 'camera_link'
    })
   
    all_params = f"NVBlox full parameters list: {parameters}"

    actions: List[Action] = []
    actions.append(lu.log_info([all_params]))

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(
            ["Starting nvblox with the 'realsense' camera in '",
             str(mode), "' mode."]))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode', 'static')
    args.add_arg('lidar', 'False')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg('use_sim_time', 'False')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
