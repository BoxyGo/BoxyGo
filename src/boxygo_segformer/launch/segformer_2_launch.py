import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch RealSense + DNN Image Encoder + Triton + SegFormer + PointCloud Proc."""

    isaac_ros_ws = os.environ.get("ISAAC_ROS_WS", "/workspaces/isaac_ros-dev")

    launch_args = [
        # --- USTAWIENIA WEJŚCIA (RealSense) ---
        # POPRAWIONE: Domyślna wysokość ustawiona na 360
        DeclareLaunchArgument(
            'input_image_width',
            default_value='640',
            description='The input image width (camera)'),
        DeclareLaunchArgument(
            'input_image_height',
            default_value='360',  # ZMIANA z 380 na 360
            description='The input image height (camera)'),

        # --- USTAWIENIA SIECI (SegFormer) ---
        # SegFormer (Cityscapes) zazwyczaj oczekuje 224x224, 512x512 lub 1024x1024
        DeclareLaunchArgument(
            'network_image_width',
            default_value='224',
            description='The input image width that the network expects'),
        DeclareLaunchArgument(
            'network_image_height',
            default_value='224',
            description='The input image height that the network expects'),

        DeclareLaunchArgument(
            'encoder_image_mean',
            default_value='[0.5, 0.5, 0.5]',
            description='The mean for image normalization'),
        DeclareLaunchArgument(
            'encoder_image_stddev',
            default_value='[0.5, 0.5, 0.5]',
            description='The standard deviation for image normalization'),

        # --- TRITON MODEL ---
        DeclareLaunchArgument(
            'model_name',
            default_value='cityscapes',
            description='The name of the model (folder name in the Triton repo)'),
        DeclareLaunchArgument(
            'model_repository_paths',
            default_value=f'["{isaac_ros_ws}/segformer_models"]',
            description='The absolute path(s) to the repository of models'),
        DeclareLaunchArgument(
            'max_batch_size',
            default_value='0',
            description='The maximum allowed batch size of the model'),
        DeclareLaunchArgument(
            'input_tensor_names',
            default_value='["input_tensor"]',
            description='A list of tensor names to bind to the specified input binding names'),
        DeclareLaunchArgument(
            'input_binding_names',
            default_value='["input"]',
            description='A list of input tensor binding names (specified by model)'),
        DeclareLaunchArgument(
            'input_tensor_formats',
            default_value='["nitros_tensor_list_nchw_rgb_f32"]',
            description='The nitros format of the input tensors'),
        DeclareLaunchArgument(
            'output_tensor_names',
            default_value='["output_tensor"]',
            description='A list of tensor names to bind to the specified output binding names'),
        DeclareLaunchArgument(
            'output_binding_names',
            default_value='["output"]',
            description='A list of output tensor binding names (specified by model)'),
        DeclareLaunchArgument(
            'output_tensor_formats',
            default_value='["nitros_tensor_list_nchw_rgb_f32"]',
            description='The nitros format of the output tensors'),

        # --- DECODER & VISUALIZATION ---
        DeclareLaunchArgument(
            'network_output_type',
            default_value='argmax',
            description='The output type that the network provides (softmax, sigmoid or argmax)'),
        DeclareLaunchArgument(
            'color_segmentation_mask_encoding',
            default_value='rgb8',
            description='The image encoding of the colored segmentation mask (rgb8 or bgr8)'),
    ]

    # Pobieranie konfiguracji
    input_image_width = LaunchConfiguration('input_image_width')
    input_image_height = LaunchConfiguration('input_image_height')
    network_image_width = LaunchConfiguration('network_image_width')
    network_image_height = LaunchConfiguration('network_image_height')
    encoder_image_mean = LaunchConfiguration('encoder_image_mean')
    encoder_image_stddev = LaunchConfiguration('encoder_image_stddev')

    model_name = LaunchConfiguration('model_name')
    model_repository_paths = LaunchConfiguration('model_repository_paths')
    max_batch_size = LaunchConfiguration('max_batch_size')
    input_tensor_names = LaunchConfiguration('input_tensor_names')
    input_binding_names = LaunchConfiguration('input_binding_names')
    input_tensor_formats = LaunchConfiguration('input_tensor_formats')
    output_tensor_names = LaunchConfiguration('output_tensor_names')
    output_binding_names = LaunchConfiguration('output_binding_names')
    output_tensor_formats = LaunchConfiguration('output_tensor_formats')

    network_output_type = LaunchConfiguration('network_output_type')
    color_segmentation_mask_encoding = LaunchConfiguration('color_segmentation_mask_encoding')

    container_name = 'segformer_container'

    # 1. REALSENSE CAMERA NODE
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        parameters=[{
            # POPRAWIONE: Wymuszamy profil 640x360x30
            'depth_module.profile': '640x360x30', 
            'rgb_camera.profile': '640x360x30',
            'align_depth.enable': True,
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_sync': True,
        }],
        output='screen'
    )

    # 2. DNN IMAGE ENCODER (Resize + Normalize)
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    encoder_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
        ),
        launch_arguments={
            'input_image_width': input_image_width,
            'input_image_height': input_image_height,
            'network_image_width': network_image_width,
            'network_image_height': network_image_height,
            'image_mean': encoder_image_mean,
            'image_stddev': encoder_image_stddev,
            'enable_padding': 'False',
            'image_input_topic': '/camera/color/image_raw',
            'camera_info_input_topic': '/camera/color/camera_info',
            'tensor_output_topic': '/tensor_pub',
            'attach_to_shared_component_container': 'True',
            'component_container_name': container_name,
        }.items(),
    )

    # 3. TRITON INFERENCE NODE
    triton_node = ComposableNode(
        name='triton_node',
        package='isaac_ros_triton',
        plugin='nvidia::isaac_ros::dnn_inference::TritonNode',
        parameters=[{
            'model_name': model_name,
            'model_repository_paths': model_repository_paths,
            'max_batch_size': max_batch_size,
            'input_tensor_names': input_tensor_names,
            'input_binding_names': input_binding_names,
            'input_tensor_formats': input_tensor_formats,
            'output_tensor_names': output_tensor_names,
            'output_binding_names': output_binding_names,
            'output_tensor_formats': output_tensor_formats,
        }]
    )

    # 4. SEGFORMER DECODER (Tensor -> Color Mask)
    # Dekoder automatycznie weźmie height 360 z LaunchConfiguration
    segformer_decoder_node = ComposableNode(
        name='segformer_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        parameters=[{
            'network_output_type': network_output_type,
            'color_segmentation_mask_encoding': color_segmentation_mask_encoding,
            # Skalowanie maski z powrotem do 640x360, żeby pasowała do głębi
            'mask_width': input_image_width, 
            'mask_height': input_image_height,
            'color_palette': [
                0x804080, 0xF423E4, 0x464646, 0x66669C, 0xBE9999, 0x999999, 0xFAAA1E,
                0xDCDC00, 0x6B8E23, 0x98FB98, 0x4682B4, 0x1E1E46, 0x3C1414, 0x00008E,
                0x000046, 0x003C64, 0x005064, 0x0000E6, 0x770B20
            ],
        }],
        remappings=[
            ('unet/colored_segmentation_mask', 'segformer/colored_segmentation_mask'),
            ('unet/raw_segmentation_mask', 'segformer/raw_segmentation_mask')
        ]
    )

    # 5. POINT CLOUD XYZRGB NODE
    pointcloud_node = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzrgbNode',
        name='point_cloud_xyzrgb_node',
        remappings=[
            ('rgb/image_rect_color', '/segformer/colored_segmentation_mask'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth_registered/image_rect', '/camera/aligned_depth_to_color/image_raw'),
            ('points', '/segformer/semantic_pointcloud')
        ]
    )

    # Kontener
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            triton_node, 
            segformer_decoder_node,
            pointcloud_node 
        ],
        output='screen'
    )

    return launch.LaunchDescription(launch_args + [realsense_camera_node, container, encoder_node_launch])