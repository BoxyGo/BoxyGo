# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    isaac_ros_ws = os.environ.get("ISAAC_ROS_WS", "/workspaces/isaac_ros-dev")

    launch_args = [
        DeclareLaunchArgument('input_image_width', default_value='640'),
        DeclareLaunchArgument('input_image_height', default_value='480'),
        DeclareLaunchArgument('network_image_width', default_value='512'),
        DeclareLaunchArgument('network_image_height', default_value='512'),
        DeclareLaunchArgument('encoder_image_mean', default_value='[0.5, 0.5, 0.5]'),
        DeclareLaunchArgument('encoder_image_stddev', default_value='[0.5, 0.5, 0.5]'),
        DeclareLaunchArgument('model_name', default_value='cityscapes'),
        DeclareLaunchArgument(
            'model_repository_paths',
            default_value=f'["{isaac_ros_ws}/segformer_models"]'
        ),
        DeclareLaunchArgument('max_batch_size', default_value='0'),
        DeclareLaunchArgument('input_tensor_names', default_value='["input_tensor"]'),
        DeclareLaunchArgument('input_binding_names', default_value='["input"]'),
        DeclareLaunchArgument(
            'input_tensor_formats',
            default_value='["nitros_tensor_list_nchw_rgb_f32"]'
        ),
        DeclareLaunchArgument('output_tensor_names', default_value='["output_tensor"]'),
        DeclareLaunchArgument('output_binding_names', default_value='["output"]'),
        DeclareLaunchArgument(
            'output_tensor_formats',
            default_value='["nitros_tensor_list_nchw_rgb_f32"]'
        ),
        DeclareLaunchArgument('network_output_type', default_value='argmax'),
        DeclareLaunchArgument('color_segmentation_mask_encoding', default_value='rgb8'),
        DeclareLaunchArgument('mask_width', default_value='512'),
        DeclareLaunchArgument('mask_height', default_value='512'),
    ]

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
    color_segmentation_mask_encoding = LaunchConfiguration(
        'color_segmentation_mask_encoding'
    )
    mask_width = LaunchConfiguration('mask_width')
    mask_height = LaunchConfiguration('mask_height')

    container_name = 'segformer_container'

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
            'enable_padding': 'True',
            'image_input_topic': '/camera/color/image_raw',
            'camera_info_input_topic': '/camera/color/camera_info',
            'tensor_output_topic': '/tensor_pub',
            'attach_to_shared_component_container': 'True',
            'component_container_name': container_name,
        }.items(),
    )

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

    segformer_decoder_node = ComposableNode(
        name='segformer_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        parameters=[{
            'network_output_type': network_output_type,
            'color_segmentation_mask_encoding': color_segmentation_mask_encoding,
            'mask_width': mask_width,
            'mask_height': mask_height,
        }],
        remappings=[
            ('unet/colored_segmentation_mask', 'segformer/colored_segmentation_mask'),
            ('unet/raw_segmentation_mask', 'segformer/raw_segmentation_mask')
        ]
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[triton_node, segformer_decoder_node],
        output='screen'
    )

    return launch.LaunchDescription(
        launch_args + [container, encoder_node_launch]
    )
