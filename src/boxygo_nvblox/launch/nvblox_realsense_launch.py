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

from typing import Optional, List
from launch_ros.actions import Node

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


EMITTER_FLASHING_CONFIG_FILE_PATH = lu.get_path(
    'nvblox_examples_bringup',
    'config/sensors/realsense_emitter_flashing.yaml')

EMITTER_ON_CONFIG_FILE_PATH = lu.get_path(
    'nvblox_examples_bringup',
    'config/sensors/realsense_emitter_on.yaml')


def get_camera_node(camera_name: str, config_file_path: str,
                    serial_number: Optional[str] = None) -> ComposableNode:
    parameters = [config_file_path, {'camera_name': camera_name}]
    if serial_number:
        parameters.append({'serial_no': str(serial_number)})
    return ComposableNode(
        namespace=camera_name,
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=parameters)


def get_splitter_node(camera_name: str) -> ComposableNode:
    return ComposableNode(
        namespace=camera_name,
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA',
            'publish_tf' : False
        }],
        remappings=[
            ('input/infra_1', f'/{camera_name}/infra1/image_rect_raw'),
            ('input/infra_1_metadata', f'/{camera_name}/infra1/metadata'),
            ('input/infra_2', f'/{camera_name}/infra2/image_rect_raw'),
            ('input/infra_2_metadata', f'/{camera_name}/infra2/metadata'),
            ('input/depth', f'/{camera_name}/depth/image_rect_raw'),
            ('input/depth_metadata', f'/{camera_name}/depth/metadata'),
            ('input/pointcloud', f'/{camera_name}/depth/color/points'),
            ('input/pointcloud_metadata', f'/{camera_name}/depth/metadata'),
        ])


def add_camera(args: lu.ArgumentContainer) -> List[Action]:
    camera_name = 'camera'
    serial_number = None if args.camera_serial_number == '' else str(args.camera_serial_number)
    run_splitter = args.run_splitter

    if run_splitter:
        config_file_path = EMITTER_FLASHING_CONFIG_FILE_PATH
    else:
        config_file_path = EMITTER_ON_CONFIG_FILE_PATH

    nodes: List[ComposableNode] = []
    nodes.append(get_camera_node(camera_name, config_file_path, serial_number))
    if run_splitter:
        nodes.append(get_splitter_node(camera_name))

    return [
        lu.load_composable_nodes(args.container_name, nodes),
        lu.log_info(f'Starting realsense with name: {camera_name}, running splitter: {run_splitter}')
    ]


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg('camera_serial_number', '')
    args.add_arg('run_splitter', True)

    args.add_opaque_function(add_camera)
    actions = args.get_launch_actions()

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '0.38', '0', '0.225',
            '1.5708', '3.1416', '1.5708',
            'base_link',
            'camera_imu_optical_frame'
        ]
    ))

    actions.append(
        lu.component_container(
            args.container_name,
            condition=IfCondition(lu.is_true(args.run_standalone))))
    
    return LaunchDescription(actions)
