#!/usr/bin/env python3

# Copyright 2026 ASTRO
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_type_arg = DeclareLaunchArgument(
        'backend_type',
        default_value='basilisk',
        description='Bridge backend to run (default: basilisk)',
    )

    max_steps_arg = DeclareLaunchArgument(
        'max_steps',
        default_value='91',
        description='Number of bridge steps to execute (0 = infinite)',
    )

    min_subscribers_arg = DeclareLaunchArgument(
        'min_subscribers',
        default_value='1',
        description='Minimum number of env_data subscribers before advancing',
    )

    bridge_node = Node(
        package='external_sim_bridge',
        executable='bridge_node',
        name='external_sim_bridge',
        output='screen',
        parameters=[{
            'backend_type': LaunchConfiguration('backend_type'),
            'max_steps': LaunchConfiguration('max_steps'),
            'min_subscribers': LaunchConfiguration('min_subscribers'),
        }],
    )

    gnc_node = Node(
        package='distributed_satellite_sim',
        executable='gnc_node',
        name='gnc_node',
        output='screen',
    )

    return LaunchDescription([
        backend_type_arg,
        max_steps_arg,
        min_subscribers_arg,
        bridge_node,
        gnc_node,
    ])
