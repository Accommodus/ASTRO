#!/usr/bin/env python3

# Copyright 2026 Accommodus
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
    max_steps_arg = DeclareLaunchArgument(
        'max_steps',
        default_value='91',
        description='Number of simulation steps (0 = infinite)')

    env_node = Node(
        package='distributed_satellite_sim',
        executable='env_node',
        name='env_node',
        output='screen',
        parameters=[{
            'max_steps': LaunchConfiguration('max_steps'),
        }],
    )

    gnc_node = Node(
        package='distributed_satellite_sim',
        executable='gnc_node',
        name='gnc_node',
        output='screen',
    )

    return LaunchDescription([
        max_steps_arg,
        env_node,
        gnc_node,
    ])
