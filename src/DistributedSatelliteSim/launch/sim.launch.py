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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    max_steps_arg = DeclareLaunchArgument(
        'max_steps',
        default_value='91',
        description='Number of simulation steps (0 = infinite)')

    min_subscribers_arg = DeclareLaunchArgument(
        'min_subscribers',
        default_value='1',
        description='Minimum number of env_data subscribers before advancing')

    enable_telemetry_buffer_arg = DeclareLaunchArgument(
        'enable_telemetry_buffer',
        default_value='false',
        description='Launch the telemetry buffer node')

    env_buffer_capacity_arg = DeclareLaunchArgument(
        'env_buffer_capacity',
        default_value='100',
        description='Number of env_data samples to retain')

    actuation_buffer_capacity_arg = DeclareLaunchArgument(
        'actuation_buffer_capacity',
        default_value='100',
        description='Number of actuation_applied samples to retain')

    log_buffer_capacity_arg = DeclareLaunchArgument(
        'log_buffer_capacity',
        default_value='100',
        description='Number of /rosout log entries to retain')

    # When telemetry_buffer is enabled it also subscribes to env_data, so the
    # startup gate must require at least 2 subscribers (gnc_node + buffer) to
    # prevent the observer-only node from satisfying the gate before gnc_node
    # is ready and altering the closed-loop trajectory.
    effective_min_subscribers = PythonExpression([
        'max(int("', LaunchConfiguration('min_subscribers'), '"), 2)'
        ' if "', LaunchConfiguration('enable_telemetry_buffer'), '" == "true"'
        ' else int("', LaunchConfiguration('min_subscribers'), '")',
    ])

    env_node = Node(
        package='distributed_satellite_sim',
        executable='env_node',
        name='env_node',
        output='screen',
        parameters=[{
            'max_steps': LaunchConfiguration('max_steps'),
            'min_subscribers': effective_min_subscribers,
        }],
    )

    gnc_node = Node(
        package='distributed_satellite_sim',
        executable='gnc_node',
        name='gnc_node',
        output='screen',
    )

    telemetry_buffer_node = Node(
        package='distributed_satellite_sim',
        executable='telemetry_buffer_node',
        name='telemetry_buffer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_telemetry_buffer')),
        parameters=[{
            'env_buffer_capacity': LaunchConfiguration('env_buffer_capacity'),
            'actuation_buffer_capacity': LaunchConfiguration('actuation_buffer_capacity'),
            'log_buffer_capacity': LaunchConfiguration('log_buffer_capacity'),
        }],
    )

    return LaunchDescription([
        max_steps_arg,
        min_subscribers_arg,
        enable_telemetry_buffer_arg,
        env_buffer_capacity_arg,
        actuation_buffer_capacity_arg,
        log_buffer_capacity_arg,
        env_node,
        gnc_node,
        telemetry_buffer_node,
    ])
