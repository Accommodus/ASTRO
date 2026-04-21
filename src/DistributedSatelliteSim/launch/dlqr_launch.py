"""Launch env_node + dlqr_gnc_node with DLQR dynamics (100 ms loop)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('distributed_satellite_sim')
    dlqr_params = os.path.join(pkg, 'config', 'dlqr_params.yaml')

    return LaunchDescription([
        Node(
            package='distributed_satellite_sim',
            executable='env_node',
            name='env_node',
            parameters=[dlqr_params],
            output='screen',
        ),
        Node(
            package='distributed_satellite_sim',
            executable='dlqr_gnc_node',
            name='dlqr_gnc_node',
            output='screen',
        ),
    ])
