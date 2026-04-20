"""Launch env_node + qp_gnc_node with QP-MPC dynamics (30 s loop)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('distributed_satellite_sim')
    qp_params = os.path.join(pkg, 'config', 'qp_mpc_params.yaml')

    return LaunchDescription([
        Node(
            package='distributed_satellite_sim',
            executable='env_node',
            name='env_node',
            parameters=[qp_params],
            output='screen',
        ),
        Node(
            package='distributed_satellite_sim',
            executable='qp_gnc_node',
            name='qp_gnc_node',
            output='screen',
        ),
    ])
