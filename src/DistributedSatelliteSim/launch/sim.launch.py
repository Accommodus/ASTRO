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

    return LaunchDescription([
        max_steps_arg,
        env_node,
    ])
