from dataclasses import dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class BridgeConfig:
    backend_type: str
    max_steps: int
    min_subscribers: int
    timer_period_sec: float = 0.1


def load_bridge_config(node: Node) -> BridgeConfig:
    node.declare_parameter('backend_type', 'fake')
    node.declare_parameter('max_steps', 91)
    node.declare_parameter('min_subscribers', 0)

    config = BridgeConfig(
        backend_type=node.get_parameter('backend_type').get_parameter_value().string_value,
        max_steps=node.get_parameter('max_steps').get_parameter_value().integer_value,
        min_subscribers=node.get_parameter('min_subscribers').get_parameter_value().integer_value,
    )

    if config.max_steps < 0:
        raise ValueError('max_steps must be greater than or equal to 0')
    if config.min_subscribers < 0:
        raise ValueError('min_subscribers must be greater than or equal to 0')

    return config
