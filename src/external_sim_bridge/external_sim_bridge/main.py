import rclpy

from external_sim_bridge.bridge_node import spin_bridge


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        spin_bridge()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
