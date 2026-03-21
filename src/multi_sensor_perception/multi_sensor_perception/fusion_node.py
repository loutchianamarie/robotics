"""Step 1 placeholder: future multi-sensor fusion logic will live here."""

import rclpy
from rclpy.node import Node


class FusionNode(Node):
    """Placeholder fusion node."""

    def __init__(self) -> None:
        super().__init__('fusion_node')
        self.get_logger().info('Fusion node started (Step 1 placeholder).')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
