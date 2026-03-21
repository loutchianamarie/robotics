"""Step 1 placeholder: future end-to-end latency metrics will attach here."""

import rclpy
from rclpy.node import Node


class LatencyNode(Node):
    """Placeholder latency evaluation node."""

    def __init__(self) -> None:
        super().__init__('latency_node')
        self.get_logger().info('Latency node started (Step 1 placeholder).')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LatencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
