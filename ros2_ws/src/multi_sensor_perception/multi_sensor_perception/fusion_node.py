"""Step 4: basic LiDAR + camera fusion node."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class FusionNode(Node):
    """Fuses simple LiDAR proximity check with camera activity."""

    def __init__(self) -> None:
        super().__init__('fusion_node')
        self.get_logger().info(
            'Fusion node started; subscribing to /scan and /camera/image_raw.'
        )

        self._camera_seen = False
        self._camera_last_ns: int | None = None
        self._camera_active_timeout_ns = 2_000_000_000  # 2 s freshness window

        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10,
        )
        self._camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._camera_callback,
            10,
        )

    def _camera_callback(self, msg: Image) -> None:
        # We only need to confirm the camera stream is alive.
        _ = msg
        self._camera_seen = True
        self._camera_last_ns = self.get_clock().now().nanoseconds

    def _is_camera_active(self, now_ns: int) -> bool:
        if not self._camera_seen or self._camera_last_ns is None:
            return False
        return (now_ns - self._camera_last_ns) <= self._camera_active_timeout_ns

    def _scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance) and 0.0 <= distance <= 7.0
        ]

        if not valid_ranges:
            return

        self.get_logger().info('Object detected by LiDAR')

        now_ns = self.get_clock().now().nanoseconds
        if self._is_camera_active(now_ns):
            self.get_logger().info('Fusion: Object confirmed')


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
