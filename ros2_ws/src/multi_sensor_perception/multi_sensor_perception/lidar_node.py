"""Step 3.2: minimal subscriber for /scan (sim or hardware)."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarNode(Node):
    """Subscribes to LaserScan; logs receipts on a light throttle."""

    def __init__(self) -> None:
        super().__init__('lidar_node')
        self.get_logger().info(
            'LiDAR node started; subscribing to /scan (Step 3.2).'
        )
        self._scan_count = 0
        self._last_log_ns: int | None = None
        self._log_period_ns = 2_000_000_000  # at most one summary line every 2 s

        self._subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10,
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_count += 1
        now_ns = self.get_clock().now().nanoseconds
        if self._last_log_ns is not None:
            if (now_ns - self._last_log_ns) < self._log_period_ns:
                return
        self._last_log_ns = now_ns

        filtered_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance) and 0.0 <= distance <= 7.0
        ]
        n_ranges = len(msg.ranges)
        self.get_logger().info(
            f'Received scan (count={self._scan_count}): '
            f'angle_min={msg.angle_min:.4f} rad angle_max={msg.angle_max:.4f} rad '
            f'range_count={n_ranges}'
        )

        if filtered_ranges:
            filtered_text = ', '.join(f'{distance:.2f} m' for distance in filtered_ranges)
            self.get_logger().info(
                f'Filtered distances within 0.0-7.0 m ({len(filtered_ranges)}): '
                f'[{filtered_text}]'
            )
        else:
            self.get_logger().info('No obstacle detected within 7 meters')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
