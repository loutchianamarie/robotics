"""Step 5: standalone latency monitor for /scan, /camera/image_raw, /imu."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan


class LatencyNode(Node):
    """Computes sensor timestamp-to-callback delay for each stream."""

    def __init__(self) -> None:
        super().__init__('latency_node')
        self.get_logger().info(
            'Latency node started; monitoring /scan, /camera/image_raw, /imu.'
        )
        self._last_log_ns: int | None = None
        self._log_period_ns = 1_000_000_000  # 1 second throttle for readability

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )
        self._camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self._camera_callback, 10
        )
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 10
        )

    @staticmethod
    def _stamp_to_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _should_log(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_log_ns is None or (now_ns - self._last_log_ns) >= self._log_period_ns:
            self._last_log_ns = now_ns
            return True
        return False

    def _log_latency(self, name: str, msg: object) -> None:
        if not self._should_log():
            return
        now_ns = self.get_clock().now().nanoseconds
        sensor_ns = self._stamp_to_ns(msg.header.stamp)
        total_ms = max(0.0, (now_ns - sensor_ns) / 1_000_000.0)
        self.get_logger().info(f'[Latency] {name}: {total_ms:.0f} ms | Total: {total_ms:.0f} ms')

    def _scan_callback(self, msg: LaserScan) -> None:
        self._log_latency('scan', msg)

    def _camera_callback(self, msg: Image) -> None:
        self._log_latency('camera', msg)

    def _imu_callback(self, msg: Imu) -> None:
        self._log_latency('imu', msg)


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
