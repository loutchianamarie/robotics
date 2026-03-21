"""Step 3.3: minimal subscriber for /imu (sim or hardware)."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNode(Node):
    """Subscribes to Imu; logs receipts on a light throttle."""

    def __init__(self) -> None:
        super().__init__('imu_node')
        self.get_logger().info(
            'IMU node started; subscribing to /imu (Step 3.3).'
        )
        self._imu_count = 0
        self._last_log_ns: int | None = None
        self._log_period_ns = 2_000_000_000  # at most one summary line every 2 s

        self._subscription = self.create_subscription(
            Imu,
            '/imu',
            self._imu_callback,
            10,
        )

    def _imu_callback(self, msg: Imu) -> None:
        self._imu_count += 1
        now_ns = self.get_clock().now().nanoseconds
        if self._last_log_ns is not None:
            if (now_ns - self._last_log_ns) < self._log_period_ns:
                return
        self._last_log_ns = now_ns

        o = msg.orientation
        w = msg.angular_velocity
        a = msg.linear_acceleration
        self.get_logger().info(
            f'Received IMU (count={self._imu_count}): '
            f'orientation xyzw=({o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f}) '
            f'angular_vel xyz=({w.x:.4f},{w.y:.4f},{w.z:.4f}) '
            f'linear_accel xyz=({a.x:.4f},{a.y:.4f},{a.z:.4f})'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
